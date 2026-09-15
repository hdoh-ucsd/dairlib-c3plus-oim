"""Orchestrate configuration, native execution and result packaging for one run."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import yaml

from c3plus.configs import (BINARIES, MODELS, OBSTACLE_COSTS,
                           REPO, RUN_OBJECTS, SCENES, environment, planner_environment,
                           write_demo_configs)
from c3plus.configs.catalog import canonical_task, canonical_object, evaluation_config
from c3plus.utils.plan import DEFAULT_WALL_CAP_SECONDS, plan_run
from c3plus.runtime.processes import classify_failure, logged_command, check_experiment_capabilities
from c3plus.runtime.provenance import capture_source_state, runtime_versions
from c3plus.evaluation.package import compact_run

def verify_goal_yaw(log, plan):
    """Check the effective native target, rather than just the launch arguments."""
    expected = (plan["goal_yaw_degrees"], *plan["controller_goal"][:2])
    for line in log.read_text(errors="replace").splitlines():
        if not line.startswith("[GOAL-YAW] "):
            continue
        try:
            fields = dict(item.split("=", 1) for item in line.split()[1:])
            observed = tuple(float(fields[key]) for key in ("goal_yaw_degrees", "goal_x", "goal_y"))
        except (ValueError, KeyError):
            continue
        if all(math.isclose(a, b, rel_tol=0, abs_tol=1e-8) for a, b in zip(observed, expected)):
            return True
    return False

def run_one(scene, obstacle_cost, start, goal, out, cap=DEFAULT_WALL_CAP_SECONDS, port=18001,
            goal_pose=None, max_frames=1200, goal_yaw_degrees=None, object_name=None, steps=None,
            record=True, video=True):
    """Run and package one trial.

    ``video=False`` keeps every recording and metric but skips the MP4 replay,
    which is the bulk of the per-trial cost after the cap. ``record=False``
    additionally skips the recorder, so the trial produces only logs: there is
    no telemetry, no result JSON, no metrics and nothing for ``eval`` to read.
    """
    if not record:
        video = False
    plan = plan_run(scene, obstacle_cost, start, goal, out, cap, port, goal_pose, max_frames,
                    goal_yaw_degrees, object_name, steps)
    scene, object_name = plan["scene"], plan["object_name"]
    pose = plan["evaluation_goal"]
    out = Path(out).resolve()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite existing run directory: {out}")
    config = evaluation_config(scene, object_name)
    config["goal"] = list(pose)
    env = environment(obstacle_cost)
    env.update(planner_environment(config))
    binary_dir = REPO / ".build/bin/examples/sampling_c3"
    for name in BINARIES:
        if not os.access(binary_dir / name, os.X_OK):
            raise RuntimeError(f"Build {name} from this checkout first")
    if "franka_sampling_c3_controller" in BINARIES:
        check_experiment_capabilities(REPO)
    cache = REPO / ".cache"
    cache.mkdir(exist_ok=True)
    with (cache / "sampling_c3_run.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another managed run or packaging job is active") from exc
        source_state = capture_source_state(REPO)
        out.mkdir(parents=True, exist_ok=False)  # Never overwrite or wipe a run.
        run_id = plan["run_id"]
        demo = plan["demo"]
        config_path = out / "evaluation_scene_config.yaml"
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        controller_path = write_demo_configs(demo, out / "config", repo=REPO,
                                              goal_yaw_degrees=goal_yaw_degrees,
                                              **({"object_name": object_name} if object_name is not None else {}))
        source_bytes = (json.dumps(source_state, indent=2, allow_nan=False) + "\n").encode("utf-8")
        source_path = out / "config/source_state.json"
        source_path.write_bytes(source_bytes)
        status = {**plan, "runtime": runtime_versions(), "goal": pose,
                  "controller_params_file": str(controller_path),
                  "config_sha256": {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in sorted((out / "config").rglob("*.yaml"))},
                  "execution": "serial", "python": sys.executable,
                  "recording_enabled": record, "video_enabled": video,
                  "worktree_dirty": source_state["worktree_dirty"],
                  "source_state": {"path": "config/source_state.json",
                                   "sha256": hashlib.sha256(source_bytes).hexdigest(),
                                   "size_bytes": len(source_bytes), "base_commit": source_state["base_commit"],
                                   "scope": source_state["scope"]},
                  "sampler_settings": {k: v for k, v in env.items() if k.startswith("SAMPLING_C3_")},
                  "commit": source_state["base_commit"],
                  "binary_sha256": {name: hashlib.sha256((binary_dir / name).read_bytes()).hexdigest()
                                    for name in BINARIES}}
        (out / "runtime_status.json").write_text(json.dumps(status, indent=2) + "\n")
        started = time.monotonic()
        launch = [sys.executable, "-m", "c3plus.runtime.launcher",
                  demo, config["object_channel_substring"], *map(str, pose),
                  str(cap), str(port), str(out), "--controller-params", str(controller_path)]
        if goal_yaw_degrees is not None:
            launch.extend(["--goal-yaw-degrees", str(plan["goal_yaw_degrees"])])
        if steps is not None:
            launch.extend(["--steps", str(steps)])
        if not record:
            launch.append("--no-record")
        rc = logged_command(launch, out / "launcher.log", env)
        # A launcher preflight failure happens before process logs exist.
        # Preserve its actual error without masking it with a missing-file error.
        if rc and not (out / "planner.log").exists():
            status.update(wrapper_rc=rc, simulation_wall_seconds=time.monotonic()-started,
                          failures=[{"process": "launcher", "reason": "preflight_failed"}])
            (out / "runtime_status.json").write_text(json.dumps(status, indent=2) + "\n")
            raise RuntimeError(f"Launcher preflight failed; inspect {out / 'launcher.log'}")
        status.update(wrapper_rc=rc, simulation_wall_seconds=time.monotonic()-started,
                      failures=classify_failure(out))
        status["seed_verified"] = "[SAMPLER-SEED] deterministic seed=42" in (out / "planner.log").read_text(errors="replace")
        if goal_yaw_degrees is not None:
            status["goal_yaw_verified"] = verify_goal_yaw(out / "planner.log", plan)
        status_path = out / "runtime_status.json"
        status_path.write_text(json.dumps(status, indent=2) + "\n")
        recordings = ("steps_raw.jsonl", "state_trace.jsonl") if record else ()
        if rc or not status["seed_verified"] or not status.get("goal_yaw_verified", True) or any(not (out / name).is_file() or not (out / name).stat().st_size
                    for name in recordings):
            raise RuntimeError(f"Invalid/no-data run; preserved logs in {out}")
        if not record:
            # Nothing was recorded, so there is no result to postprocess,
            # render, validate or compact. Keep the logs and stop here.
            print(f"[COMPLETE] {run_id} unrecorded failures={status['failures']} folder={out}", flush=True)
            return status
        # Packaging occurs after simulation cleanup and under the same lock.
        (out / "tmp").mkdir(exist_ok=True)
        env.update(TMPDIR=str(out / "tmp"), MPLCONFIGDIR=str(out / "tmp/matplotlib"), OMP_NUM_THREADS="1",
                   OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
        models = REPO / "examples/sampling_c3/urdf"
        obj, obs = MODELS[scene]
        object_sdf = models / obj
        if object_name is not None:
            simulation = yaml.safe_load((out / "config/simulation.yaml").read_text())
            object_sdf = REPO / simulation["object_model"]
            status["render_object_model"] = str(object_sdf)
        render = [sys.executable, "-m", "c3plus.visualization.render",
                  "--result", str(out / f"{run_id}_result.json"), "--out", str(out / f"{run_id}.mp4"),
                  "--object-sdf", str(object_sdf), "--goal", *map(str, pose),
                  "--title", run_id, "--max-frames", str(max_frames)]
        if obs:
            render.extend(["--obstacle-sdf", str(models / obs)])
        commands = {
            "postprocess": [sys.executable, "-m", "c3plus.evaluation.postprocess",
                            "--run-dir", str(out), "--scene", scene, "--run-id", run_id,
                            "--scene-config", str(config_path), "--demo", demo],
        }
        if video:
            commands["render"] = render
        for phase, command in commands.items():
            print(f"[PACKAGE] {run_id} {phase}", flush=True)
            status[phase + "_rc"] = logged_command(command, out / f"{phase}.log", env)
            status_path.write_text(json.dumps(status, indent=2) + "\n")
            if status[phase + "_rc"]:
                raise RuntimeError(f"{phase} failed; inspect {out / (phase + '.log')}")
        # Commit one validated record before deleting redundant intermediate files.
        print(f"[PACKAGE] {run_id} consolidate and clean", flush=True)
        compact_run(out, run_id, status=status, require_legacy_complete=False,
                    video_required=video)
        print(f"[COMPLETE] {run_id} failures={status['failures']} folder={out}", flush=True)
        return status

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", "--scene", dest="scene", type=canonical_task, choices=SCENES, required=True)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--object", type=canonical_object, choices=RUN_OBJECTS,
                           help="One manipulated object (default: T_shape)")
    selection.add_argument("--objects", nargs="+", type=canonical_object, choices=RUN_OBJECTS,
                           help="Objects to push serially; multiple objects use OUT/object_name")
    parser.add_argument("--obstacle_cost", choices=OBSTACLE_COSTS, default="exponential")
    parser.add_argument("--start", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal-yaw-degrees", type=int, choices=[90, 0, -90],
                        help="Absolute world yaw; preserve the indexed goal position")
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int, default=DEFAULT_WALL_CAP_SECONDS,
                        help=f"Recorder wall-time cap (default {DEFAULT_WALL_CAP_SECONDS} seconds)")
    parser.add_argument("--steps", type=int, help="Maximum actually applied outer policies, including reposition; default unlimited")
    parser.add_argument("--port", type=int, default=18001)
    parser.add_argument("--out", type=Path, required=True, help="New directory; existing dirs are refused")
    parser.add_argument("--max-frames", type=int, default=1200)
    parser.add_argument("--no-video", dest="video", action="store_false",
                        help="Skip MP4 rendering; keep all recordings, metrics and the result JSON")
    parser.add_argument("--no-record", dest="record", action="store_false",
                        help="Skip the recorder entirely: logs only, no telemetry, "
                             "no result JSON and nothing for eval; implies --no-video")
    parser.add_argument("--dry-run", action="store_true", help="Print the resolved run plan without building, running, or writing files")
    args = parser.parse_args(argv)
    try:
        selected = args.objects if args.objects is not None else [args.object or "T_shape"]
        if len(set(selected)) != len(selected):
            raise ValueError("--objects must not contain duplicates")
        options = dict(max_frames=args.max_frames, goal_yaw_degrees=args.goal_yaw_degrees)
        if args.steps is not None:
            options["steps"] = args.steps
        # Recording choices affect execution and packaging only; plan_run
        # resolves the configuration and does not accept them.
        execution_options = dict(record=args.record, video=args.video)
        jobs = [(name, args.out / name if len(selected) > 1 else args.out) for name in selected]
        plans = [plan_run(args.scene, args.obstacle_cost, args.start, args.goal,
                          out, args.cap, args.port, **options,
                          **({"object_name": name} if name is not None else {})) for name, out in jobs]
        if args.dry_run:
            print(json.dumps(plans[0] if len(plans) == 1 else
                             {"execution": "serial", "run_count": len(plans), "runs": plans}, indent=2))
            return
        for _, out in jobs:
            if out.exists():
                raise FileExistsError(f"Refusing to overwrite existing run directory: {out.resolve()}")
        for name, out in jobs:
            run_one(args.scene, args.obstacle_cost, args.start, args.goal, out,
                    args.cap, args.port, **options, **execution_options,
                    **({"object_name": name} if name is not None else {}))
    except (RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(1, f"{exc}\n")

if __name__ == "__main__":
    main()
