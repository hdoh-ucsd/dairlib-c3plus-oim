#!/usr/bin/env python3
"""Run and package one xArm6 exponential/ReLU experiment from this checkout."""
import argparse
import fcntl
import hashlib
import json
import math
import os
import platform
from pathlib import Path
from importlib.metadata import PackageNotFoundError, version
import shlex
import subprocess
import sys
import time

import yaml

if __package__:
    from .catalog import (BINARIES, CONFIG_DIR, MODELS, OBSTACLE_COSTS, REPO,
                          SCENES, TOOL_DIR, demo_name, load_controller_goal,
                          compose_demo_configs, demo_config_digest, planner_environment,
                          write_demo_configs)
else:
    from catalog import (BINARIES, CONFIG_DIR, MODELS, OBSTACLE_COSTS, REPO,
                         SCENES, TOOL_DIR, demo_name, load_controller_goal,
                         compose_demo_configs, demo_config_digest, planner_environment,
                         write_demo_configs)


def environment(obstacle_cost):
    # Isolate settings from previously exported experiment knobs.
    env = {k: v for k, v in os.environ.items() if not k.startswith("SAMPLING_C3_")}
    env.update(PYTHON=sys.executable, SAMPLING_C3_SEED="42",
               SAMPLING_C3_OBSTACLE_MODE="lcs_contact")
    if obstacle_cost == "relu":
        env.update(SAMPLING_C3_RANK_OBS_MODE="relu_footprint",
                   SAMPLING_C3_OBS_RELU_EPS="0.01", SAMPLING_C3_OBS_RELU_W="200")
    return env


def classify_failure(out):
    failures = []
    for process, name in (("controller", "planner.log"), ("simulator", "sim.log"),
                          ("osc", "osc.log")):
        text = (out / name).read_text(errors="replace")
        if "AllFinite(q_v)" in text:
            reason = "AllFinite(q_v)"
        elif "abort: Failure" in text and "CheckForWorkspaceLimitViolations" in text:
            reason = "workspace_limit_assertion"
        elif "SAMPLING_C3_TOPPLE_GUARD tripped" in text:
            reason = "topple_guard"
        elif "terminate called" in text or "Segmentation fault" in text:
            reason = "other_process_failure"
        else:
            continue
        failures.append({"process": process, "reason": reason})
    if not failures and any(s in (out / "launcher.log").read_text(errors="replace")
                            for s in ("Aborted", "Segmentation fault")):
        failures.append({"process": "unknown", "reason": "see_launcher_log"})
    return failures


def logged_command(command, log, env):
    with log.open("w") as stream:
        stream.write(f"[COMMAND] {shlex.join(map(os.fsdecode, command))}\n")
        stream.write(f"[CWD] {shlex.quote(str(REPO))}\n")
        stream.flush()
        return subprocess.run(command, cwd=REPO, env=env, stdout=stream,
                              stderr=subprocess.STDOUT).returncode


def yaw_suffix(degrees):
    """Give each supported absolute goal orientation a distinct run identity."""
    if degrees is None:
        return ""
    if isinstance(degrees, bool) or degrees not in (-90, 0, 90):
        raise ValueError("Goal yaw must be -90, 0, or 90 degrees")
    return "_yaw_" + {-90: "m090", 0: "000", 90: "p090"}[degrees]


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


def plan_run(scene, obstacle_cost, start, goal, out, cap=600, port=18001,
             goal_pose=None, max_frames=1200, goal_yaw_degrees=None):
    """Validate inputs and describe a run without launching or writing files."""
    if scene not in SCENES or obstacle_cost not in OBSTACLE_COSTS:
        raise ValueError("Unknown scene or obstacle cost")
    if not 1 <= start <= 5 or not 1 <= goal <= 5:
        raise ValueError("Start/goal indices must be between 1 and 5")
    if cap <= 0 or not 1024 <= port <= 65535 or max_frames <= 0:
        raise ValueError("Invalid cap, port, or frame count")
    suffix = yaw_suffix(goal_yaw_degrees)
    demo = demo_name(scene, start, goal)
    goal_file, controller_goal = load_controller_goal(demo, repo=REPO)
    resolved = compose_demo_configs(demo, repo=REPO, goal_yaw_degrees=goal_yaw_degrees)
    source_controller_goal = controller_goal
    if goal_yaw_degrees is not None:
        controller_goal = (*controller_goal[:2], math.radians(goal_yaw_degrees))
    x, y, _ = controller_goal
    pose = tuple(goal_pose) if goal_pose is not None else controller_goal
    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise ValueError("Goal pose must contain three finite values")
    # Archived manifests rounded their evaluation yaw values. Evaluation goals
    # must agree with the native target, including any explicit yaw override.
    angle_delta = math.atan2(math.sin(pose[2] - controller_goal[2]),
                             math.cos(pose[2] - controller_goal[2]))
    if math.hypot(pose[0] - x, pose[1] - y) > 1e-4 or abs(angle_delta) > 1e-4:
        raise ValueError("Manifest goal_pose does not match the selected demo goal; "
                         "choose a --goal index from 1 to 5 and an optional --goal-yaw-degrees.")
    plan = {"scene": scene, "obstacle_cost": obstacle_cost, "start": start, "goal_index": goal,
            "run_id": f"{obstacle_cost}_{scene}_s{start:02d}g{goal:02d}{suffix}_seed42",
            "demo": demo, "seed": 42, "controller_goal": controller_goal,
            "start_pose": resolved["simulation"]["q_init_object"],
            "evaluation_goal": pose, "configuration_file": str(goal_file.relative_to(REPO)),
            "configuration_digest": demo_config_digest(demo, repo=REPO, goal_yaw_degrees=goal_yaw_degrees),
            "out": str(Path(out).resolve()), "wall_cap_seconds": cap, "port": port,
            "max_frames": max_frames}
    if goal_yaw_degrees is not None:
        plan.update(goal_yaw_degrees=int(goal_yaw_degrees), source_controller_goal=source_controller_goal)
    return plan


def runtime_versions():
    packages = {}
    for name in ("drake", "numpy", "scipy", "matplotlib", "Pillow", "trimesh", "PyYAML"):
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            packages[name] = None
    return {"python": sys.version, "platform": platform.platform(), "packages": packages,
            "container_image": os.environ.get("C3PLUS_CONTAINER_IMAGE"),
            "container_image_id": os.environ.get("C3PLUS_CONTAINER_IMAGE_ID")}


def run_one(scene, obstacle_cost, start, goal, out, cap=600, port=18001,
            goal_pose=None, max_frames=1200, goal_yaw_degrees=None):
    plan = plan_run(scene, obstacle_cost, start, goal, out, cap, port, goal_pose, max_frames,
                    goal_yaw_degrees)
    pose = plan["evaluation_goal"]
    out = Path(out).resolve()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite existing run directory: {out}")
    config = yaml.safe_load((CONFIG_DIR / f"{scene}.yaml").read_text())
    config["goal"] = list(pose)
    env = environment(obstacle_cost)
    env.update(planner_environment(config))
    binary_dir = REPO / "bazel-bin/examples/sampling_c3"
    for name in BINARIES:
        if not os.access(binary_dir / name, os.X_OK):
            raise RuntimeError(f"Build {name} from this checkout first")
    cache = REPO / ".cache"
    cache.mkdir(exist_ok=True)
    with (cache / "sampling_c3_run.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Another managed run or packaging job is active") from exc
        out.mkdir(parents=True, exist_ok=False)  # Never overwrite or wipe a run.
        run_id = plan["run_id"]
        demo = plan["demo"]
        config_path = out / "evaluation_scene_config.yaml"
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        controller_path = write_demo_configs(demo, out / "config", repo=REPO,
                                              goal_yaw_degrees=goal_yaw_degrees)
        status = {**plan, "runtime": runtime_versions(), "goal": pose,
                  "controller_params_file": str(controller_path),
                  "config_sha256": {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in sorted((out / "config").rglob("*.yaml"))},
                  "execution": "serial", "python": sys.executable,
                  "worktree_dirty": bool(subprocess.check_output(
                      ["git", "status", "--porcelain", "--untracked-files=no"], cwd=REPO)),
                  "sampler_settings": {k: v for k, v in env.items() if k.startswith("SAMPLING_C3_")},
                  "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
                  "binary_sha256": {name: hashlib.sha256((binary_dir / name).read_bytes()).hexdigest()
                                    for name in BINARIES}}
        started = time.monotonic()
        launch = ["bash", str(TOOL_DIR / "launch_run.sh"),
                  demo, config["object_channel_substring"], *map(str, pose),
                  str(cap), str(port), str(out), "--controller-params", str(controller_path)]
        if goal_yaw_degrees is not None:
            launch.extend(["--goal-yaw-degrees", str(plan["goal_yaw_degrees"])])
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
        if rc or not status["seed_verified"] or not status.get("goal_yaw_verified", True) or any(not (out / name).is_file() or not (out / name).stat().st_size
                    for name in ("steps_raw.jsonl", "state_trace.jsonl")):
            raise RuntimeError(f"Invalid/no-data run; preserved logs in {out}")
        # Packaging occurs after simulation cleanup and under the same lock.
        (out / "tmp").mkdir(exist_ok=True)
        env.update(TMPDIR=str(out / "tmp"), MPLCONFIGDIR=str(out / "tmp/matplotlib"), OMP_NUM_THREADS="1",
                   OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
        models = REPO / "examples/sampling_c3/urdf"
        obj, obs = MODELS[scene]
        render = [sys.executable, str(TOOL_DIR / "render_run_3d.py"),
                  "--trace", str(out / "state_trace.jsonl"), "--out", str(out / f"{run_id}.mp4"),
                  "--object-sdf", str(models / obj), "--goal", *map(str, pose),
                  "--title", run_id, "--max-frames", str(max_frames)]
        if obs:
            render.extend(["--obstacle-sdf", str(models / obs)])
        commands = {
            "postprocess": [sys.executable, str(TOOL_DIR / "postprocess_run.py"),
                            "--run-dir", str(out), "--scene", scene, "--run-id", run_id,
                            "--scene-config", str(config_path), "--demo", demo],
            "render": render,
            "cost_fig": [sys.executable, str(TOOL_DIR / "cost_fig.py"),
                         "--run-dir", str(out), "--scene", scene, "--obstacle_cost", obstacle_cost],
        }
        for phase, command in commands.items():
            print(f"[PACKAGE] {run_id} {phase}", flush=True)
            status[phase + "_rc"] = logged_command(command, out / f"{phase}.log", env)
            status_path.write_text(json.dumps(status, indent=2) + "\n")
            if status[phase + "_rc"]:
                raise RuntimeError(f"{phase} failed; inspect {out / (phase + '.log')}")
        (out / "RUN_COMPLETE").touch()
        print(f"[COMPLETE] {run_id} failures={status['failures']} folder={out}", flush=True)
        return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=SCENES, required=True)
    parser.add_argument("--obstacle_cost", choices=OBSTACLE_COSTS, default="exponential")
    parser.add_argument("--start", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal-yaw-degrees", type=int, choices=[90, 0, -90],
                        help="Absolute world yaw; preserve the indexed goal position")
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int, default=600, help="Recorder wall-time cap, seconds")
    parser.add_argument("--port", type=int, default=18001)
    parser.add_argument("--out", type=Path, required=True, help="New directory; existing dirs are refused")
    parser.add_argument("--max-frames", type=int, default=1200)
    parser.add_argument("--dry-run", action="store_true", help="Print the resolved run plan without building, running, or writing files")
    args = parser.parse_args()
    try:
        if args.dry_run:
            print(json.dumps(plan_run(args.scene, args.obstacle_cost, args.start, args.goal,
                                      args.out, args.cap, args.port,
                                      max_frames=args.max_frames, goal_yaw_degrees=args.goal_yaw_degrees), indent=2))
            return
        run_one(args.scene, args.obstacle_cost, args.start, args.goal, args.out,
                args.cap, args.port, max_frames=args.max_frames, goal_yaw_degrees=args.goal_yaw_degrees)
    except (RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
