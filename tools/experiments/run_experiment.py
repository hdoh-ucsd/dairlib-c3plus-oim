#!/usr/bin/env python3
"""Run and package xArm6 exponential/ReLU experiments from this checkout."""
import argparse
import base64
import fcntl
import hashlib
import json
import math
import os
import platform
from pathlib import Path
from importlib.metadata import PackageNotFoundError, version
import re
import shlex
import stat
import subprocess
import sys
import time

import yaml

if __package__:
    from .run_artifacts import compact_run
    from .catalog import (BINARIES, CONFIG_DIR, MESH_OBJECTS, MODELS, OBSTACLE_COSTS, REPO, RUN_OBJECTS,
                          SCENES, TOOL_DIR, demo_name, load_controller_goal,
                          compose_demo_configs, demo_config_digest, model_assets, planner_environment,
                          resolve_object_profile, write_demo_configs)
else:
    from run_artifacts import compact_run
    from catalog import (BINARIES, CONFIG_DIR, MESH_OBJECTS, MODELS, OBSTACLE_COSTS, REPO, RUN_OBJECTS,
                         SCENES, TOOL_DIR, demo_name, load_controller_goal,
                         compose_demo_configs, demo_config_digest, model_assets, planner_environment,
                         resolve_object_profile, write_demo_configs)


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
    """Retain complete phase logs and forward recorder progress as it arrives."""
    with log.open("w") as stream:
        stream.write(f"[COMMAND] {shlex.join(map(os.fsdecode, command))}\n")
        stream.write(f"[CWD] {shlex.quote(str(REPO))}\n")
        stream.flush()
        with subprocess.Popen(command, cwd=REPO, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, errors="replace", bufsize=1) as process:
            for line in process.stdout:
                stream.write(line)
                stream.flush()
                if re.match(r"^\[[^\]]+\] step=\d+ ", line):
                    print(line, end="", flush=True)
            return process.wait()


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
             goal_pose=None, max_frames=1200, goal_yaw_degrees=None, object_name=None, steps=None):
    """Validate inputs and describe a run without launching or writing files."""
    if scene not in SCENES or obstacle_cost not in OBSTACLE_COSTS:
        raise ValueError("Unknown scene or obstacle cost")
    if not 1 <= start <= 5 or not 1 <= goal <= 5:
        raise ValueError("Start/goal indices must be between 1 and 5")
    if cap <= 0 or not 1024 <= port <= 65535 or max_frames <= 0:
        raise ValueError("Invalid cap, port, or frame count")
    if steps is not None and (type(steps) is not int or steps <= 0):
        raise ValueError("--steps must be a positive execution-step budget")
    if object_name is not None and object_name not in RUN_OBJECTS:
        raise ValueError(f"Unsupported run object: {object_name}; choose one of {', '.join(RUN_OBJECTS)}")
    object_options = {"object_name": object_name} if object_name is not None else {}
    profile = resolve_object_profile(scene, repo=REPO, **object_options) if object_options else None
    if profile is not None and object_name not in MESH_OBJECTS:
        # Built-in objects retain the scene's existing recorder channel.
        scene_config = yaml.safe_load((CONFIG_DIR / f"{scene}.yaml").read_text())
        profile["object_channel_substring"] = scene_config["object_channel_substring"]
    suffix = yaw_suffix(goal_yaw_degrees)
    demo = demo_name(scene, start, goal, **object_options)
    goal_file, controller_goal = load_controller_goal(demo, repo=REPO, **object_options)
    resolved = compose_demo_configs(demo, repo=REPO, goal_yaw_degrees=goal_yaw_degrees, **object_options)
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
    object_suffix = f"_{object_name}" if object_name is not None else ""
    plan = {"scene": scene, "obstacle_cost": obstacle_cost, "start": start, "goal_index": goal,
            "run_id": f"{obstacle_cost}_{scene}{object_suffix}_s{start:02d}g{goal:02d}{suffix}_seed42",
            "demo": demo, "seed": 42, "controller_goal": controller_goal,
            "start_pose": resolved["simulation"]["q_init_object"],
            "evaluation_goal": pose, "configuration_file": str(goal_file.relative_to(REPO)),
            "configuration_digest": demo_config_digest(demo, repo=REPO, goal_yaw_degrees=goal_yaw_degrees,
                                                        **object_options),
            "out": str(Path(out).resolve()), "wall_cap_seconds": cap, "port": port,
            "max_frames": max_frames}
    if steps is not None:
        plan["execution_step_budget"] = steps
    if goal_yaw_degrees is not None:
        plan.update(goal_yaw_degrees=int(goal_yaw_degrees), source_controller_goal=source_controller_goal)
    if profile is not None:
        plan.update(object_name=object_name, object_profile=profile,
                    simulation_model=profile["simulation_model"],
                    controller_model=profile["controller_model"],
                    object_body_name=profile["object_body_name"],
                    object_channel_substring=profile["object_channel_substring"])
        plan["asset_sha256"] = {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in model_assets(resolved, REPO)}
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


def capture_source_state(repo):
    """Capture launch-time source bytes without changing the worktree or index.

    The binary patch reproduces the net tracked working tree relative to HEAD;
    staging distinctions and ignored files are not part of this source snapshot.
    Existing binary hashes identify executables separately and do not prove they
    were built from this captured checkout.
    """
    repo = Path(repo).resolve()

    def git(*arguments):
        try:
            return subprocess.check_output(["git", "--no-optional-locks", *arguments],
                                           cwd=repo, stderr=subprocess.PIPE)
        except subprocess.CalledProcessError as exc:
            raise RuntimeError("Cannot capture source state: " +
                               exc.stderr.decode("utf-8", errors="replace").strip()) from exc

    def encoded(raw):
        try:
            content, encoding = raw.decode("utf-8"), "utf-8"
        except UnicodeDecodeError:
            content, encoding = base64.b64encode(raw).decode("ascii"), "base64"
        return {"encoding": encoding, "content": content, "size_bytes": len(raw),
                "sha256": hashlib.sha256(raw).hexdigest()}

    if Path(os.fsdecode(git("rev-parse", "--show-toplevel")).strip()).resolve() != repo:
        raise ValueError("Source capture requires the repository root")
    base_commit = git("rev-parse", "HEAD").decode("ascii").strip()
    patch_args = ("diff", "--binary", "--full-index", "--no-ext-diff", "--no-textconv", "HEAD", "--")
    status_args = ("status", "--porcelain=v1", "--untracked-files=all", "-z")
    source_patch = git(*patch_args)
    status = git(*status_args)
    untracked = {}
    for raw_name in git("ls-files", "--others", "--exclude-standard", "-z").split(b"\0"):
        if not raw_name:
            continue
        name = os.fsdecode(raw_name)
        relative = Path(name)
        path = repo / relative
        if relative.is_absolute() or ".." in relative.parts or not path.parent.resolve().is_relative_to(repo):
            raise ValueError(f"Untracked source path escapes the repository: {name}")
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode):
            raw, kind = os.readlink(os.fsencode(path)), "symlink"
        elif stat.S_ISREG(info.st_mode):
            with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), "rb") as stream:
                raw, kind = stream.read(), "file"
        else:
            raise ValueError(f"Cannot capture non-file untracked source: {name}")
        after = path.lstat()
        if (info.st_ino, info.st_mode, info.st_size, info.st_mtime_ns) != (
                after.st_ino, after.st_mode, after.st_size, after.st_mtime_ns):
            raise RuntimeError(f"Untracked source changed during capture: {name}")
        untracked[name] = {"kind": kind, "mode": format(stat.S_IMODE(info.st_mode), "04o"), **encoded(raw)}
    if (git("rev-parse", "HEAD").decode("ascii").strip() != base_commit or
            git(*patch_args) != source_patch or git(*status_args) != status):
        raise RuntimeError("Repository changed during source capture; retry before launching")
    return {"format": "git-source-state/v1", "base_commit": base_commit,
            "worktree_dirty": bool(status),
            "scope": "Net staged and unstaged tracked changes relative to HEAD, plus nonignored "
                     "untracked files. Git index staging and ignored files are not reproduced.",
            "tracked_patch": encoded(source_patch), "git_status": encoded(status),
            "untracked_files": untracked}


def run_one(scene, obstacle_cost, start, goal, out, cap=600, port=18001,
            goal_pose=None, max_frames=1200, goal_yaw_degrees=None, object_name=None, steps=None):
    plan = plan_run(scene, obstacle_cost, start, goal, out, cap, port, goal_pose, max_frames,
                    goal_yaw_degrees, object_name, steps)
    pose = plan["evaluation_goal"]
    out = Path(out).resolve()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite existing run directory: {out}")
    config = yaml.safe_load((CONFIG_DIR / f"{scene}.yaml").read_text())
    config["goal"] = list(pose)
    if object_name in MESH_OBJECTS:
        profile = plan["object_profile"]
        config.update({key: profile[key] for key in ("footprint", "block_half_height", "tip_target_z",
                                                    "object_channel_substring")})
        if "tip_floor_z_real" in profile:
            config["tip_floor_z_real"] = profile["tip_floor_z_real"]
    if object_name is not None:
        config.update(object_name=object_name, simulation_model=plan["simulation_model"],
                      controller_model=plan["controller_model"], object_body_name=plan["object_body_name"])
    env = environment(obstacle_cost)
    env.update(planner_environment(config))
    binary_dir = REPO / ".build/bin/examples/sampling_c3"
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
        launch = ["bash", str(TOOL_DIR / "launch_run.sh"),
                  demo, config["object_channel_substring"], *map(str, pose),
                  str(cap), str(port), str(out), "--controller-params", str(controller_path)]
        if goal_yaw_degrees is not None:
            launch.extend(["--goal-yaw-degrees", str(plan["goal_yaw_degrees"])])
        if steps is not None:
            launch.extend(["--steps", str(steps)])
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
        object_sdf = models / obj
        if object_name is not None:
            simulation = yaml.safe_load((out / "config/simulation.yaml").read_text())
            object_sdf = REPO / simulation["object_model"]
            status["render_object_model"] = str(object_sdf)
        render = [sys.executable, str(TOOL_DIR / "render_run_3d.py"),
                  "--result", str(out / f"{run_id}_result.json"), "--out", str(out / f"{run_id}.mp4"),
                  "--object-sdf", str(object_sdf), "--goal", *map(str, pose),
                  "--title", run_id, "--max-frames", str(max_frames)]
        if obs:
            render.extend(["--obstacle-sdf", str(models / obs)])
        commands = {
            "postprocess": [sys.executable, str(TOOL_DIR / "postprocess_run.py"),
                            "--run-dir", str(out), "--scene", scene, "--run-id", run_id,
                            "--scene-config", str(config_path), "--demo", demo],
            "render": render,
        }
        for phase, command in commands.items():
            print(f"[PACKAGE] {run_id} {phase}", flush=True)
            status[phase + "_rc"] = logged_command(command, out / f"{phase}.log", env)
            status_path.write_text(json.dumps(status, indent=2) + "\n")
            if status[phase + "_rc"]:
                raise RuntimeError(f"{phase} failed; inspect {out / (phase + '.log')}")
        # Commit one validated record before deleting redundant intermediate files.
        print(f"[PACKAGE] {run_id} consolidate and clean", flush=True)
        compact_run(out, run_id, status=status, require_legacy_complete=False)
        print(f"[COMPLETE] {run_id} failures={status['failures']} folder={out}", flush=True)
        return status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=SCENES, required=True)
    parser.add_argument("--objects", nargs="+", choices=RUN_OBJECTS,
                        help="Objects to push serially; T_block uses its existing scenes, "
                             "imported meshes require open_task; multiple objects use OUT/object_name")
    parser.add_argument("--obstacle_cost", choices=OBSTACLE_COSTS, default="exponential")
    parser.add_argument("--start", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal-yaw-degrees", type=int, choices=[90, 0, -90],
                        help="Absolute world yaw; preserve the indexed goal position")
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int, default=600, help="Recorder wall-time cap, seconds")
    parser.add_argument("--steps", type=int, help="Maximum actually applied outer policies, including reposition; default unlimited")
    parser.add_argument("--port", type=int, default=18001)
    parser.add_argument("--out", type=Path, required=True, help="New directory; existing dirs are refused")
    parser.add_argument("--max-frames", type=int, default=1200)
    parser.add_argument("--dry-run", action="store_true", help="Print the resolved run plan without building, running, or writing files")
    args = parser.parse_args(argv)
    try:
        selected = args.objects if args.objects is not None else [None]
        if len(set(selected)) != len(selected):
            raise ValueError("--objects must not contain duplicates")
        options = dict(max_frames=args.max_frames, goal_yaw_degrees=args.goal_yaw_degrees)
        if args.steps is not None:
            options["steps"] = args.steps
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
                    args.cap, args.port, **options,
                    **({"object_name": name} if name is not None else {}))
    except (RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
