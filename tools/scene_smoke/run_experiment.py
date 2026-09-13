#!/usr/bin/env python3
"""Run and package one xArm6 baseline/ReLU experiment from this checkout."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools/relu_chomp"))
import common as C

SCENES = tuple(C.SCENES)
MODELS = {
    "open_task": ("push_t_oimscale_m01.sdf", None),
    "single_obstacle": ("push_t_oimscale_m01.sdf", "single_obstacle_box_oimframe.sdf"),
    "shelf_gap": ("push_t_oimscale_m01.sdf", "scene_shelf_gap_oimframe.sdf"),
    "ycb_clutter": ("push_t_oimscale_m01.sdf", "scene_ycb_clutter_oimframe.sdf"),
    "icra_sign": ("push_c_glyph.sdf", "scene_icra_sign.sdf"),
    "slalom": ("push_t_oimscale_m01.sdf", "scene_slalom_oimframe.sdf"),
}
BINARIES = ("franka_sim", "franka_osc_controller", "franka_sampling_c3_controller")


def environment(variant):
    # Isolate settings from previously exported experiment knobs.
    env = {k: v for k, v in os.environ.items() if not k.startswith("SAMPLING_C3_")}
    env.update(PYTHON=sys.executable, SAMPLING_C3_SEED="42",
               SAMPLING_C3_OBSTACLE_MODE="lcs_contact")
    if variant == "relu":
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
        return subprocess.run(command, cwd=REPO, env=env, stdout=stream,
                              stderr=subprocess.STDOUT).returncode


def run_one(scene, variant, start, goal, out, cap=600, port=18001,
            goal_pose=None, max_frames=1200):
    if scene not in SCENES or variant not in C.VARIANTS:
        raise ValueError("Unknown scene or variant")
    if not 1 <= start <= 5 or not 1 <= goal <= 5:
        raise ValueError("Start/goal indices must be between 1 and 5")
    if cap <= 0 or not 1024 <= port <= 65535 or max_frames <= 0:
        raise ValueError("Invalid cap, port, or frame count")
    pose = tuple(goal_pose) if goal_pose is not None else C.goal_of(scene, goal)
    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise ValueError("Goal pose must contain three finite values")
    out = Path(out).resolve()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite existing run directory: {out}")
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
        run_id = f"{variant}_{scene}_s{start:02d}g{goal:02d}_seed42"
        demo = C.demo_name(scene, start, goal)
        config = yaml.safe_load((REPO / "tools/scene_smoke/scene_configs" / f"{scene}.yaml").read_text())
        config["goal"] = list(pose)
        config_path = out / "evaluation_scene_config.yaml"
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        env = environment(variant)
        status = {"run_id": run_id, "scene": scene, "variant": variant, "seed": 42,
                  "demo": demo, "goal": pose, "wall_cap_seconds": cap,
                  "execution": "serial", "python": sys.executable,
                  "worktree_dirty": bool(subprocess.check_output(
                      ["git", "status", "--porcelain", "--untracked-files=no"], cwd=REPO)),
                  "sampler_settings": {k: v for k, v in env.items() if k.startswith("SAMPLING_C3_")},
                  "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
                  "binary_sha256": {name: hashlib.sha256((binary_dir / name).read_bytes()).hexdigest()
                                    for name in BINARIES}}
        env_file = "" if scene == "open_task" else str(REPO / "tools/scene_smoke" / f"env_{scene}.sh")
        started = time.monotonic()
        rc = logged_command(["bash", str(REPO / "tools/scene_smoke/run_scene_smoke.sh"),
                             demo, config["object_channel_substring"], *map(str, pose),
                             str(cap), str(port), str(out), env_file], out / "launcher.log", env)
        status.update(wrapper_rc=rc, simulation_wall_seconds=time.monotonic()-started,
                      failures=classify_failure(out))
        status["seed_verified"] = "[SAMPLER-SEED] deterministic seed=42" in (out / "planner.log").read_text(errors="replace")
        status_path = out / "runtime_status.json"
        status_path.write_text(json.dumps(status, indent=2) + "\n")
        if rc or not status["seed_verified"] or any(not (out / name).is_file() or not (out / name).stat().st_size
                    for name in ("steps_raw.jsonl", "state_trace.jsonl")):
            raise RuntimeError(f"Invalid/no-data run; preserved logs in {out}")
        # Packaging occurs after simulation cleanup and under the same lock.
        env.update(MPLCONFIGDIR=str(out / "tmp/matplotlib"), OMP_NUM_THREADS="1",
                   OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
        models = REPO / "examples/sampling_c3/urdf"
        obj, obs = MODELS[scene]
        render = [sys.executable, str(REPO / "tools/scene_smoke/render_run_3d.py"),
                  "--trace", str(out / "state_trace.jsonl"), "--out", str(out / f"{run_id}.mp4"),
                  "--object-sdf", str(models / obj), "--goal", *map(str, pose),
                  "--title", run_id, "--max-frames", str(max_frames)]
        if obs:
            render.extend(["--obstacle-sdf", str(models / obs)])
        commands = {
            "postprocess": [sys.executable, str(REPO / "tools/scene_smoke/postprocess_run.py"),
                            "--run-dir", str(out), "--scene", scene, "--run-id", run_id,
                            "--scene-config", str(config_path), "--demo", demo],
            "render": render,
            "cost_fig": [sys.executable, str(REPO / "tools/relu_chomp/cost_fig.py"),
                         "--run-dir", str(out), "--scene", scene, "--variant", variant],
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
    parser.add_argument("--variant", choices=C.VARIANTS, default="baseline")
    parser.add_argument("--start", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int, default=600, help="Recorder wall-time cap, seconds")
    parser.add_argument("--port", type=int, default=18001)
    parser.add_argument("--out", type=Path, required=True, help="New directory; existing dirs are refused")
    parser.add_argument("--max-frames", type=int, default=1200)
    args = parser.parse_args()
    try:
        run_one(args.scene, args.variant, args.start, args.goal, args.out,
                args.cap, args.port, max_frames=args.max_frames)
    except (RuntimeError, ValueError, FileExistsError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
