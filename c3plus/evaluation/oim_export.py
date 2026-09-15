"""Emit a small OIM-contract run file beside this repository's own result JSON.

`oim.run_eval` scores ADMM, MPPI and C3+ from one directory of run files, and
derives every metric from ten fields. This repository's result JSON already
carries all ten, but under its own names: the task is `open_table` where OIM
spells it `xarm6_open_table`, and the T-block's object is `T_shape` where OIM
records `scene`. Row grouping is a string compare, so those two names would
silently place C3+ in its own rows next to the baselines instead of joining
them -- the table still looks right.

Renaming them in place is not an option: this repository's own evaluator reads
`run.task` and cross-checks `hyperparameters.object`. So this module writes a
separate, small file holding exactly the contract, and leaves the rich result
untouched.

    python3 -m c3plus.evaluation.oim_export --runs-dir results/my_run \
        --out-dir results/my_run/oim

Then, in the OIM checkout:

    uv run python -m oim.run_eval --runs-dir <out-dir> --group-by object task
"""
import argparse
import json
import math
from pathlib import Path

# `run.task`, as OIM spells it: `{robot}_{scene}`.
OIM_TASKS = {
    "open_table": "xarm6_open_table",
    "single_obstacle": "xarm6_single_obstacle",
    "shelf_gap": "xarm6_shelf_gap",
    "ycb_clutter": "xarm6_ycb_clutter",
    "icra_sign": "xarm6_icra_sign",
    "slalom": "xarm6_slalom",
}

# `hyperparameters.object`, as `oim.objects.library` names them. Only the
# T-block differs: it is the scene's own built-in object, recorded as `scene`.
OIM_OBJECTS = {
    "T_shape": "scene",
    "hammer": "hammer",
    "sugar_box": "sugar_box",
    "power_drill": "power_drill",
    "banana": "banana",
}

_SCHEMA = {
    "indexing": (
        "state[i] --control[i]--> state[i+1]. object_pose has steps_run+1 "
        "entries, entry 0 being the initial condition; compute_time has "
        "steps_run entries, entry i being the solve that produced the "
        "command applied from state[i] to state[i+1]."
    ),
    "frames": (
        "object_pose is [x, y, theta] in the world frame, theta in radians; "
        "goal is the same."
    ),
    "source": (
        "Derived from a dairlib C3+ result JSON. control_dt is that run's mean "
        "SIMULATED execution interval; C3+ policy durations vary, so it is a "
        "per-run mean rather than a fixed control period. See clocks."
    ),
    "clocks": (
        "control_dt and compute_time describe the SAME execution steps on "
        "DIFFERENT clocks, so their ratio is the simulator's realtime rate and "
        "not an inconsistency. control_dt is simulation seconds per step, so "
        "T = steps_run * control_dt is the simulated task duration. "
        "compute_time is wall-clock seconds per step. True optimizer solve time "
        "is not recorded, so f_bar = 1/mean(compute_time) is the achieved "
        "execution rate of the whole step -- solve plus simulation plus "
        "messaging. Against a solver-only f_bar this is a LOWER BOUND on the "
        "C3+ solver rate, never an overestimate: the extra work inflates the "
        "mean and so deflates the frequency. Treat it as conservative for C3+."
    ),
}


def _obstacles(scene_config):
    """Scene obstacles in the shape oim/utils/trajectory_figure.py reads.

    Never scored -- the metrics ignore this field -- but an empty list draws a
    bare scene. Rectangles and discs convert exactly; any other polygon is
    reduced to its axis-aligned bounding box and counted, so an approximation
    is visible rather than silent.
    """
    obstacles, approximated = [], 0
    for disc in (scene_config.get("discs") or []):
        if len(disc) == 3:
            obstacles.append({"type": "circle", "center": [float(disc[0]), float(disc[1])],
                              "radius": float(disc[2])})
    for polygon in (scene_config.get("polygons") or []):
        points = [(float(x), float(y)) for x, y in polygon]
        if not points:
            continue
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        centre = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2]
        extents = [(max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2]
        corners = {(min(xs), min(ys)), (max(xs), min(ys)),
                   (max(xs), max(ys)), (min(xs), max(ys))}
        if len(points) != 4 or set(points) != corners:
            approximated += 1
        obstacles.append({"type": "box", "center": centre, "half_extents": extents, "angle": 0.0})
    return obstacles, approximated


def _wrap(angle):
    """Wrap to (-pi, pi], matching oim.utils.metrics._wrap."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def validate_oim_run(payload):
    """Return the problems that would make this file score wrongly, as strings.

    Entries beginning "warning:" still score, but probably not as intended.
    Mirrors the checks in the OIM logging template, because each of them fails
    silently: a wrong name forms its own row, a short `object_pose` shifts
    every error by one step, and a trajectory that reached the goal and drifted
    back out is scored a failure.
    """
    problems = []
    run = payload.get("run", {})
    hp = payload.get("hyperparameters", {})
    static = payload.get("static", {})
    dynamic = payload.get("dynamic", {})

    if run.get("task") not in set(OIM_TASKS.values()):
        problems.append(f"run.task {run.get('task')!r} is not an OIM task name")
    if not run.get("algorithm"):
        problems.append("run.algorithm is missing (use 'c3plus')")
    if run.get("object_opt"):
        problems.append("run.object_opt is set; it renames the method")
    if hp.get("object") not in set(OIM_OBJECTS.values()):
        problems.append(f"hyperparameters.object {hp.get('object')!r} is not an OIM object name")
    if hp.get("consensus") is not None:
        problems.append("hyperparameters.consensus is set; it splits C3+ into extra rows")

    for key in ("goal_pos_tol", "goal_theta_tol", "control_dt"):
        value = hp.get(key)
        if value is None:
            problems.append(f"hyperparameters.{key} is required")
        elif not (isinstance(value, (int, float)) and value > 0):
            problems.append(f"hyperparameters.{key} must be > 0, got {value!r}")

    goal = static.get("goal")
    if goal is None or len(goal) != 3 or not all(math.isfinite(v) for v in goal):
        problems.append("static.goal must be three finite numbers")

    poses = dynamic.get("object_pose")
    if not poses:
        problems.append("dynamic.object_pose is required and must be non-empty")
        return problems
    if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in poses):
        problems.append("dynamic.object_pose rows must be three finite numbers")
    if len(poses) < 2:
        problems.append("dynamic.object_pose needs the initial condition plus one entry per step")

    steps_run = len(poses) - 1
    compute = dynamic.get("compute_time")
    if compute is None:
        problems.append("warning: no dynamic.compute_time, so the f_bar column will be blank")
    elif len(compute) != steps_run:
        problems.append(f"dynamic.compute_time has {len(compute)} entries but object_pose "
                        f"implies {steps_run} steps -- off by {len(compute) - steps_run}")
    elif not all(math.isfinite(c) and c > 0 for c in compute):
        problems.append("dynamic.compute_time has a non-finite or <= 0 entry")

    cap = hp.get("steps")
    if cap is None:
        problems.append("warning: no hyperparameters.steps, so a failed trial is censored "
                        "at its own length instead of the step cap")
    elif cap < steps_run:
        problems.append(f"hyperparameters.steps ({cap}) is below the {steps_run} steps actually run")

    if goal is not None and len(poses) >= 2:
        pos_tol = hp.get("goal_pos_tol") or 0.0
        theta_tol = hp.get("goal_theta_tol") or 0.0
        at_goal = [math.hypot(p[0] - goal[0], p[1] - goal[1]) < pos_tol
                   and abs(_wrap(p[2] - goal[2])) < theta_tol for p in poses[1:]]
        if any(at_goal) and not at_goal[-1]:
            problems.append(f"warning: the goal was met at step {at_goal.index(True) + 1} of "
                            f"{steps_run} but not at the end, so this trial scores as a FAILURE")
    return problems


def build_oim_run(result, step_cap=None):
    """Translate one dairlib C3+ result into the OIM contract payload.

    Args:
        result: A parsed `*_result.json` from this repository.
        step_cap: Override for `hyperparameters.steps`. When omitted, an
            explicit `--steps` budget is used if the run had one; otherwise the
            count is derived so that `steps * control_dt` equals the run's
            configured simulated-time cap, so every failure censors at the same
            task time as the baselines rather than at its own length.

    Returns:
        The payload, ready for `json.dump`. `hyperparameters` is kept to exactly
        the contract so nothing extra is listed above the table as averaged over.
    """
    run = result.get("run") or {}
    hp = result.get("hyperparameters") or {}
    static = result.get("static") or {}
    dynamic = result.get("dynamic") or {}

    task = run.get("task") or result.get("scenario")
    object_name = hp.get("object") or run.get("object")
    if task not in OIM_TASKS:
        raise ValueError(f"Unknown task for OIM export: {task!r}")
    if object_name not in OIM_OBJECTS:
        raise ValueError(f"Unknown object for OIM export: {object_name!r}")

    scene_config = (((result.get("provenance") or {}).get("configuration") or {})
                    .get("files") or {}).get("evaluation_scene_config.yaml") or {}
    obstacles, approximated = _obstacles((scene_config.get("data") or {}).get("obstacles") or {})

    poses = [[float(v) for v in pose] for pose in (dynamic.get("object_pose") or [])]
    compute = dynamic.get("compute_time")
    control_dt = hp.get("control_dt")

    if step_cap is None:
        step_cap = (result.get("execution") or {}).get("step_budget")
    if step_cap is None and control_dt:
        # C3+ is budgeted in SIMULATED SECONDS (--cap), not steps, and its
        # control_dt varies per run. A constant step count would therefore
        # censor each failure at a different time (44-52 s for a 50 s budget);
        # deriving the count from this run's own control_dt censors every
        # failure at exactly the configured budget, which is what T compares.
        runtime = result.get("runtime_status") or {}
        cap_seconds = runtime.get("simulation_cap_seconds") or runtime.get("wall_cap_seconds")
        if cap_seconds:
            step_cap = int(round(float(cap_seconds) / float(control_dt)))

    payload = {
        "schema": _SCHEMA,
        "run": {
            "task": OIM_TASKS[task],
            "algorithm": run.get("algorithm") or "c3plus",
            "robot": run.get("robot") or "xarm6",
            "world": run.get("world") or "3d",
            "seed": run.get("seed"),
            "start_index": run.get("start_index"),
            "goal_index": run.get("goal_index"),
        },
        # Exactly the contract; nothing that varies and is not grouped on.
        "hyperparameters": {
            "object": OIM_OBJECTS[object_name],
            "control_dt": control_dt,
            "goal_pos_tol": hp.get("goal_pos_tol"),
            "goal_theta_tol": hp.get("goal_theta_tol"),
            "steps": None if step_cap is None else int(step_cap),
        },
        "static": {"goal": [float(v) for v in (static.get("goal") or [])],
                   "obstacles": obstacles},
        "dynamic": {"object_pose": poses},
    }
    if compute is not None:
        payload["dynamic"]["compute_time"] = [float(c) for c in compute]
    if approximated:
        payload["schema"] = dict(payload["schema"],
                                 obstacles=f"{approximated} non-rectangular polygon obstacle(s) "
                                           "reduced to axis-aligned bounding boxes for drawing")
    return payload


def export_result(path, out_dir, step_cap=None, strict=True):
    """Write one OIM run file next to a dairlib result. Returns the written path."""
    path = Path(path)
    result = json.loads(path.read_text())
    payload = build_oim_run(result, step_cap=step_cap)
    problems = validate_oim_run(payload)
    hard = [problem for problem in problems if not problem.startswith("warning:")]
    for problem in problems:
        print(f"[oim-export] {path.name}: {problem}")
    if hard and strict:
        raise ValueError(f"{len(hard)} problem(s) would make {path.name} score wrongly")
    out_dir = Path(out_dir)
    target = out_dir / payload["run"]["task"] / payload["hyperparameters"]["object"]
    target.mkdir(parents=True, exist_ok=True)
    written = target / f"{result.get('run_id') or path.stem}.json"
    written.write_text(json.dumps(payload, indent=2) + "\n")
    return written


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--result", type=Path, help="One dairlib *_result.json")
    source.add_argument("--runs-dir", type=Path, help="Directory searched recursively")
    parser.add_argument("--out-dir", type=Path, required=True, help="Destination for OIM run files")
    parser.add_argument("--steps", type=int, help="Override hyperparameters.steps")
    parser.add_argument("--allow-problems", action="store_true",
                        help="Write even when the file would score wrongly")
    args = parser.parse_args(argv)
    results = ([args.result] if args.result is not None
               else sorted(args.runs_dir.rglob("*_result.json")))
    if not results:
        parser.exit(1, "No *_result.json found\n")
    try:
        for path in results:
            print(export_result(path, args.out_dir, args.steps, not args.allow_problems))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
