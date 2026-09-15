#!/usr/bin/env python3
"""Serial exponential/ReLU campaign with safe resume and stop-after-current."""
import argparse
import csv
import json
import hashlib
import subprocess
import tempfile
from pathlib import Path
import sys

from c3plus.configs import REPO, SCENES, OBJECTS, configuration_snapshot
from c3plus.configs.catalog import canonical_task, canonical_object
from c3plus.configs.poses import pose_ids, pose_provenance
from c3plus.runtime.environment import full_preflight
from c3plus.utils.plan import DEFAULT_WALL_CAP_SECONDS, plan_run, yaw_suffix
from c3plus.utils.run import run_one
from c3plus.evaluation.package import completion, load_status
from c3plus.evaluation.serialization import _json
from c3plus.evaluation.validation import _validate


NAMED_CAMPAIGNS = {"run_launch": range(1, 6), "run_launch_simple_s2": (2,)}


def jobs(args):
    if getattr(args, "suite", None) == "full":
        return [dict(scene=task, object_name=obj, start=int(start), goal=int(goal),
                     obstacle_cost=args.obstacle_cost)
                for task in SCENES for obj in OBJECTS
                for start in pose_ids("start", task) for goal in pose_ids("goal", task)]
    campaign_name = getattr(args, "campaign_name", None)
    if campaign_name:
        return [dict(scene=scene, start=start, goal=2, goal_yaw_degrees=yaw,
                     obstacle_cost=cost)
                for scene in SCENES for start in NAMED_CAMPAIGNS[campaign_name]
                for yaw in (90, 0, -90) for cost in ("exponential", "relu")]
    if args.manifest:
        data = json.loads(args.manifest.read_text())
        if data["seed"] != 42:
            raise ValueError("This reproduction workflow uses seed 42 only")
        return data["runs"]
    obstacle_costs = ["exponential", "relu"] if args.obstacle_cost == "both" else [args.obstacle_cost]
    return [dict(scene=s, object_name=obj, start=m, goal=n, obstacle_cost=v) for s in args.scenes
            for obj in (getattr(args, "objects", None) or ["T_shape"])
            for m in map(int, pose_ids("start", s)) for n in map(int, pose_ids("goal", s))
            if args.pairs == "all" or (args.pairs == "diagonal" and m == n)
            or (args.pairs == "smoke" and m == n == 1)
            for v in obstacle_costs]


def write_summary(planned, output_root):
    """Refresh the campaign table from saved artifacts, including resumed runs."""
    phase_codes = ("wrapper_rc", "postprocess_rc", "render_rc", "cost_fig_rc")
    fields = ["run_id", "scene", "obstacle_cost", "start", "goal_index",
              "goal_yaw_degrees", "seed", "wall_cap_seconds", "status", "success",
              "t_success", "simulation_wall_seconds", "sim_time_end", "n_control_steps",
              "final_position_error", "final_orientation_error", "failures", *phase_codes,
              "seed_verified", "goal_yaw_verified", "video", "folder"]
    path = output_root / "summary.csv"
    with tempfile.NamedTemporaryFile(mode="w", newline="", dir=output_root,
                                     prefix=".summary-", suffix=".csv", delete=False) as stream:
        temporary = Path(stream.name)
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for plan in planned:
            out = Path(plan["out"])
            result_path = out / f"{plan['run_id']}_result.json"
            status = load_status(out, plan["run_id"])
            result = json.loads(result_path.read_text()) if result_path.exists() else {}
            complete = completion(out, plan["run_id"])
            state = "complete" if complete else "partial" if out.exists() else "pending"
            if (status.get("failures") or any(status.get(key) for key in phase_codes)
                    or status.get("seed_verified") is False or status.get("goal_yaw_verified") is False):
                state += "_with_failures"
            video = out / f"{plan['run_id']}.mp4"
            row = {key: plan[key] for key in fields if key in plan}
            row.update({key: result[key] for key in fields if key in result})
            row.update({key: status[key] for key in (*phase_codes, "seed_verified", "goal_yaw_verified")
                        if key in status})
            row.update(status=state, simulation_wall_seconds=status.get("simulation_wall_seconds", ""),
                       failures=json.dumps(status.get("failures", [])),
                       video=str(video.relative_to(output_root)) if video.is_file() else "",
                       folder=str(out.relative_to(output_root)))
            writer.writerow(row)
    temporary.replace(path)


@configuration_snapshot()
def build_manifest(args):
    """Resolve the complete ordered selection without writing or launching."""
    selected, planned = jobs(args), []
    for index, job in enumerate(selected, 1):
        task = canonical_task(job["scene"])
        obj = canonical_object(job.get("object_name") or "T_shape")
        pair = f"s{job['start']:02d}g{job['goal']:02d}" + yaw_suffix(job.get("goal_yaw_degrees"))
        out = args.output_root / job["obstacle_cost"] / task / obj / pair
        plan = plan_run(task, job["obstacle_cost"], job["start"], job["goal"], out,
                        args.cap if args.cap is not None else job.get("cap", DEFAULT_WALL_CAP_SECONDS),
                        args.port_base + index, job.get("goal_pose"),
                        goal_yaw_degrees=job.get("goal_yaw_degrees"), object_name=obj)
        if args.suite == "full":
            plan["out"] = str(args.output_root / task / obj / plan["run_id"])
        planned.append(plan)
    if len({plan["run_id"] for plan in planned}) != len(planned):
        raise ValueError("Campaign contains duplicate run identities")
    data = {"seed": args.seed, "run_count": len(planned), "runs": planned}
    if args.suite == "full":
        starts = list(pose_ids("start"))
        goals = list(pose_ids("goal"))
        for task in SCENES:
            if list(pose_ids("start", task)) != starts or list(pose_ids("goal", task)) != goals:
                raise ValueError(f"{task}: inconsistent canonical pose IDs")
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
        patch = subprocess.check_output(["git", "diff", "HEAD", "--binary"], cwd=REPO)
        data.update(schema="c3plus-campaign/v1", suite="full", tasks=list(SCENES), objects=list(OBJECTS),
                    start_ids=starts, goal_ids=goals, n_starts=len(starts), n_goals=len(goals),
                    pairs_per_task_object=len(starts) * len(goals), output_root=str(args.output_root),
                    obstacle_cost=args.obstacle_cost, pose_catalogue=pose_provenance(),
                    repository={"commit": commit, "tracked_patch_sha256": hashlib.sha256(patch).hexdigest()})
        # Shared profile/asset/pose provenance appears once, rather than 750 times.
        data["object_profiles"] = {obj: next(p["object_profile"] for p in planned if p["object_name"] == obj)
                                   for obj in OBJECTS}
        data["object_assets"] = {obj: next(p["asset_sha256"] for p in planned if p["object_name"] == obj)
                                 for obj in OBJECTS}
        for plan in planned:
            for key in ("object_profile", "asset_sha256", "pose_catalogue"):
                plan.pop(key)
    return json.loads(json.dumps(data, allow_nan=False))


def validate_completed_run(plan):
    """Apply existing package/recording checks before accepting a full-suite skip."""
    directory = Path(plan["out"])
    if directory.is_symlink() or not completion(directory, plan["run_id"]):
        raise ValueError("missing or invalid final JSON/video package")
    result = _json((directory / f"{plan['run_id']}_result.json").read_text())
    if {p.name for p in directory.iterdir()} != {f"{plan['run_id']}_result.json", f"{plan['run_id']}.mp4"}:
        raise ValueError("unexpected artifacts remain in the final run directory")
    runtime = result.get("runtime_status") or {}
    if any(runtime.get(key) != 0 for key in ("wrapper_rc", "postprocess_rc", "render_rc")):
        raise ValueError("saved launch/postprocess/render did not finish successfully")
    if runtime.get("seed_verified") is not True or runtime.get("goal_yaw_verified") is False:
        raise ValueError("saved seed or goal verification failed")
    for key in ("run_id", "scene", "native_scene", "object_name", "native_object_name", "start", "goal_index",
                "seed", "configuration_digest", "wall_cap_seconds", "canonical_start_pose", "canonical_goal_pose",
                "controller_goal", "simulation_model", "controller_model", "start_pose", "evaluation_goal",
                "object_body_name", "object_channel_substring", "max_frames"):
        if runtime.get(key) != plan[key]:
            raise ValueError(f"saved {key} differs from the campaign manifest")
    for key in ("execution_step_budget", "goal_yaw_degrees"):
        if runtime.get(key) != plan.get(key):
            raise ValueError(f"saved {key} differs from the campaign manifest")
    cfg = result["provenance"]["configuration"]["files"]["evaluation_scene_config.yaml"]["data"]
    _validate(result, result["recording"], cfg, directory)


def inventory_runs(manifest):
    """Inspect every destination up front; a partial run never becomes pending."""
    completed, pending, invalid = [], [], []
    for plan in manifest["runs"]:
        directory = Path(plan["out"])
        if not directory.exists() and not directory.is_symlink():
            pending.append(plan["run_id"])
            continue
        try:
            if manifest.get("suite") == "full":
                validate_completed_run(plan)
            elif not completion(directory, plan["run_id"]):
                raise ValueError("missing or invalid final JSON/video package")
            completed.append(plan["run_id"])
        except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
            invalid.append({"run_id": plan["run_id"], "out": plan["out"], "reason": str(exc)})
    return {"completed": len(completed), "pending": len(pending), "invalid": len(invalid),
            "total": len(manifest["runs"]), "completed_run_ids": completed, "invalid_runs": invalid}


def check_campaign_metadata(root):
    for name in ("manifest.json", "manifest.json.tmp", "campaign_plan.json", "campaign_plan.json.tmp",
                 "summary.csv", "summary.csv.tmp", "campaign_driver.log", "STOP_AFTER_CURRENT"):
        path = root / name
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise ValueError(f"Refusing nonregular campaign metadata: {path}")


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    name = argv.pop(0) if argv and argv[0] in NAMED_CAMPAIGNS else None
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=["full"], help="Every canonical task, object, start and goal")
    parser.add_argument("--manifest", type=Path, help="Exact targeted job list")
    parser.add_argument("--tasks", "--scenes", dest="scenes", type=canonical_task, choices=SCENES, nargs="+")
    parser.add_argument("--objects", type=canonical_object, choices=OBJECTS, nargs="+")
    parser.add_argument("--obstacle_cost", choices=["exponential", "relu", "both"])
    parser.add_argument("--pairs", choices=["all", "diagonal", "smoke"])
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int,
                        help=f"Per-run wall-time cap (default {DEFAULT_WALL_CAP_SECONDS} seconds; "
                             "preserves manifest caps unless explicitly overridden)")
    parser.add_argument("--port-base", type=int, default=19000)
    parser.add_argument("--out", "--output-root", dest="output_root", type=Path, required=not name,
                        default=REPO / "results" / name if name else None)
    parser.add_argument("--resume", action="store_true", help="Skip validated complete runs; refuse partial output")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print the manifest without writing or simulating")
    args = parser.parse_args(argv)
    args.campaign_name = name
    selectors = (args.scenes, args.objects, args.pairs)
    if name and (args.manifest or any(value is not None for value in (*selectors, args.obstacle_cost))):
        parser.error("Named targeted campaigns have a fixed selection; do not combine them with grid selectors")
    if args.suite and (name or args.manifest or any(value is not None for value in selectors)):
        parser.error("--suite full selects every task/object/pose; do not combine it with targeted selectors")
    if args.suite and args.obstacle_cost == "both":
        parser.error("--suite full uses one obstacle cost per campaign; choose exponential or relu")
    if args.manifest and any(value is not None for value in (*selectors, args.obstacle_cost)):
        parser.error("--manifest supplies the exact job list; do not combine it with grid selectors")
    args.scenes = args.scenes or ["single_obstacle", "icra_sign"]
    args.obstacle_cost = args.obstacle_cost or ("exponential" if args.suite else "both")
    args.pairs = args.pairs or "smoke"
    args.output_root = args.output_root.absolute()
    try:
        if args.output_root.is_symlink():
            raise ValueError("Refusing a symlinked campaign output root")
        args.output_root = args.output_root.resolve()
        check_campaign_metadata(args.output_root)
        data = build_manifest(args)
        plan_path = args.output_root / ("manifest.json" if args.suite else "campaign_plan.json")
        if plan_path.exists() and _json(plan_path.read_text()) != data:
            raise ValueError("Output root already contains a different campaign plan; choose a new --out")
        if args.suite and args.output_root.exists() and not plan_path.exists() and any(args.output_root.iterdir()):
            raise ValueError("Nonempty output root has no matching manifest.json; choose a new --out")
        inventory = inventory_runs(data)
        if args.suite:
            geometry = ({"not_checked": "Output inventory is invalid"} if inventory["invalid"] else
                        full_preflight(data["runs"], args.output_root, require_runtime=not args.dry_run))
            report = {"suite": "full", "seed": args.seed, "tasks": data["tasks"], "objects": data["objects"],
                      "starts": data["n_starts"], "goals": data["n_goals"],
                      "pairs_per_task_object": data["pairs_per_task_object"], "total_runs": data["run_count"],
                      "output_root": str(args.output_root), "resume": args.resume,
                      "status": inventory, "preflight": geometry}
        else:
            report = {"seed": args.seed, "resume": args.resume, "status": inventory}
        if args.dry_run:
            print(json.dumps({**report, "manifest": data, "run_count": data["run_count"],
                              "runs": data["runs"]} if not args.suite else {**report, "manifest": data}, indent=2))
        else:
            print(json.dumps(report, indent=2), flush=True)
        if inventory["invalid"]:
            raise ValueError("Invalid/partial runs found; preserved every artifact. Use a new --out or resolve the listed runs.")
        if inventory["completed"] and not args.resume:
            raise ValueError("Completed runs already exist; use --resume to skip them")
        if args.dry_run:
            return
        args.output_root.mkdir(parents=True, exist_ok=True)
        check_campaign_metadata(args.output_root)
        if not plan_path.exists():
            with tempfile.NamedTemporaryFile(mode="w", dir=args.output_root, prefix=".manifest-",
                                             suffix=".json", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(json.dumps(data, indent=2, allow_nan=False) + "\n")
            temporary.replace(plan_path)
        completed = set(inventory["completed_run_ids"])
        with (args.output_root / "campaign_driver.log").open("a", buffering=1) as driver:
            for index, plan in enumerate(data["runs"], 1):
                if (args.output_root / "STOP_AFTER_CURRENT").exists():
                    driver.write(f"[STOPPED_AFTER_CURRENT] next_index={index}\n")
                    print("Stopped between runs; current run fully packaged", flush=True)
                    return
                out = Path(plan["out"])
                if plan["run_id"] in completed:
                    driver.write(f"[SKIP_COMPLETE] {out}\n")
                    continue
                driver.write(f"[START] {index}/{len(data['runs'])} {out}\n")
                print(f"[START] {index}/{len(data['runs'])} {out}", flush=True)
                try:
                    status = run_one(plan["scene"], plan["obstacle_cost"], plan["start"], plan["goal_index"],
                                     out, plan["wall_cap_seconds"], plan["port"], plan["evaluation_goal"],
                                     goal_yaw_degrees=plan.get("goal_yaw_degrees"), object_name=plan["object_name"])
                finally:
                    write_summary(data["runs"], args.output_root)
                driver.write(f"[COMPLETE] {out} failures={status['failures']}\n")
            write_summary(data["runs"], args.output_root)
            driver.write(f"[CAMPAIGN_COMPLETE] planned={len(data['runs'])}\n")
    except (RuntimeError, ValueError, OSError, KeyError, TypeError, subprocess.SubprocessError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
