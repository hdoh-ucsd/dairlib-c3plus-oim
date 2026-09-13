#!/usr/bin/env python3
"""Serial exponential/ReLU campaign with safe resume and stop-after-current."""
import argparse
import csv
import json
from pathlib import Path
import sys

if __package__:
    from .run_experiment import REPO, SCENES, plan_run, run_one, yaw_suffix
else:
    from run_experiment import REPO, SCENES, plan_run, run_one, yaw_suffix


NAMED_CAMPAIGNS = {"run_launch": range(1, 6), "run_launch_simple_s2": (2,)}


def jobs(args):
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
    return [dict(scene=s, start=m, goal=n, obstacle_cost=v) for s in args.scenes
            for m in range(1, 6) for n in range(1, 6)
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
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for plan in planned:
            out = Path(plan["out"])
            status_path = out / "runtime_status.json"
            result_path = out / f"{plan['run_id']}_result.json"
            status = json.loads(status_path.read_text()) if status_path.exists() else {}
            result = json.loads(result_path.read_text()) if result_path.exists() else {}
            complete = (out / "RUN_COMPLETE").exists()
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


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    campaign_name = argv.pop(0) if argv and argv[0] in NAMED_CAMPAIGNS else None
    description = (f"{campaign_name}: all six scenes, goal 2 at absolute yaw 90/0/-90 degrees, "
                   f"both costs, {'starts 1–5' if campaign_name == 'run_launch' else 'start 2 only'}."
                   if campaign_name else __doc__)
    parser = argparse.ArgumentParser(description=description)
    if not campaign_name:
        parser.add_argument("--manifest", type=Path, help="Exact job list, e.g. campaign_seed42.json")
        parser.add_argument("--scenes", choices=SCENES, nargs="+", help="Grid scenes (default: single_obstacle icra_sign)")
        parser.add_argument("--obstacle_cost", choices=["exponential", "relu", "both"], help="Grid obstacle costs (default: both)")
        parser.add_argument("--pairs", choices=["all", "diagonal", "smoke"], help="Grid pairs (default: smoke)")
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int, help="Override wall-time cap (default 600 or manifest cap)")
    parser.add_argument("--port-base", type=int, default=19000)
    parser.add_argument("--output-root", type=Path, required=not campaign_name,
                        default=REPO / "results/reproduce" / campaign_name if campaign_name else None,
                        help="Campaign output directory" + (f" (default: results/reproduce/{campaign_name})"
                                                          if campaign_name else ""))
    parser.add_argument("--resume", action="store_true", help="Skip complete runs; refuse partial directories")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print all planned runs without writing files")
    args = parser.parse_args(argv)
    args.campaign_name = campaign_name
    if not campaign_name and args.manifest and any(value is not None for value in (args.scenes, args.obstacle_cost, args.pairs)):
        parser.error("--manifest cannot be combined with --scenes, --obstacle_cost, or --pairs; "
                     "the manifest supplies the exact job list")
    if not campaign_name:
        args.scenes = args.scenes if args.scenes is not None else ["single_obstacle", "icra_sign"]
        args.obstacle_cost = args.obstacle_cost if args.obstacle_cost is not None else "both"
        args.pairs = args.pairs if args.pairs is not None else "smoke"
    args.output_root = args.output_root.resolve()
    try:
        selected = jobs(args)
        planned = []
        for index, job in enumerate(selected, 1):
            pair = f"s{job['start']:02d}g{job['goal']:02d}" + yaw_suffix(job.get("goal_yaw_degrees"))
            out = args.output_root / job["obstacle_cost"] / job["scene"] / pair
            yaw = {"goal_yaw_degrees": job["goal_yaw_degrees"]} if "goal_yaw_degrees" in job else {}
            planned.append(plan_run(job["scene"], job["obstacle_cost"], job["start"], job["goal"],
                                    out, args.cap if args.cap is not None else job.get("cap", 600),
                                    args.port_base + index, job.get("goal_pose"), **yaw))
        if len({job["out"] for job in planned}) != len(planned):
            raise ValueError("Campaign contains duplicate output directories")
        if args.dry_run:
            print(json.dumps({"seed": 42, "run_count": len(planned), "runs": planned}, indent=2))
            return
        plan_path = args.output_root / "campaign_plan.json"
        plan_data = {"seed": 42, "run_count": len(planned), "runs": planned}
        # Preserve the original plan when resuming; changing selection or caps
        # in place would make skipped results appear to use the new settings.
        if plan_path.exists() and json.loads(plan_path.read_text()) != json.loads(json.dumps(plan_data)):
            raise ValueError("Output root already contains a different campaign plan; choose a new --output-root")
        args.output_root.mkdir(parents=True, exist_ok=True)
        if not plan_path.exists():
            plan_path.write_text(json.dumps(plan_data, indent=2) + "\n")
        with (args.output_root / "campaign_driver.log").open("a", buffering=1) as driver:
            for index, (job, plan) in enumerate(zip(selected, planned), 1):
                if (args.output_root / "STOP_AFTER_CURRENT").exists():
                    driver.write(f"[STOPPED_AFTER_CURRENT] next_index={index}\n")
                    print("Stopped between runs; current run fully packaged", flush=True)
                    return
                out = Path(plan["out"])
                if args.resume and (out / "RUN_COMPLETE").exists():
                    driver.write(f"[SKIP_COMPLETE] {out}\n")
                    write_summary(planned, args.output_root)
                    continue
                driver.write(f"[START] {index}/{len(selected)} {out}\n")
                print(f"[START] {index}/{len(selected)} {out}", flush=True)
                yaw = {"goal_yaw_degrees": job["goal_yaw_degrees"]} if "goal_yaw_degrees" in job else {}
                try:
                    status = run_one(job["scene"], job["obstacle_cost"], job["start"], job["goal"],
                                     out, plan["wall_cap_seconds"], plan["port"], job.get("goal_pose"), **yaw)
                finally:
                    write_summary(planned, args.output_root)
                driver.write(f"[COMPLETE] {out} failures={status['failures']}\n")
            driver.write(f"[CAMPAIGN_COMPLETE] planned={len(selected)}\n")
    except (RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
