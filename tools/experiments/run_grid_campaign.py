#!/usr/bin/env python3
"""Serial exponential/ReLU campaign with safe resume and stop-after-current."""
import argparse
import json
from pathlib import Path

if __package__:
    from .run_experiment import SCENES, plan_run, run_one
else:
    from run_experiment import SCENES, plan_run, run_one


def jobs(args):
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, help="Exact job list, e.g. campaign_seed42.json")
    parser.add_argument("--scenes", choices=SCENES, nargs="+", help="Grid scenes (default: single_obstacle icra_sign)")
    parser.add_argument("--obstacle_cost", choices=["exponential", "relu", "both"], help="Grid obstacle costs (default: both)")
    parser.add_argument("--pairs", choices=["all", "diagonal", "smoke"], help="Grid pairs (default: smoke)")
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int, help="Override wall-time cap (default 600 or manifest cap)")
    parser.add_argument("--port-base", type=int, default=19000)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--resume", action="store_true", help="Skip complete runs; refuse partial directories")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print all planned runs without writing files")
    args = parser.parse_args()
    if args.manifest and any(value is not None for value in (args.scenes, args.obstacle_cost, args.pairs)):
        parser.error("--manifest cannot be combined with --scenes, --obstacle_cost, or --pairs; "
                     "the manifest supplies the exact job list")
    args.scenes = args.scenes if args.scenes is not None else ["single_obstacle", "icra_sign"]
    args.obstacle_cost = args.obstacle_cost if args.obstacle_cost is not None else "both"
    args.pairs = args.pairs if args.pairs is not None else "smoke"
    args.output_root = args.output_root.resolve()
    try:
        selected = jobs(args)
        planned = []
        for index, job in enumerate(selected, 1):
            pair = f"s{job['start']:02d}g{job['goal']:02d}"
            out = args.output_root / job["obstacle_cost"] / job["scene"] / pair
            planned.append(plan_run(job["scene"], job["obstacle_cost"], job["start"], job["goal"],
                                    out, args.cap if args.cap is not None else job.get("cap", 600),
                                    args.port_base + index, job.get("goal_pose")))
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
            for index, job in enumerate(selected, 1):
                if (args.output_root / "STOP_AFTER_CURRENT").exists():
                    driver.write(f"[STOPPED_AFTER_CURRENT] next_index={index}\n")
                    print("Stopped between runs; current run fully packaged", flush=True)
                    return
                pair = f"s{job['start']:02d}g{job['goal']:02d}"
                out = args.output_root / job["obstacle_cost"] / job["scene"] / pair
                if args.resume and (out / "RUN_COMPLETE").exists():
                    driver.write(f"[SKIP_COMPLETE] {out}\n")
                    continue
                driver.write(f"[START] {index}/{len(selected)} {out}\n")
                print(f"[START] {index}/{len(selected)} {out}", flush=True)
                status = run_one(job["scene"], job["obstacle_cost"], job["start"], job["goal"],
                                 out, args.cap if args.cap is not None else job.get("cap", 600),
                                 args.port_base + index, job.get("goal_pose"))
                driver.write(f"[COMPLETE] {out} failures={status['failures']}\n")
            driver.write(f"[CAMPAIGN_COMPLETE] planned={len(selected)}\n")
    except (RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
