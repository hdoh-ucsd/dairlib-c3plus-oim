#!/usr/bin/env python3
"""Serial baseline/ReLU campaign with safe resume and stop-after-current."""
import argparse
import json
from pathlib import Path

from run_experiment import SCENES, run_one


def jobs(args):
    if args.manifest:
        data = json.loads(args.manifest.read_text())
        if data["seed"] != 42:
            raise ValueError("This reproduction workflow uses seed 42 only")
        return data["runs"]
    variants = ["baseline", "relu"] if args.variant == "both" else [args.variant]
    return [dict(scene=s, start=m, goal=n, variant=v) for s in args.scenes
            for m in range(1, 6) for n in range(1, 6)
            if args.pairs == "all" or (args.pairs == "diagonal" and m == n)
            or (args.pairs == "smoke" and m == n == 1)
            for v in variants]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, help="Exact job list, e.g. campaign_seed42.json")
    parser.add_argument("--scenes", choices=SCENES, nargs="+", default=["single_obstacle", "icra_sign"])
    parser.add_argument("--variant", choices=["baseline", "relu", "both"], default="both")
    parser.add_argument("--pairs", choices=["all", "diagonal", "smoke"], default="smoke")
    parser.add_argument("--seed", type=int, choices=[42], default=42)
    parser.add_argument("--cap", type=int, help="Override wall-time cap (default 600 or manifest cap)")
    parser.add_argument("--port-base", type=int, default=19000)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--resume", action="store_true", help="Skip complete runs; refuse partial directories")
    args = parser.parse_args()
    args.output_root = args.output_root.resolve()
    args.output_root.mkdir(parents=True, exist_ok=True)
    try:
        selected = jobs(args)
        with (args.output_root / "campaign_driver.log").open("a", buffering=1) as driver:
            for index, job in enumerate(selected, 1):
                if (args.output_root / "STOP_AFTER_CURRENT").exists():
                    driver.write(f"[STOPPED_AFTER_CURRENT] next_index={index}\n")
                    print("Stopped between runs; current run fully packaged", flush=True)
                    return
                pair = f"s{job['start']:02d}g{job['goal']:02d}"
                out = args.output_root / job["variant"] / job["scene"] / pair
                if args.resume and (out / "RUN_COMPLETE").exists():
                    driver.write(f"[SKIP_COMPLETE] {out}\n")
                    continue
                driver.write(f"[START] {index}/{len(selected)} {out}\n")
                print(f"[START] {index}/{len(selected)} {out}", flush=True)
                status = run_one(job["scene"], job["variant"], job["start"], job["goal"],
                                 out, args.cap if args.cap is not None else job.get("cap", 600),
                                 args.port_base + index, job.get("goal_pose"))
                driver.write(f"[COMPLETE] {out} failures={status['failures']}\n")
            driver.write(f"[CAMPAIGN_COMPLETE] planned={len(selected)}\n")
    except (RuntimeError, ValueError, FileExistsError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
