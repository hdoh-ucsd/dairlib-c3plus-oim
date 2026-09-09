#!/usr/bin/env python3
"""Per-run task metrics -> metrics/{baseline,relu}_all_runs.csv (idempotent).

success = trace-latch: a "SUCCESS t=..." line in launcher.log.
first_success_t / best errors come from the FINAL {json} launcher line;
final errors from the last state_trace row.
Usage: collect_task_metrics.py [run_id ...]
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C

FIELDS = ["variant", "scene", "start", "goal", "run_id", "success",
          "first_success_t", "final_pos_err_m", "final_yaw_err_rad",
          "best_pos_err_m", "best_yaw_err_rad", "n_control_steps",
          "duration_trace_s", "solves_per_s"]


def eval_run(run_dir):
    success, success_t, final, _ = C.parse_launcher(run_dir)
    tr = C.read_state_trace(run_dir)
    if tr is None or final is None:
        return None
    fst = final.get("first_success_t")
    if fst is None and success:
        fst = success_t
    dur = float(tr["t"][-1] - tr["t"][0]) if len(tr["t"]) > 1 else 0.0
    steps = int(final.get("control_steps", 0))
    return dict(
        success=int(success),
        first_success_t="" if fst is None else f"{float(fst):.6g}",
        final_pos_err_m=f"{tr['pos_err'][-1]:.6g}",
        final_yaw_err_rad=f"{tr['ang_err'][-1]:.6g}",
        best_pos_err_m=f"{float(final.get('best_pos_err', float('nan'))):.6g}",
        best_yaw_err_rad=f"{float(final.get('best_ang_err', float('nan'))):.6g}",
        n_control_steps=steps,
        duration_trace_s=f"{dur:.6g}",
        solves_per_s=f"{(steps / dur) if dur > 0 else float('nan'):.6g}")


def main():
    only = set(sys.argv[1:]) or None
    per_variant = {v: [] for v in C.VARIANTS}
    for variant, scene, s, g, rid, rd in C.iter_runs():
        if only and rid not in only:
            continue
        r = eval_run(rd)
        if r is None:
            print(f"[SKIP incomplete] {rid}")
            continue
        per_variant[variant].append(dict(variant=variant, scene=scene,
                                         start=s, goal=g, run_id=rid, **r))
        print(f"{rid}: success={r['success']} t={r['first_success_t'] or '-'} "
              f"final={r['final_pos_err_m']}/{r['final_yaw_err_rad']} "
              f"steps={r['n_control_steps']} sps={r['solves_per_s']}")
    for variant, rows in per_variant.items():
        out = os.path.join(C.ROOT, "metrics", f"{variant}_all_runs.csv")
        C.upsert_csv(out, FIELDS, rows, ["variant", "scene", "start", "goal"])
        print(f"wrote {out} ({len(rows)} rows updated)")


if __name__ == "__main__":
    main()
