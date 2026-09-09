#!/usr/bin/env python3
"""Per-run safety metrics from footprint-boundary SDF samples.

Same body-point set + SDF machinery as chomp_eval.py (planner footprint ==
physical footprint, validated to microns). icra caveat: rows where the
state-trace quaternion gives |roll| or |pitch| > 0.1 rad are EXCLUDED from
all planar-SDF claims and counted separately (n_tilted_rows).

Output: results/c3plus_relu_chomp_comparison/metrics/safety_per_run.csv
(idempotent upsert). Usage: safety_metrics.py [run_id ...]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C

OUT = os.path.join(C.ROOT, "metrics", "safety_per_run.csv")
FIELDS = ["variant", "scene", "start", "goal", "run_id",
          "min_clearance_m", "n_neg_samples", "max_penetration_m",
          "time_below_5mm_s", "time_below_10mm_s", "contact_episodes",
          "n_rows_planar", "n_tilted_rows", "duration_s"]


def eval_run(scene, run_dir):
    sc = C.load_scene(scene)
    tr = C.read_state_trace(run_dir)
    if tr is None:
        return None
    tilt = (np.abs(tr["roll"]) > C.TILT_THRESH) | \
           (np.abs(tr["pitch"]) > C.TILT_THRESH)
    n_tilted = int(tilt.sum())
    t, x, y, yaw = tr["t"], tr["x"], tr["y"], tr["yaw"]
    duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    if not (sc["discs"] or sc["polys"]):
        return dict(min_clearance_m=float("inf"), n_neg_samples=0,
                    max_penetration_m=0.0, time_below_5mm_s=0.0,
                    time_below_10mm_s=0.0, contact_episodes=0,
                    n_rows_planar=int((~tilt).sum()), n_tilted_rows=n_tilted,
                    duration_s=duration)
    keep = ~tilt
    xk, yk, yawk, tk = x[keep], y[keep], yaw[keep], t[keep]
    bp = C.sample_boundary(sc["footprint"])
    cos, sin = np.cos(yawk), np.sin(yawk)
    xw = xk[:, None] + bp[:, 0][None, :] * cos[:, None] - \
        bp[:, 1][None, :] * sin[:, None]
    yw = yk[:, None] + bp[:, 0][None, :] * sin[:, None] + \
        bp[:, 1][None, :] * cos[:, None]
    K, U = xw.shape
    d = np.full((K, U), np.inf)
    for ox, oy, r in sc["discs"]:
        np.minimum(d, np.hypot(xw - ox, yw - oy) - r, out=d)
    if sc["polys"]:
        flatx, flaty = xw.ravel(), yw.ravel()
        for p in sc["polys"]:
            pdf = np.fromiter((C.poly_signed_dist(px, py, p)
                               for px, py in zip(flatx, flaty)),
                              dtype=float, count=K * U)
            np.minimum(d, pdf.reshape(K, U), out=d)
    dmin = d.min(axis=1)  # per-row min footprint clearance
    # per-row dwell time: half-gap to neighbors (trapezoid weights)
    if K > 1:
        dt = np.diff(tk)
        w = np.zeros(K)
        w[:-1] += dt / 2.0
        w[1:] += dt / 2.0
    else:
        w = np.zeros(K)
    below = dmin < 1e-3  # contact = d < 1 mm
    episodes, prev = 0, False
    for b in below:
        if b and not prev:
            episodes += 1
        prev = bool(b)
    return dict(
        min_clearance_m=float(dmin.min()) if K else float("nan"),
        n_neg_samples=int((d < 0).sum()),
        max_penetration_m=float(max(0.0, -d.min())) if K else 0.0,
        time_below_5mm_s=float(w[dmin < 5e-3].sum()),
        time_below_10mm_s=float(w[dmin < 1e-2].sum()),
        contact_episodes=episodes,
        n_rows_planar=K, n_tilted_rows=n_tilted, duration_s=duration)


def main():
    only = set(sys.argv[1:]) or None
    rows = []
    for variant, scene, s, g, rid, rd in C.iter_runs():
        if only and rid not in only:
            continue
        r = eval_run(scene, rd)
        if r is None:
            print(f"[SKIP empty trace] {rid}")
            continue
        rows.append(dict(variant=variant, scene=scene, start=s, goal=g,
                         run_id=rid,
                         **{k: (f"{v:.6g}" if isinstance(v, float) else v)
                            for k, v in r.items()}))
        print(f"{rid}: min_clr={r['min_clearance_m']:.4g} "
              f"neg={r['n_neg_samples']} pen={r['max_penetration_m']:.4g} "
              f"t<5mm={r['time_below_5mm_s']:.1f}s eps={r['contact_episodes']}"
              f" tilted={r['n_tilted_rows']}")
    C.upsert_csv(OUT, FIELDS, rows, ["variant", "scene", "start", "goal"])
    print(f"wrote {OUT} ({len(rows)} rows updated)")


if __name__ == "__main__":
    main()
