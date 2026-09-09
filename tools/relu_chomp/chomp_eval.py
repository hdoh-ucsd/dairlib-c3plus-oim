#!/usr/bin/env python3
"""CHOMP-style obstacle-cost functional over each run's object trajectory.

Evaluation-only metric; independent of any controller ranking cost (see
chomp_definition.md written next to the output CSV).

Output: results/c3plus_relu_chomp_comparison/chomp/per_run_chomp.csv
(append/update per run; idempotent). Usage: chomp_eval.py [run_id ...]
where run_id = variant/scene/sMMgNN (default: all complete runs).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C

OUT = os.path.join(C.ROOT, "chomp", "per_run_chomp.csv")
FIELDS = ["variant", "scene", "start", "goal", "run_id", "M_CHOMP",
          "M_CHOMP_norm", "path_length_m", "n_states"]


def chomp_cost(d, eps=C.CHOMP_EPS):
    """c(d) = -d + eps/2 (d<0); (d-eps)^2/(2 eps) (0<=d<=eps); 0 (d>eps)."""
    c = np.zeros_like(d)
    neg = d < 0
    mid = (~neg) & (d <= eps)
    c[neg] = -d[neg] + eps / 2.0
    c[mid] = (d[mid] - eps) ** 2 / (2.0 * eps)
    return c


def eval_run(scene, run_dir):
    sc = C.load_scene(scene)
    tr = C.read_state_trace(run_dir)
    if tr is None:
        return None
    bp = C.sample_boundary(sc["footprint"])  # (U,2) object frame
    x, y, yaw = tr["x"], tr["y"], tr["yaw"]
    K, U = len(x), len(bp)
    # center path length
    path_len = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    if not (sc["discs"] or sc["polys"]):
        return dict(M_CHOMP=0.0, M_CHOMP_norm=0.0, path_length_m=path_len,
                    n_states=K)
    cos, sin = np.cos(yaw), np.sin(yaw)
    # world positions of every body point at every state: (K,U)
    xw = x[:, None] + bp[:, 0][None, :] * cos[:, None] - \
        bp[:, 1][None, :] * sin[:, None]
    yw = y[:, None] + bp[:, 0][None, :] * sin[:, None] + \
        bp[:, 1][None, :] * cos[:, None]
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
    c = chomp_cost(d)
    seg = np.hypot(np.diff(xw, axis=0), np.diff(yw, axis=0))  # (K-1,U)
    M = float(np.sum(0.5 * (c[:-1] + c[1:]) * seg))
    return dict(M_CHOMP=M,
                M_CHOMP_norm=(M / path_len if path_len > 0 else 0.0),
                path_length_m=path_len, n_states=K)


DEFN = """# CHOMP-style obstacle metric — definition

Evaluation-only functional computed OFFLINE from `state_trace.jsonl` object
poses and the scene geometry in `tools/scene_smoke/scene_configs/<scene>.yaml`.

## Cost function
With eps = 0.01 m and d the min signed distance (m) from a body point to all
scene obstacles (polygon and disc SDFs; negative inside):

    c(d) = -d + eps/2            if d < 0
    c(d) = (d - eps)^2 / (2 eps) if 0 <= d <= eps
    c(d) = 0                     if d > eps

## Body points
A FIXED set of points in the object frame: the footprint polygon boundary of
the scene config, sampled every 5 mm (each edge split into
ceil(edge_len/0.005) equal parts; every vertex included exactly once). The
same set is used for every run of a scene, transformed rigidly by the planar
object pose (x, y, yaw from the state-trace quaternion) at each state k.

## Functional
    M_CHOMP = sum_k sum_u 0.5 * (c(d_{k,u}) + c(d_{k+1,u})) * ||x_{k+1,u} - x_{k,u}||
    M_CHOMP_norm = M_CHOMP / L_center     (0 if L_center = 0)

where x_{k,u} is the world position of body point u at state k and L_center
is the total object CENTER path length. Scenes without obstacles (open_task)
score exactly 0.

## Independence from the controller ranking cost
This metric is NOT the controllers' obstacle ranking term. The baseline
variant ranks with 5000*exp(-(d_center - r)/0.04) on the object CENTER; the
ReLU variant ranks with 200*max(0, (0.01 - d_fp)/0.01)^2 on the footprint.
M_CHOMP differs from both in functional form (piecewise CHOMP potential),
integration (path-length-weighted trapezoid over a dense boundary point set),
and role: it is never fed back to any controller — it is a neutral
evaluation yardstick applied identically to both variants.
"""


def main():
    only = set(sys.argv[1:]) or None
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    dpath = os.path.join(os.path.dirname(OUT), "chomp_definition.md")
    with open(dpath, "w") as f:
        f.write(DEFN)
    rows = []
    for variant, scene, s, g, rid, rd in C.iter_runs():
        if only and rid not in only:
            continue
        r = eval_run(scene, rd)
        if r is None:
            print(f"[SKIP empty trace] {rid}")
            continue
        rows.append(dict(variant=variant, scene=scene, start=s, goal=g,
                         run_id=rid, **{k: f"{v:.6g}" if isinstance(v, float)
                                        else v for k, v in r.items()}))
        print(f"{rid}: M_CHOMP={r['M_CHOMP']:.6g} norm={r['M_CHOMP_norm']:.6g}"
              f" L={r['path_length_m']:.3f} K={r['n_states']}")
    C.upsert_csv(OUT, FIELDS, rows, ["variant", "scene", "start", "goal"])
    print(f"wrote {OUT} ({len(rows)} rows updated)")


if __name__ == "__main__":
    main()
