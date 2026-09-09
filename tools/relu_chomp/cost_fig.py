#!/usr/bin/env python3
"""Per-run cost-diagnostics figure for the relu/baseline grid runs.

Standalone wrapper around the tools/scene_smoke/cost_diagnostics_v2.py cost
reconstruction (that script is coupled to the old scene-smoke campaign layout
and failure-classification CSVs; its baseline behavior is untouched).

--variant baseline: obstacle curve = exp reconstruction
    sum_obs 5000*exp(-(d_center - r)/0.04)   (object CENTER distance)
--variant relu: obstacle curve labeled 'obstacle_ReLU' =
    200 * max(0, (0.01 - d_fp)/0.01)^2       (min FOOTPRINT distance)

Usage: cost_fig.py --run-dir DIR --scene SCENE --variant baseline|relu
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C

W_XY, W_PRE = 10000.0, 12500.0
W_ROT_POST, W_ROT_PRE = 510.0, 5.0
LATCH = 0.25
W_OBS, SIG_OBS = 5000.0, 0.04
RELU_W, RELU_EPS = 200.0, 0.01


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def read_metrics(run_dir):
    fns = [f for f in os.listdir(run_dir) if f.endswith("_metrics.csv")]
    if not fns:
        raise SystemExit(f"no *_metrics.csv in {run_dir} "
                         "(run postprocess_run.py first)")
    fn = fns[0]
    cols = ["control_step", "object_x", "object_y", "object_yaw", "goal_x",
            "goal_y", "goal_yaw", "position_error_m", "orientation_error_rad"]
    data = {c: [] for c in cols}
    with open(os.path.join(run_dir, fn)) as f:
        for row in csv.DictReader(f):
            for c in cols:
                data[c].append(float(row[c]))
    return fn[:-len("_metrics.csv")], {c: np.asarray(v)
                                       for c, v in data.items()}


def obs_curve(variant, scene, x, y, yaw):
    sc = C.load_scene(scene)
    discs, polys = sc["discs"], sc["polys"]
    if not (discs or polys):
        return np.zeros_like(x), False
    if variant == "baseline":
        j = np.zeros_like(x)
        for ox, oy, r in discs:
            j += W_OBS * np.exp(-(np.hypot(x - ox, y - oy) - r) / SIG_OBS)
        for p in polys:
            for i in range(len(x)):
                j[i] += W_OBS * math.exp(
                    -C.poly_signed_dist(x[i], y[i], p) / SIG_OBS)
        return j, True
    # relu: footprint distance
    bp = C.sample_boundary(sc["footprint"])
    cos, sin = np.cos(yaw), np.sin(yaw)
    xw = x[:, None] + bp[:, 0][None, :] * cos[:, None] - \
        bp[:, 1][None, :] * sin[:, None]
    yw = y[:, None] + bp[:, 0][None, :] * sin[:, None] + \
        bp[:, 1][None, :] * cos[:, None]
    d = np.full(xw.shape, np.inf)
    for ox, oy, r in discs:
        np.minimum(d, np.hypot(xw - ox, yw - oy) - r, out=d)
    if polys:
        flat = list(zip(xw.ravel(), yw.ravel()))
        for p in polys:
            pdf = np.fromiter((C.poly_signed_dist(px, py, p)
                               for px, py in flat), dtype=float,
                              count=len(flat))
            np.minimum(d, pdf.reshape(xw.shape), out=d)
    d_fp = d.min(axis=1)
    return RELU_W * np.maximum(0.0, (RELU_EPS - d_fp) / RELU_EPS) ** 2, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--variant", required=True,
                    choices=["baseline", "relu"])
    a = ap.parse_args()
    stem, m = read_metrics(a.run_dir)
    k = m["control_step"]
    epos = m["position_error_m"]
    eth = m["orientation_error_rad"]
    dx = m["object_x"] - m["goal_x"]
    dy = m["object_y"] - m["goal_y"]
    eyaw = wrap(m["object_yaw"] - m["goal_yaw"])
    latched = np.zeros(len(k), bool)
    if np.any(epos < LATCH):
        latched[int(np.argmax(epos < LATCH)):] = True
    j_trans = np.where(latched, W_XY * (dx**2 + dy**2),
                       W_PRE * (dx**2 + dy**2))
    j_rot = np.where(latched, W_ROT_POST * eyaw**2, W_ROT_PRE * eyaw**2)
    j_task = j_trans + j_rot
    j_obs, has_obs = obs_curve(a.variant, a.scene, m["object_x"],
                               m["object_y"], m["object_yaw"])
    obs_label = ("obstacle_ReLU" if a.variant == "relu" else "obstacle") + \
        ("" if has_obs else " (no obstacles)")

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle(f"{stem} ({a.scene}, {a.variant})")
    axL.plot(k, epos, lw=1.4, label="position error (m)")
    axL.plot(k, eth, lw=1.4, label="orientation error (rad)")
    axL.set_title("Task diagnostics")
    axL.set_xlabel("control step")
    axL.legend(loc="upper right")
    axL.grid(alpha=0.3)
    axR.plot(k, j_trans, lw=1.0, label=f"translation (Σ {np.nansum(j_trans):.3g})")
    axR.plot(k, j_rot, lw=1.0, label=f"orientation (Σ {np.nansum(j_rot):.3g})")
    axR.plot(k, j_obs, lw=1.0, label=f"{obs_label} (Σ {np.nansum(j_obs):.3g})")
    axR.plot(k, j_task, color="black", lw=2.2,
             label="total (translation+orientation)")
    axR.set_yscale("symlog", linthresh=1e-3)
    axR.set_title("C3+ cost decomposition")
    axR.set_xlabel("control step")
    axR.set_ylabel("cost per control step")
    axR.legend(loc="upper right", fontsize=8)
    axR.grid(alpha=0.3)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out = os.path.join(a.run_dir, stem + "_cost_diagnostics.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
