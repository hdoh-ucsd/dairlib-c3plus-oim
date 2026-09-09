#!/usr/bin/env python3
"""5x5 grid heatmaps per scene/variant/metric.

Output: figures/heatmaps/<scene>_<variant>_<metric>.png for metrics
success, T_goal, min_clearance, chomp. Start rows 1-5, goal cols 1-5;
missing runs are gray (NaN). Idempotent. Usage: heatmaps.py
"""
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C

M = os.path.join(C.ROOT, "metrics")
OUT = os.path.join(C.ROOT, "figures", "heatmaps")

METRICS = {
    "success": ("success", "RdYlGn", "success (1=yes)", None),
    "T_goal": ("first_success_t", "viridis_r", "T_goal (s)", None),
    "min_clearance": ("safety_min_clearance_m", "viridis",
                      "min clearance (m)", None),
    "chomp": ("M_CHOMP", "magma_r", "M_CHOMP", None),
}


def read_csv(path):
    if not os.path.isfile(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def main():
    os.makedirs(OUT, exist_ok=True)
    idx = {}
    for v in C.VARIANTS:
        for r in read_csv(os.path.join(M, f"{v}_all_runs.csv")):
            idx[(v, r["scene"], int(r["start"]), int(r["goal"]))] = dict(r)
    for r in read_csv(os.path.join(M, "safety_per_run.csv")):
        k = (r["variant"], r["scene"], int(r["start"]), int(r["goal"]))
        idx.setdefault(k, {})["safety_min_clearance_m"] = r["min_clearance_m"]
    for r in read_csv(os.path.join(C.ROOT, "chomp", "per_run_chomp.csv")):
        k = (r["variant"], r["scene"], int(r["start"]), int(r["goal"]))
        idx.setdefault(k, {})["M_CHOMP"] = r["M_CHOMP"]

    n_figs = 0
    for scene in C.SCENES:
        for variant in C.VARIANTS:
            if not any(k[0] == variant and k[1] == scene for k in idx):
                continue
            for mname, (field, cmap, label, _) in METRICS.items():
                grid = np.full((5, 5), np.nan)
                for s in range(1, 6):
                    for g in range(1, 6):
                        m = idx.get((variant, scene, s, g))
                        if m and field in m:
                            grid[s - 1, g - 1] = fnum(m[field])
                fig, ax = plt.subplots(figsize=(5.2, 4.6))
                cmo = matplotlib.colormaps[cmap].copy()
                cmo.set_bad("0.75")
                vmin, vmax = (0, 1) if mname == "success" else (None, None)
                im = ax.imshow(np.ma.masked_invalid(grid), cmap=cmo,
                               vmin=vmin, vmax=vmax)
                for s in range(5):
                    for g in range(5):
                        v = grid[s, g]
                        if not math.isnan(v):
                            txt = (f"{int(v)}" if mname == "success"
                                   else f"{v:.3g}")
                            ax.text(g, s, txt, ha="center", va="center",
                                    fontsize=8)
                ax.set_xticks(range(5), [f"g{g}" for g in range(1, 6)])
                ax.set_yticks(range(5), [f"s{s}" for s in range(1, 6)])
                ax.set_xlabel("goal")
                ax.set_ylabel("start")
                ax.set_title(f"{scene} / {variant} / {label}")
                fig.colorbar(im, ax=ax, shrink=0.85)
                fig.tight_layout()
                fp = os.path.join(OUT, f"{scene}_{variant}_{mname}.png")
                fig.savefig(fp, dpi=110)
                plt.close(fig)
                n_figs += 1
    print(f"wrote {n_figs} heatmaps -> {OUT}")


if __name__ == "__main__":
    main()
