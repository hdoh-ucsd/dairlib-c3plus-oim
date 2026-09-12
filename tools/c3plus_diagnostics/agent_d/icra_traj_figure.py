#!/usr/bin/env python3
"""Representative trajectory figure for the faithful icra_sign baseline."""
import json, math, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly, Circle as MplCircle

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
run = sys.argv[1] if len(sys.argv) > 1 else f"{WT}/results_icra_port/runs/v5"
tag = os.path.basename(run)
polys = []
for entry in open(f"{WT}/results_icra_port/obstacles/obs_polys_env.txt").read().strip().split(";"):
    parts = entry.split("|")
    polys.append([tuple(map(float, p.split(","))) for p in parts[1:]])
xs, ys, ts = [], [], []
for line in open(f"{run}/state_trace.jsonl"):
    r = json.loads(line)
    xs.append(r["obj"][4]); ys.append(r["obj"][5]); ts.append(r["t"])
fig, ax = plt.subplots(figsize=(7, 10))
for i, v in enumerate(polys):
    ax.add_patch(MplPoly(v, closed=True, color="tab:red", alpha=0.45))
ax.add_patch(MplCircle((0, 0), 0.09, fill=False, ls="--", color="k"))
sc = ax.scatter(xs, ys, c=ts, s=3, cmap="viridis")
plt.colorbar(sc, ax=ax, label="t (s)")
ax.plot(xs[0], ys[0], "bs", ms=8, label="C start")
ax.plot(0.4845 if "v5" in tag else 0.5, -0.4286 if "v5" in tag else -0.4,
        "g*", ms=14, label="goal slot")
ax.set_aspect("equal"); ax.grid(alpha=0.2); ax.legend(fontsize=8)
ax.set_xlim(-0.15, 0.85); ax.set_ylim(-0.75, 0.75)
ax.set_title(f"Faithful icra_sign baseline — C trajectory ({tag})")
fig.tight_layout()
fig.savefig(f"{WT}/results_icra_port/figures/icra_sign_trajectory_{tag}.png", dpi=130)
print("wrote trajectory figure", tag)
