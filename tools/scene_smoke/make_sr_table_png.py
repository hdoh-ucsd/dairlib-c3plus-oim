#!/usr/bin/env python3
"""Success/failure table PNG for the pair campaign (600 s canonical tier,
trace-latch scoring, with a 200 s SR comparison column)."""
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = "results/xarm6_c3plus_scene_smoke"
cur = list(csv.DictReader(open(f"{BASE}/metrics/pair_campaign_summary.csv")))
old = list(csv.DictReader(open(f"{BASE}/metrics/pair_campaign_summary_200s_2lane.csv")))
SCENES = ["open_task", "single_obstacle", "shelf_gap", "ycb_clutter", "icra_sign", "slalom"]
PAIRS = ["01", "02", "03", "04", "05"]

def get(rows, s, p):
    return next(r for r in rows if r["scene"] == s and r["pair"] == p)

cell_text, cell_color = [], []
GREEN, RED, HDR = "#c8e6c9", "#ffcdd2", "#eceff1"
for s in SCENES:
    row_t, row_c = [], []
    for p in PAIRS:
        r = get(cur, s, p)
        if r["success_trace"] == "True":
            row_t.append(f"SUCCESS\n{float(r['t_success_trace']):.1f} s")
            row_c.append(GREEN)
        else:
            row_t.append(f"fail\n{float(r['final_position_error']):.2f} m / "
                         f"{float(r['final_orientation_error']):.2f} rad")
            row_c.append(RED)
    sr6 = sum(get(cur, s, p)["success_trace"] == "True" for p in PAIRS)
    sr2 = sum(get(old, s, p)["success"] == "True" for p in PAIRS)
    row_t += [f"{sr6}/5", f"{sr2}/5"]
    row_c += [GREEN if sr6 else RED, GREEN if sr2 else RED]
    cell_text.append(row_t)
    cell_color.append(row_c)

tot6 = sum(r["success_trace"] == "True" for r in cur)
tot2 = sum(r["success"] == "True" for r in old)
cell_text.append([""] * 5 + [f"{tot6}/30", f"{tot2}/30"])
cell_color.append([HDR] * 5 + [GREEN, GREEN])

fig, ax = plt.subplots(figsize=(13, 4.6))
ax.axis("off")
cols = [f"pair {int(p)}" for p in PAIRS] + ["SR (600 s)", "SR (200 s)"]
rows = SCENES + ["OVERALL"]
tbl = ax.table(cellText=cell_text, cellColours=cell_color,
               rowLabels=rows, colLabels=cols, loc="center",
               cellLoc="center")
tbl.auto_set_font_size(False)
tbl.set_fontsize(9)
tbl.scale(1, 2.2)
for j in range(len(cols)):
    tbl[0, j].set_facecolor(HDR)
    tbl[0, j].set_text_props(weight="bold")
for i in range(len(rows)):
    tbl[i + 1, 5].set_text_props(weight="bold")
    tbl[i + 1, 6].set_text_props(weight="bold")
ax.set_title("xArm6 C3+ pair campaign — success/failure by scene and start→goal pair\n"
             "600 s cap, 2 lanes, trace-latch scoring (pos < 0.05 m ∧ ang < 0.1 rad); "
             "fail cells show final pos/ang error", fontsize=11, pad=18)
fig.tight_layout()
for out in (f"{BASE}/figures/sr_table.png",
            f"{BASE}/videos_and_metrics/sr_table.png"):
    fig.savefig(out, dpi=180, bbox_inches="tight")
    print("wrote", out)
