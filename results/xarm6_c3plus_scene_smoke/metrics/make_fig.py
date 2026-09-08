import pandas as pd, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

BASE = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics/results/xarm6_c3plus_scene_smoke"
cls = pd.read_csv(f"{BASE}/metrics/failure_classification_600s.csv", dtype={"pair": str})
roster = pd.read_csv(f"{BASE}/metrics/pair_campaign_summary.csv", dtype={"pair": str})
scenes = ["open_task", "single_obstacle", "shelf_gap", "ycb_clutter", "icra_sign", "slalom"]
pairs = ["01", "02", "03", "04", "05"]
colors = {"SUCCESS": "#2e9e4f", "OBSTACLE_BLOCKED": "#c0392b",
          "EARLY_ROTATION_GRIND": "#8e44ad", "SLOW_MONOTONE_PROGRESS": "#2980b9",
          "OTHER_PLATEAU": "#7f8c8d"}
grid = {}
for _, r in roster.iterrows():
    grid[(r.scene, r.pair)] = "SUCCESS" if r.success_trace else None
for _, r in cls.iterrows():
    grid[(r.scene, r.pair)] = r["class"]
fig, ax = plt.subplots(figsize=(11, 5.5))
used = set()
for i, s in enumerate(scenes):
    for j, p in enumerate(pairs):
        c = grid[(s, p)]
        used.add(c)
        ax.add_patch(plt.Rectangle((j, len(scenes) - 1 - i), 1, 1,
                                   facecolor=colors[c], edgecolor="white", lw=2))
        lbl = c.replace("_", "\n")
        if c != "SUCCESS":
            note = cls[(cls.scene == s) & (cls.pair == p)].notes.values[0]
            if "CRASH" in str(note):
                lbl += "\n(crash)"
        ax.text(j + 0.5, len(scenes) - 1 - i + 0.5, lbl, ha="center", va="center",
                fontsize=6.5, color="white", weight="bold")
ax.set_xlim(0, 5); ax.set_ylim(0, 6)
ax.set_xticks([k + 0.5 for k in range(5)])
ax.set_xticklabels([f"pair{p}" for p in pairs])
ax.set_yticks([k + 0.5 for k in range(6)])
ax.set_yticklabels(scenes[::-1])
ax.set_title("xArm6 C3+ 600 s scene campaign - failure classes (trace-latch criterion)")
ax.tick_params(length=0)
handles = [mpatches.Patch(color=colors[c], label=c) for c in colors if c in used]
ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
plt.tight_layout()
plt.savefig(f"{BASE}/figures/failure_classes.png", dpi=150, bbox_inches="tight")
print("figure written")
