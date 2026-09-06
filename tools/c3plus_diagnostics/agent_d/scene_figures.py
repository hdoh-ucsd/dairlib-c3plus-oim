#!/usr/bin/env python3
"""Agent D: top-down scene figures — port geometry vs OIM original."""
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow

OUT = "results_agent_d/figures"

def t_outline(x, y, yaw, scale=1.0):
    # m01 T: crossbar 0.089x0.0198 @ +y, stem 0.0198x0.0794 (matches TFootprint())
    cb_w, cb_h, st_w, st_h = 0.089, 0.0198, 0.0198, 0.0794
    pts = []
    for (cx, cy, w, h) in [(0, 0.0099, cb_w, cb_h), (0, -0.0397, st_w, st_h)]:
        c, s = math.cos(yaw), math.sin(yaw)
        for dx, dy in [(-w/2,-h/2),(w/2,-h/2),(w/2,h/2),(-w/2,h/2),(-w/2,-h/2)]:
            pts.append((x + c*(cx+dx) - s*(cy+dy), y + s*(cx+dx) + c*(cy+dy)))
        pts.append((None, None))
    return pts

def draw_t(ax, x, y, yaw, color, label):
    pts = t_outline(x, y, yaw)
    seg = []
    for p in pts:
        if p[0] is None:
            xs, ys = zip(*seg)
            ax.plot(xs, ys, color=color, lw=1.5)
            seg = []
        else:
            seg.append(p)
    ax.plot([], [], color=color, label=label)

def base_ax(ax, title):
    ax.add_patch(Circle((0, 0), 0.75, fill=False, ls=":", color="gray"))
    ax.add_patch(Circle((0, 0), 0.30, fill=False, ls=":", color="gray"))
    ax.plot(0, 0, "ks", ms=8)
    ax.text(0.02, 0.02, "robot base", fontsize=7)
    ax.set_xlim(-0.15, 0.95); ax.set_ylim(-0.75, 0.75)
    ax.set_aspect("equal"); ax.grid(alpha=0.2); ax.set_title(title, fontsize=9)

# ---- ycb_clutter
fig, axes = plt.subplots(1, 2, figsize=(12, 6))
ax = axes[0]
base_ax(ax, "PORT ycb_clutter (sim+planner)\n2 boxes; planner discs dashed")
ax.add_patch(Rectangle((0.45, -0.05), 0.1, 0.1, color="tab:orange", alpha=0.6))
ax.add_patch(Rectangle((0.37 - 0.0875, 0.2 - 0.0475), 0.175, 0.095, color="tab:orange", alpha=0.6))
ax.add_patch(Circle((0.5, 0.0), 0.0707, fill=False, ls="--", color="red"))
ax.add_patch(Circle((0.37, 0.2), 0.0996, fill=False, ls="--", color="red"))
draw_t(ax, 0.5, 0.3, 0.0, "tab:blue", "T start")
draw_t(ax, 0.5, -0.3, 0.0, "tab:green", "T goal")
ax.legend(fontsize=7)
ax = axes[1]
base_ax(ax, "OIM ycb_clutter original (4 obstacles)\ngray = dropped by port")
ax.add_patch(Rectangle((0.30, -0.05), 0.1, 0.1, color="tab:orange", alpha=0.6))       # cube 0.35,0
ax.add_patch(Rectangle((0.22-0.0875, 0.2-0.0475), 0.175, 0.095, color="tab:orange", alpha=0.6))  # sugar
ax.add_patch(Rectangle((0.60-0.051, -0.16-0.030), 0.102, 0.060, color="gray", alpha=0.7))  # spam can bbox
ax.add_patch(Rectangle((0.15-0.049, -0.18-0.033), 0.097, 0.067, color="gray", alpha=0.7))  # mustard bbox
draw_t(ax, 0.381, 0.4, 0.0, "tab:blue", "T start (OIM)")
draw_t(ax, 0.5, -0.4, 0.0, "tab:green", "T goal (OIM band)")
ax.legend(fontsize=7)
fig.tight_layout()
fig.savefig(f"{OUT}/ycb_clutter_topdown.png", dpi=130)

# ---- icra_sign
fig, axes = plt.subplots(1, 2, figsize=(12, 6))
ax = axes[0]
base_ax(ax, "PORT icra_sign: NO obstacles anywhere\n(T to C-slot pose on open table)")
draw_t(ax, 0.5, 0.3, 0.0, "tab:blue", "T start")
draw_t(ax, 0.5, -0.4, math.pi / 2, "tab:green", "goal (0.5,-0.4,+90deg)")
ax.legend(fontsize=7)
ax = axes[1]
base_ax(ax, "OIM icra_sign original: C glyph into slot\nglyph row x=[0.4485,0.5515]")
glyphs = [("I", -0.55), ("R", -0.25), ("A", -0.1), ("2", 0.15), ("0", 0.3), ("2b", 0.45), ("6", 0.6)]
for name, y in glyphs:
    ax.add_patch(Rectangle((0.4485, y - 0.0515), 0.103, 0.103, color="tab:red", alpha=0.5))
    ax.text(0.5, y, name, ha="center", va="center", fontsize=7)
ax.add_patch(Rectangle((0.4485, -0.4 - 0.0515), 0.103, 0.103, fill=False, ls="--", color="tab:green"))
ax.text(0.5, -0.4, "C slot", ha="center", va="center", fontsize=7, color="tab:green")
# C start
ax.add_patch(Rectangle((0.3 - 0.0483, 0.4 - 0.0515), 0.0966, 0.103, fill=False, color="tab:blue"))
ax.text(0.3, 0.4, "C start", ha="center", fontsize=7, color="tab:blue")
ax.legend(fontsize=7)
fig.tight_layout()
fig.savefig(f"{OUT}/icra_sign_topdown.png", dpi=130)

# passage math
slot_gap = (-0.25 - 0.0515) - (-0.55 + 0.0515)   # inner edges of R and I
c_width = 0.103
print("OIM icra slot passage (I..R inner gap):", round(slot_gap, 4), "m; C footprint:", c_width,
      "m; side clearance:", round((slot_gap - c_width) / 2, 4), "m; pusher dia:", 0.039)
print("Figures written to", OUT)
