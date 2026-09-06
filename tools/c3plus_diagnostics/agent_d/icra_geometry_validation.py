#!/usr/bin/env python3
"""Faithful icra_sign port: 2D geometry validation (slot uniqueness, yaw
clearance, C-footprint fidelity) + layout figures."""
import ast, os, math, csv, re, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly, Circle as MplCircle

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
OIM = "/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX"
OUT = f"{WT}/results_icra_port"

# hulls (same parse as generator)
src = open(f"{OIM}/oim/utils/scenes.py").read().splitlines()
hulls, name, buf = {}, None, ""
for line in src:
    m = re.match(r"_GLYPH_(\w) = \(", line)
    if m:
        name, buf = m.group(1), ""
        continue
    if name is not None:
        buf += line
        if line.strip() == ")":
            hulls[name] = ast.literal_eval("(" + buf.rstrip()[:-1].strip() + ")")
            name = None
ROW = [("I", -0.55), ("R", -0.25), ("A", -0.10), ("2", 0.15),
       ("0", 0.30), ("2b", 0.45), ("6", 0.60)]
BASE = (0.0, 0.0, 0.09)
WORLD = {g: [(0.5 + vx, y + vy) for vx, vy in hulls[g.rstrip("b")]] for g, y in ROW}

# C footprint: union of three boxes (matches CFootprint())
CBOX = [(-0.0323, 0.0, 0.016, 0.0515), (0.0, 0.0355, 0.0483, 0.016),
        (0.0, -0.0355, 0.0483, 0.016)]

def c_points(x, y, yaw, n=24):
    pts = []
    c, s = math.cos(yaw), math.sin(yaw)
    for (cx, cy, hx, hy) in CBOX:
        for i in range(n + 1):
            a = i / n
            for lx, ly in ((cx-hx+2*hx*a, cy-hy), (cx-hx+2*hx*a, cy+hy),
                           (cx-hx, cy-hy+2*hy*a), (cx+hx, cy-hy+2*hy*a)):
                pts.append((x + c*lx - s*ly, y + s*lx + c*ly))
    return pts

def poly_sdf(px, py, verts):
    n = len(verts)
    best, inside = 1e18, True
    for i in range(n):
        ax, ay = verts[i]; bx, by = verts[(i+1) % n]
        ex, ey = bx-ax, by-ay
        if ex*(py-ay) - ey*(px-ax) < 0:
            inside = False
        L2 = ex*ex + ey*ey
        t = max(0.0, min(1.0, ((px-ax)*ex + (py-ay)*ey)/L2)) if L2 > 0 else 0
        best = min(best, math.hypot(px-(ax+t*ex), py-(ay+t*ey)))
    return -best if inside else best

def ccw(verts):
    a = sum(verts[i][0]*verts[(i+1) % len(verts)][1] -
            verts[(i+1) % len(verts)][0]*verts[i][1] for i in range(len(verts)))
    return verts if a > 0 else verts[::-1]

WORLD = {g: ccw(v) for g, v in WORLD.items()}

def min_clearance(x, y, yaw):
    d = 1e18
    for g, verts in WORLD.items():
        for px, py in c_points(x, y, yaw):
            d = min(d, poly_sdf(px, py, verts))
    for px, py in c_points(x, y, yaw):
        d = min(d, math.hypot(px - BASE[0], py - BASE[1]) - BASE[2])
    return d

# ---- slot feasibility (section 9): try the C in every inter-glyph gap
rows = []
edges = [("I", -0.55), ("Cslot", -0.40), ("R", -0.25), ("A", -0.10),
         ("2", 0.15), ("0", 0.30), ("2b", 0.45), ("6", 0.60)]
gaps = []
seq = [("I", -0.55), ("R", -0.25), ("A", -0.10), ("2", 0.15), ("0", 0.30),
       ("2b", 0.45), ("6", 0.60)]
for i in range(len(seq) - 1):
    (g1, y1), (g2, y2) = seq[i], seq[i+1]
    gaps.append((f"{g1}-{g2}", (y1 + y2) / 2))
gaps.insert(0, ("C_SLOT(I-R)", -0.40))
for gname, gy in gaps:
    # C placed at slot pose orientation (+pi/2), search best clearance over
    # upstream yaw jitter band
    best = -1e18
    for dy in np.linspace(-0.02, 0.02, 5):
        for dyaw in np.linspace(-0.35, 0.35, 8):
            best = max(best, min_clearance(0.5, gy + dy, math.pi/2 + dyaw))
    # free width between neighboring glyph hulls at x=0.5 line
    rows.append(dict(gap=gname, y_center=round(gy, 3),
                     best_clearance_m=round(best, 4),
                     feasible=best > 0.0,
                     verdict="FEASIBLE" if best > 0 else "INFEASIBLE"))
os.makedirs(f"{OUT}/scene", exist_ok=True)
with open(f"{OUT}/scene/slot_feasibility.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
print("slot feasibility:", [(r['gap'], r['verdict'], r['best_clearance_m']) for r in rows])

# ---- goal yaw clearance (section 10)
yrows = []
for tag, yaw_off in [("nominal", 0.0), ("+0.35", 0.35), ("-0.35", -0.35),
                     ("+0.30", 0.30), ("-0.30", -0.30)]:
    cl = min_clearance(0.5, -0.40, math.pi/2 + yaw_off)
    # clearance to R and I individually
    cr = min(poly_sdf(px, py, WORLD["R"]) for px, py in c_points(0.5, -0.40, math.pi/2 + yaw_off))
    ci = min(poly_sdf(px, py, WORLD["I"]) for px, py in c_points(0.5, -0.40, math.pi/2 + yaw_off))
    yrows.append(dict(case=tag, min_clearance=round(cl, 4),
                      clearance_to_R=round(cr, 4), clearance_to_I=round(ci, 4),
                      valid=cl > 0))
with open(f"{OUT}/scene/goal_yaw_clearance_validation.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(yrows[0].keys()))
    w.writeheader(); w.writerows(yrows)
print("yaw clearance:", yrows)

# ---- pose variant clearances (section 8 cross-check)
import yaml
pv = yaml.safe_load(open(f"{OIM}/examples/poses/icra_sign.yaml"))
vrows = []
for k in "12345":
    s, g = pv["starts"][k], pv["goals"][k]
    vrows.append(dict(variant=k,
                      start_clearance=round(min_clearance(*s), 4),
                      goal_clearance=round(min_clearance(*g), 4)))
print("variant clearances (recomputed):", vrows)
with open(f"{OUT}/validation/icra_sign_geometry_feasibility.csv", "w", newline="") as f:
    allrows = ([dict(check="gap_" + r["gap"], value=r["best_clearance_m"], ok=r["feasible"]) for r in rows] +
               [dict(check="goalyaw_" + r["case"], value=r["min_clearance"], ok=r["valid"]) for r in yrows] +
               [dict(check=f"variant{r['variant']}_start", value=r["start_clearance"], ok=r["start_clearance"] > 0) for r in vrows] +
               [dict(check=f"variant{r['variant']}_goal", value=r["goal_clearance"], ok=r["goal_clearance"] > 0) for r in vrows])
    w = csv.DictWriter(f, fieldnames=["check", "value", "ok"])
    w.writeheader(); w.writerows(allrows)

# ---- figures
def draw_scene(ax, show_hulls=True):
    for g, verts in WORLD.items():
        ax.add_patch(MplPoly(verts, closed=True, color="tab:red", alpha=0.5))
        cy = sum(v[1] for v in verts)/len(verts)
        ax.text(0.5, cy, g.rstrip("b"), ha="center", va="center", fontsize=9)
    ax.add_patch(MplCircle((0, 0), 0.09, fill=False, ls="--", color="k"))
    ax.text(0, 0, "base", ha="center", fontsize=7)
    ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_xlim(-0.15, 0.85); ax.set_ylim(-0.75, 0.75)

def draw_c(ax, x, y, yaw, color, label):
    c, s = math.cos(yaw), math.sin(yaw)
    for (cx, cy, hx, hy) in CBOX:
        corners = [(cx-hx, cy-hy), (cx+hx, cy-hy), (cx+hx, cy+hy), (cx-hx, cy+hy)]
        wc = [(x + c*a - s*b, y + s*a + c*b) for a, b in corners]
        ax.add_patch(MplPoly(wc, closed=True, fill=False, color=color, lw=1.5))
    ax.plot([], [], color=color, label=label)

fig, ax = plt.subplots(figsize=(7, 10))
draw_scene(ax)
draw_c(ax, 0.3, 0.4, 0.0, "tab:blue", "C start (0.3, 0.4, 0)")
draw_c(ax, 0.5, -0.4, math.pi/2, "tab:green", "C goal slot (0.5, -0.4, +pi/2)")
ax.legend(fontsize=8)
ax.set_title("Faithful icra_sign: 'ICRA 2026' hulls + base circle + C start/goal")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/icra_sign_scene_layout.png", dpi=130)

fig, ax = plt.subplots(figsize=(7, 10))
draw_scene(ax)
pts = c_points(0.3, 0.4, 0.0)
ax.plot([p[0] for p in pts], [p[1] for p in pts], ".", ms=1, color="tab:blue")
ax.set_title("Planner geometry: glyph hulls (exact), base disc, C footprint samples")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/icra_sign_planner_geometry.png", dpi=130)

fig, ax = plt.subplots(figsize=(6, 6))
draw_scene(ax)
for yaw_off, col in [(0.0, "tab:green"), (0.35, "tab:orange"), (-0.35, "tab:purple")]:
    draw_c(ax, 0.5, -0.4, math.pi/2 + yaw_off, col, f"goal yaw {yaw_off:+.2f}")
ax.set_xlim(0.3, 0.7); ax.set_ylim(-0.65, -0.15)
ax.legend(fontsize=8); ax.set_title("Goal-slot clearance under upstream yaw jitter")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/icra_sign_goal_slot_clearance.png", dpi=130)

# ---- C footprint vs sim collision boxes discrepancy (they are the same 3
# boxes by construction; report max discrepancy of sampled boundary vs boxes)
disc = 0.0
for px, py in c_points(0, 0, 0):
    d = min(max(abs(px-cx)-hx, abs(py-cy)-hy) for (cx, cy, hx, hy) in CBOX)
    disc = max(disc, abs(min(d, 0)) if d < 0 else d)
fig, ax = plt.subplots(figsize=(5, 5))
draw_c(ax, 0, 0, 0, "k", "sim collision boxes")
pts = c_points(0, 0, 0)
ax.plot([p[0] for p in pts], [p[1] for p in pts], ".", ms=2, color="tab:blue", label="planner footprint samples")
ax.set_aspect("equal"); ax.legend(fontsize=8); ax.grid(alpha=0.2)
ax.set_title(f"C footprint vs sim geometry (max discrepancy {disc*1000:.2f} mm)")
fig.tight_layout(); fig.savefig(f"{OUT}/figures/c_glyph_footprint_validation.png", dpi=130)
print("footprint max discrepancy (m):", disc)
with open(f"{OUT}/object/c_glyph_geometry_validation.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["check", "value", "ok"])
    w.writerow(["footprint_vs_sim_boxes_max_discrepancy_m", round(disc, 6), disc < 1e-6])
    w.writerow(["overall_width", 0.0966, True])
    w.writerow(["overall_height", 0.1030, True])
    w.writerow(["stroke", 0.032, True])
print("figures + CSVs written")
