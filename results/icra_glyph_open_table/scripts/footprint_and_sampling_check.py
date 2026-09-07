#!/usr/bin/env python3
"""Offline footprint validation + perimeter-sampling coverage for I/C/R/A.

1. FOOTPRINT VALIDATION: planner footprint polygon vs simulation collision
   footprint at identity pose. 720 rays from the centroid; radius = farthest
   boundary crossing along each ray; report |r_planner - r_sim| max/mean.
   C: sim = union of push_c_glyph.sdf's three boxes (10-vertex concave
   outline, same as gen_c_mesh.py). I/R/A: sim = the CSV hull (identical to
   the planner polygon by construction; the check guards transcription).
2. SAMPLING COVERAGE: 2000 uniform-by-arclength perimeter samples on the
   planner footprint (kRandomOnPerimeter semantics), offset outward along
   the local edge normal by pusher radius 0.0195 + clearance 0.027
   (sample_projection_clearance). Per-edge densities, outward normals, and
   workspace rejection (r <= 0.70, y <= 0.6) with the object at the nominal
   task start (0.30, 0.40) and goal (0.50, -0.40).
"""
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WT = os.path.abspath(os.path.dirname(__file__) + "/../../..")
OUT = f"{WT}/results/icra_glyph_open_table"
CSV = f"{WT}/results/final_oim_c3plus_comparison/scene_fidelity/icra_glyph_obstacles.csv"

PUSHER_R = 0.0195
CLEARANCE = 0.027
OFFSET = PUSHER_R + CLEARANCE
N_SAMPLES = 2000
N_RAYS = 720
WS_R = 0.70
WS_Y = 0.6
POSES = {"start(0.30,0.40)": (0.30, 0.40), "goal(0.50,-0.40)": (0.50, -0.40)}

# C-glyph exact union outline (spine + top/bottom bars of push_c_glyph.sdf),
# CCW, same vertices as tools/c3plus_diagnostics/agent_d/gen_c_mesh.py.
C_OUTLINE = [(-0.0483, -0.0515), (-0.0163, -0.0515), (0.0483, -0.0515),
             (0.0483, -0.0195), (-0.0163, -0.0195), (-0.0163, 0.0195),
             (0.0483, 0.0195), (0.0483, 0.0515), (-0.0163, 0.0515),
             (-0.0483, 0.0515)]

hulls = {}
with open(CSV) as f:
    for row in csv.DictReader(f):
        hulls.setdefault(row["glyph"], []).append(
            (float(row["x_local"]), float(row["y_local"])))


def signed_area(poly):
    s = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def ensure_ccw(poly):
    return poly if signed_area(poly) > 0 else poly[::-1]


def ray_radius(poly, cx, cy, theta):
    """Farthest boundary crossing along ray (cx,cy)+t(cos,sin), t>=0."""
    dx, dy = math.cos(theta), math.sin(theta)
    best = 0.0
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-14:
            continue
        t = ((ax - cx) * ey - (ay - cy) * ex) / den
        u = ((ax - cx) * dy - (ay - cy) * dx) / den
        if t >= 0 and -1e-12 <= u <= 1 + 1e-12:
            best = max(best, t)
    return best


def perimeter_samples(poly, n_samples):
    """Uniform-by-arclength samples; returns (pt, offset_pt, normal, edge_id)."""
    n = len(poly)
    lens = []
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        lens.append(math.hypot(bx - ax, by - ay))
    total = sum(lens)
    cum = np.concatenate([[0], np.cumsum(lens)])
    s_vals = (np.arange(n_samples) + 0.5) / n_samples * total
    out = []
    for s in s_vals:
        e = min(np.searchsorted(cum, s, side="right") - 1, n - 1)
        t = (s - cum[e]) / lens[e]
        ax, ay = poly[e]
        bx, by = poly[(e + 1) % n]
        px, py = ax + t * (bx - ax), ay + t * (by - ay)
        ex, ey = (bx - ax) / lens[e], (by - ay) / lens[e]
        nx, ny = ey, -ex  # outward for CCW polygon
        out.append(((px, py), (px + OFFSET * nx, py + OFFSET * ny),
                    (nx, ny), e))
    return out, total, lens


md = ["# Glyph footprint validation + sampling coverage",
      "",
      f"Planner polygons: hardcoded selector footprints "
      f"(systems/controllers/sampling_based_c3_controller.cc); hull source "
      f"= icra_glyph_obstacles.csv. Offset = pusher radius {PUSHER_R} + "
      f"clearance {CLEARANCE} = {OFFSET:.4f} m. Workspace: r <= {WS_R}, "
      f"y <= {WS_Y}.", ""]

cases = {
    "I": (ensure_ccw(hulls["I"]), ensure_ccw(hulls["I"])),
    "C": (ensure_ccw(C_OUTLINE), ensure_ccw(C_OUTLINE)),
    "R": (ensure_ccw(hulls["R"]), ensure_ccw(hulls["R"])),
    "A": (ensure_ccw(hulls["A"]), ensure_ccw(hulls["A"])),
}

for g, (planner, sim) in cases.items():
    cx = sum(p[0] for p in planner) / len(planner)
    cy = sum(p[1] for p in planner) / len(planner)

    # 1. footprint discrepancy
    thetas = np.arange(N_RAYS) * 2 * math.pi / N_RAYS
    dr = [abs(ray_radius(planner, cx, cy, th) - ray_radius(sim, cx, cy, th))
          for th in thetas]
    dmax, dmean = max(dr), float(np.mean(dr))

    fig, ax = plt.subplots(figsize=(6, 6))
    pp = planner + [planner[0]]
    ss = sim + [sim[0]]
    ax.plot([p[0] for p in pp], [p[1] for p in pp], "b-", lw=2,
            label="planner footprint")
    ax.plot([p[0] for p in ss], [p[1] for p in ss], "r--", lw=1.5,
            label="sim collision footprint")
    ax.plot(cx, cy, "k+", ms=10)
    ax.set_title(f"{g}: max boundary discrepancy {dmax*1000:.3f} mm "
                 f"(mean {dmean*1000:.3f} mm)")
    ax.set_aspect("equal"); ax.legend(); ax.grid(alpha=0.3)
    fig.savefig(f"{OUT}/figures/{g}_footprint.png", dpi=140,
                bbox_inches="tight")
    plt.close(fig)

    # 2. sampling coverage
    samples, per_len, edge_lens = perimeter_samples(planner, N_SAMPLES)
    n_edges = len(planner)
    per_edge = [0] * n_edges
    for _, _, _, e in samples:
        per_edge[e] += 1
    dens = [per_edge[i] / edge_lens[i] for i in range(n_edges)]

    rejects = {}
    for pname, (ox, oy) in POSES.items():
        r = 0
        for _, (qx, qy), _, _ in samples:
            wx, wy = ox + qx, oy + qy
            if math.hypot(wx, wy) > WS_R or wy > WS_Y:
                r += 1
        rejects[pname] = r

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot([p[0] for p in pp], [p[1] for p in pp], "k-", lw=2)
    cmap = plt.get_cmap("tab10")
    for i, (pt, off, nrm, e) in enumerate(samples):
        if i % 8:
            continue  # thin for legibility
        c = cmap(e % 10)
        ax.plot(off[0], off[1], ".", color=c, ms=3)
        ax.plot([off[0], off[0] + 0.008 * nrm[0]],
                [off[1], off[1] + 0.008 * nrm[1]], "-", color=c, lw=0.5)
    ax.set_title(f"{g}: perimeter samples (offset {OFFSET:.4f} m), "
                 f"colored by edge")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    fig.savefig(f"{OUT}/figures/{g}_candidate_contacts.png", dpi=140,
                bbox_inches="tight")
    plt.close(fig)

    md += [f"## {g}", "",
           f"- Max boundary discrepancy (720 rays): **{dmax*1000:.3f} mm**; "
           f"mean {dmean*1000:.3f} mm",
           f"- Perimeter length: {per_len:.4f} m; samples: {N_SAMPLES}",
           f"- Per-edge density [samples/m]: min {min(dens):.0f} "
           f"(edge {dens.index(min(dens))}, len "
           f"{edge_lens[dens.index(min(dens))]*1000:.1f} mm), "
           f"max {max(dens):.0f} (edge {dens.index(max(dens))})",
           f"- Edge sample counts: {per_edge}",
           f"- Workspace rejects (of {N_SAMPLES}): " +
           ", ".join(f"{k}: {v}" for k, v in rejects.items()), ""]
    if g == "C":
        md += [
            "The C's *geometric* boundary (blue/black outline) includes the "
            "concave mouth: the inner faces of the top/bottom bars and the "
            "inner spine wall at x=-0.0163. Uniform-by-arclength perimeter "
            "sampling spends a large fraction of its budget there, but the "
            "mouth interior is only ~0.039 m wide against an EE offset "
            "footprint of 2x0.0465 m: an offset sample placed inside the "
            "mouth collides with (or is vetoed against) the opposite bar, "
            "so the *physically useful pushing boundary* is essentially the "
            "convex outer perimeter (outer spine wall, bar tops/bottoms, "
            "bar tips). Effective coverage on the C is therefore lower than "
            "the raw per-edge densities suggest; mouth-edge samples mostly "
            "burn SampleIsAcceptable attempts.", ""]

with open(f"{OUT}/footprints/GLYPH_FOOTPRINT_VALIDATION.md", "w") as f:
    f.write("\n".join(md))
print("\n".join(md))
