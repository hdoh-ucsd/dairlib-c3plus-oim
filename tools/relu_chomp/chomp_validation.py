#!/usr/bin/env python3
"""CHECKPOINT F — independent validation of the CHOMP-style obstacle metric.

Three legs, none of which touch any controller:
 1. Analytic unit checks of the piecewise potential c(d) (values, continuity,
    C1 smoothness at d=0).
 2. Closed-form functional checks THROUGH THE REAL eval_run CODE PATH
    (monkeypatched scene/trace providers): constant-clearance transit,
    linear approach ramp, penetration ramp.
 3. Physics cross-check: per-state min footprint SDF (the d used by CHOMP)
    vs the offline Drake SignedDistancePairs ground truth recorded by the
    single_obstacle penetration forensics (d_phys), joined on trace time.

Writes chomp/chomp_validation.md and exits nonzero on any failure.
"""
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C
import chomp_eval as CE

EPS = C.CHOMP_EPS
FORENSICS = ("/root/push_anything_ADMM/results/"
             "single_obstacle_penetration_forensics/geometry/"
             "physical_signed_distance.csv")
OUTMD = os.path.join(C.ROOT, "chomp", "chomp_validation.md")
results = []


def check(name, ok, detail):
    results.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


# ---------------------------------------------------------------- leg 1
d = np.array([-0.01, -0.005, 0.0, 0.005, 0.01, 0.02])
got = CE.chomp_cost(d)
want = np.array([0.01 + EPS / 2, 0.005 + EPS / 2, EPS / 2,
                 (0.005 - EPS) ** 2 / (2 * EPS), 0.0, 0.0])
check("c(d) values", np.allclose(got, want, atol=1e-12),
      f"max|err|={np.max(np.abs(got - want)):.2e}")
h = 1e-7
c0m = CE.chomp_cost(np.array([-h]))[0]
c0p = CE.chomp_cost(np.array([h]))[0]
check("continuity at 0", abs(c0m - c0p) < 1e-6, f"gap={abs(c0m-c0p):.2e}")
dm = (CE.chomp_cost(np.array([0.0]))[0] - c0m) / h
dp = (c0p - CE.chomp_cost(np.array([0.0]))[0]) / h
check("C1 at 0 (slope -1 both sides)",
      abs(dm + 1) < 1e-3 and abs(dp + 1) < 1e-3,
      f"left={dm:.4f} right={dp:.4f}")
ce = CE.chomp_cost(np.array([EPS - 1e-9, EPS + 1e-9]))
check("continuity at eps", np.all(ce < 1e-12), f"c(eps±)={ce}")

# ------------------------------------------------- leg 2: real code path
class FakeC:
    pass


def run_eval_with(points_xy, scene):
    """Drive CE.eval_run with a synthetic trace + scene via monkeypatch."""
    saved = (CE.C.load_scene, CE.C.read_state_trace, CE.C.sample_boundary)
    try:
        CE.C.load_scene = lambda s: scene
        CE.C.read_state_trace = lambda rd: dict(
            t=np.arange(len(points_xy), dtype=float),
            x=np.asarray([p[0] for p in points_xy]),
            y=np.asarray([p[1] for p in points_xy]),
            yaw=np.zeros(len(points_xy)))
        CE.C.sample_boundary = lambda fp: np.array([[0.0, 0.0]])
        return CE.eval_run("synthetic", "/nonexistent")
    finally:
        CE.C.load_scene, CE.C.read_state_trace, CE.C.sample_boundary = saved


# 2a: single body point, straight transit length L at constant clearance d0
# past a disc obstacle -> M = c(d0) * L exactly (trapezoid of a constant).
d0, L, r = 0.004, 0.2, 0.05
disc_scene = dict(footprint=None, discs=[(0.1, -(r + d0), r)], polys=[])
# path along y=0 from x=0.1-L/2 to 0.1+L/2 (min distance to disc center is
# NOT constant along x -- so use a wall poly instead for exactness)
wall = [(-1.0, -1.0), (1.0, -1.0), (1.0, 0.0), (-1.0, 0.0)]
wall_scene = dict(footprint=None, discs=[], polys=[wall])
N = 2001
pts = [(-L / 2 + L * i / (N - 1), d0) for i in range(N)]
out = run_eval_with(pts, wall_scene)
want = CE.chomp_cost(np.array([d0]))[0] * L
check("constant-clearance transit (wall poly)",
      math.isclose(out["M_CHOMP"], want, rel_tol=1e-9),
      f"M={out['M_CHOMP']:.6e} closed-form={want:.6e}")
check("normalization M/L", math.isclose(out["M_CHOMP_norm"], want / L,
      rel_tol=1e-9), f"norm={out['M_CHOMP_norm']:.6e}")

# 2b: vertical approach ramp y: eps -> 0 (arc length eps): integral
# of (d-eps)^2/(2 eps) with d=eps-s  ==> eps^2/6.
N = 4001
pts = [(0.0, EPS * (1 - i / (N - 1))) for i in range(N)]
out = run_eval_with(pts, wall_scene)
want = EPS ** 2 / 6
check("approach ramp eps->0 = eps^2/6",
      math.isclose(out["M_CHOMP"], want, rel_tol=1e-6),
      f"M={out['M_CHOMP']:.8e} exact={want:.8e}")

# 2c: penetration ramp y: 0 -> -eps: integral of (-d+eps/2) with d=-s
# ==> eps^2/2 + eps^2/2 = eps^2.
pts = [(0.0, -EPS * i / (N - 1)) for i in range(N)]
out = run_eval_with(pts, wall_scene)
want = EPS ** 2
check("penetration ramp 0->-eps = eps^2",
      math.isclose(out["M_CHOMP"], want, rel_tol=1e-6),
      f"M={out['M_CHOMP']:.8e} exact={want:.8e}")

# 2d: disc-branch sanity: point orbiting a disc at constant clearance d0.
M_arc = 0.03
nseg = 4000
ang = np.linspace(0, M_arc / (r + d0), nseg)
pts = [(0.1 + (r + d0) * math.cos(a), -(r + d0) + (r + d0) * math.sin(a))
       for a in ang]
out = run_eval_with(pts, dict(footprint=None,
                              discs=[(0.1, -(r + d0), r)], polys=[]))
want = CE.chomp_cost(np.array([d0]))[0] * M_arc
check("constant-clearance arc (disc)",
      math.isclose(out["M_CHOMP"], want, rel_tol=1e-5),
      f"M={out['M_CHOMP']:.6e} closed-form={want:.6e}")

# ------------------------------------------------ leg 3: physics cross-check
run_dir = os.path.join(C.ROOT, "baseline", "single_obstacle", "s01g01")
sc = C.load_scene("single_obstacle")
tr = C.read_state_trace(run_dir)
bp = C.sample_boundary(sc["footprint"])
# obstacle BOX only (forensics d_phys is the T<->box pair): keep geometry
# whose reference x is ~0.35 (the env box), drop the robot base disc etc.
box_polys = [p for p in sc["polys"]
             if 0.2 < np.mean([q[0] for q in p]) < 0.5]
box_discs = [dd for dd in sc["discs"] if 0.2 < dd[0] < 0.5]
check("scene has the env box geometry", len(box_polys) + len(box_discs) >= 1,
      f"polys={len(box_polys)} discs={len(box_discs)} "
      f"(all: {len(sc['polys'])}/{len(sc['discs'])})")
cos, sin = np.cos(tr["yaw"]), np.sin(tr["yaw"])
xw = tr["x"][:, None] + bp[:, 0][None, :] * cos[:, None] - \
    bp[:, 1][None, :] * sin[:, None]
yw = tr["y"][:, None] + bp[:, 0][None, :] * sin[:, None] + \
    bp[:, 1][None, :] * cos[:, None]
dmin = np.full(len(tr["t"]), np.inf)
for ox, oy, rr in box_discs:
    np.minimum(dmin, np.min(np.hypot(xw - ox, yw - oy) - rr, axis=1),
               out=dmin)
for p in box_polys:
    pd = np.fromiter((C.poly_signed_dist(px, py, p)
                      for px, py in zip(xw.ravel(), yw.ravel())),
                     dtype=float, count=xw.size).reshape(xw.shape)
    np.minimum(dmin, np.min(pd, axis=1), out=dmin)
phys = {}
with open(FORENSICS) as f:
    for row in csv.DictReader(f):
        phys[round(float(row["t"]), 4)] = float(row["d_phys"])
tj, dj, pj = [], [], []
for i, t in enumerate(tr["t"]):
    k = round(float(t), 4)
    if k in phys and abs(tr["roll"][i]) < 0.1 and abs(tr["pitch"][i]) < 0.1:
        tj.append(k)
        dj.append(dmin[i])
        pj.append(phys[k])
dj, pj = np.asarray(dj), np.asarray(pj)
err = np.abs(dj - pj)
check("joined rows vs forensics", len(dj) > 1500, f"n={len(dj)}")
check("footprint SDF == offline Drake d_phys (<2 mm max)",
      float(np.max(err)) < 0.002,
      f"median={np.median(err)*1e6:.1f} um  max={np.max(err)*1e3:.3f} mm  "
      f"min d: sdf={dj.min()*1e3:.2f} mm phys={pj.min()*1e3:.2f} mm")

# ---------------------------------------------------------------- report
ok_all = all(ok for _, ok, _ in results)
with open(OUTMD, "w") as f:
    f.write("# CHECKPOINT F — CHOMP metric independent validation\n"
            "2026-09-09 · `tools/relu_chomp/chomp_validation.py` · "
            "evaluation-only, no controller code touched.\n\n")
    f.write("| check | result | detail |\n|---|---|---|\n")
    for name, ok, detail in results:
        f.write(f"| {name} | {'PASS' if ok else 'FAIL'} | {detail} |\n")
    f.write(f"\n**Overall: {'PASS' if ok_all else 'FAIL'}**\n\n"
            "Legs: (1) analytic potential checks; (2) closed-form functional "
            "identities driven through the real `eval_run` code path via "
            "monkeypatched providers (wall poly, disc, approach/penetration "
            "ramps); (3) the CHOMP distance substrate (footprint boundary "
            "SDF) reproduces the offline Drake `SignedDistancePairs` ground "
            "truth recorded by the single_obstacle penetration forensics on "
            "the s01g01 trace, box-pair geometry only, tilted rows "
            "excluded.\n\nIndependence from the ranking costs: different "
            "functional form (piecewise CHOMP potential vs exp/ReLU), "
            "different aggregation (arc-length trapezoid over a dense fixed "
            "body-point set vs per-knot sums), computed offline from the "
            "EXECUTED trajectory, never fed to any controller.\n")
print(f"\nwrote {OUTMD}\nOVERALL: {'PASS' if ok_all else 'FAIL'}")
sys.exit(0 if ok_all else 1)
