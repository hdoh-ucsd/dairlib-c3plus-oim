#!/usr/bin/env python3
"""Faithful icra_sign planner<->simulator geometry consistency test (§12).

Equivalent of upstream tests/test_scenes.py::_assert_hull_matches for the C++
port: loads the SIM SDFs through Drake, the PLANNER polygons from the
obs_polys env string + scenario_params, and fails on drift > TOL.
Exit code 0 = pass.
"""
import os, sys, math, json
import numpy as np
import yaml
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
TOL = 1e-6
fails = []

def check(name, ok, detail=""):
    print(("PASS" if ok else "FAIL"), name, detail)
    if not ok:
        fails.append(name)

# planner polys
env = open(f"{WT}/results_icra_port/obstacles/obs_polys_env.txt").read().strip()
polys = {}
for entry in env.split(";"):
    parts = entry.split("|")
    polys[int(parts[0])] = [tuple(map(float, p.split(","))) for p in parts[1:]]
scen = yaml.safe_load(open(f"{WT}/examples/sampling_c3/anything_icra_c/parameters/scenario_params.yaml"))
check("planner_obstacle_count", len(scen["obstacles"]) == 8, f"{len(scen['obstacles'])}")
check("planner_poly_count", len(polys) == 7, f"{len(polys)}")
check("base_disc", scen["obstacles"][7] == [0.0, 0.0, 0.09])

# sim glyphs via Drake
builder = DiagramBuilder()
plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
parser = Parser(plant, sg)
inst = parser.AddModels(f"{WT}/examples/sampling_c3/urdf/scene_icra_sign.sdf")[0]
plant.Finalize()
diagram = builder.Build()
ctx = diagram.CreateDefaultContext()
pctx = plant.GetMyContextFromRoot(ctx)
insp = sg.model_inspector()
ROW = [("I", -0.55), ("R", -0.25), ("A", -0.10), ("2", 0.15),
       ("0", 0.30), ("2b", 0.45), ("6", 0.60)]
bodies = {}
for bi in plant.GetBodyIndices(inst):
    b = plant.get_body(bi)
    bodies[b.name()] = b
check("sim_glyph_link_count", len([n for n in bodies if n.startswith("glyph_")]) == 7,
      str(sorted(bodies)))
for oi, (g, y) in enumerate(ROW):
    b = bodies.get(f"glyph_{g}")
    if b is None:
        check(f"glyph_{g}_exists", False)
        continue
    X = plant.EvalBodyPoseInWorld(pctx, b)
    check(f"glyph_{g}_pose", abs(X.translation()[0] - 0.5) < TOL and
          abs(X.translation()[1] - y) < TOL and abs(X.translation()[2] - 0.0085) < 1e-4,
          str(np.round(X.translation(), 4)))
    gids = plant.GetCollisionGeometriesForBody(b)
    check(f"glyph_{g}_one_collision", len(gids) == 1)
    # mesh vertex extents vs planner poly (world frame)
    shape = insp.GetShape(gids[0])
    # read the OBJ back directly (same file Drake collides)
    obj_path = f"{WT}/examples/sampling_c3/urdf/icra_glyphs/glyph_{g.rstrip('b')}.obj"
    verts = [tuple(map(float, l.split()[1:3])) for l in open(obj_path) if l.startswith("v ")]
    sim_world = sorted(set((round(0.5 + vx, 6), round(y + vy, 6)) for vx, vy in verts))
    plan_world = sorted(set((round(px, 6), round(py, 6)) for px, py in polys[oi]))
    drift = max(min(math.hypot(a[0]-b2[0], a[1]-b2[1]) for b2 in plan_world) for a in sim_world)
    check(f"glyph_{g}_hull_vertices_match", drift < 1e-4, f"max drift {drift:.2e}")
    check(f"glyph_{g}_vertex_count", len(plan_world) == 10, str(len(plan_world)))

# C footprint vs C sim SDF boxes
CBOX = [(-0.0323, 0.0, 0.016, 0.0515), (0.0, 0.0355, 0.0483, 0.016),
        (0.0, -0.0355, 0.0483, 0.016)]
import re
sdf = open(f"{WT}/examples/sampling_c3/urdf/push_c_glyph.sdf").read()
sizes = re.findall(r"<pose>([-\d. ]+)</pose>\s*\n\s*<geometry><box><size>([\d. ]+)</size>", sdf)
got = set()
for pose, size in sizes:
    px, py = map(float, pose.split()[:2])
    sx, sy = map(float, size.split()[:2])
    got.add((round(px, 4), round(py, 4), round(sx/2, 4), round(sy/2, 4)))
want = set((round(a, 4), round(b2, 4), round(c, 4), round(d, 4)) for a, b2, c, d in CBOX)
check("c_footprint_matches_sim_boxes", want <= got, f"sim={sorted(got)}")

# goal-slot clearance sanity (recompute with planner polys)
def poly_sdf(px, py, verts):
    n = len(verts); best = 1e18; inside = True
    a2 = sum(verts[i][0]*verts[(i+1) % n][1] - verts[(i+1) % n][0]*verts[i][1] for i in range(n))
    vv = verts if a2 > 0 else verts[::-1]
    for i in range(n):
        ax, ay = vv[i]; bx, by = vv[(i+1) % n]
        ex, ey = bx-ax, by-ay
        if ex*(py-ay) - ey*(px-ax) < 0: inside = False
        L2 = ex*ex+ey*ey
        t = max(0.0, min(1.0, ((px-ax)*ex+(py-ay)*ey)/L2)) if L2 > 0 else 0
        best = min(best, math.hypot(px-(ax+t*ex), py-(ay+t*ey)))
    return -best if inside else best
cs, sn = math.cos(math.pi/2), math.sin(math.pi/2)
pts = []
for (cx, cy, hx, hy) in CBOX:
    for i in range(25):
        a = i/24
        for lx, ly in ((cx-hx+2*hx*a, cy-hy), (cx-hx+2*hx*a, cy+hy),
                       (cx-hx, cy-hy+2*hy*a), (cx+hx, cy-hy+2*hy*a)):
            pts.append((0.5 + cs*lx - sn*ly, -0.4 + sn*lx + cs*ly))
clear = min(poly_sdf(px, py, v) for v in polys.values() for px, py in pts)
check("goal_slot_clearance_positive", clear > 0.03, f"{clear:.4f} (upstream 0.051)")
check("goal_slot_clearance_matches_upstream", abs(clear - 0.0511) < 0.003, f"{clear:.4f}")

print("RESULT:", "PASS" if not fails else f"FAIL {fails}")
sys.exit(0 if not fails else 1)
