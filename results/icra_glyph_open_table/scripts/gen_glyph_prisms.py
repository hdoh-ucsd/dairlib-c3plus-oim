#!/usr/bin/env python3
"""Extruded prism OBJs for I/R/A glyph hulls, for the kMeshNormal sampler.

Same mechanism as tools/c3plus_diagnostics/agent_d/gen_c_mesh.py (the C
task's exact union mesh): hull side walls are true exterior faces so
face-normal sampling never lands inside the object. Vertices are the
10-point local hulls from
results/final_oim_c3plus_comparison/scene_fidelity/icra_glyph_obstacles.csv.
Written to examples/sampling_c3/urdf/icra_glyphs/dynamic/<g>_prism.obj
(the path other agents reference); skips any file that already exists so a
parallel producer wins.
"""
import csv
import os

WT = os.path.abspath(os.path.dirname(__file__) + "/../../..")
CSV = f"{WT}/results/final_oim_c3plus_comparison/scene_fidelity/icra_glyph_obstacles.csv"
OUTD = f"{WT}/examples/sampling_c3/urdf/icra_glyphs/dynamic"
ZH = 0.0125  # match the C prism half-thickness

hulls = {}
with open(CSV) as f:
    for row in csv.DictReader(f):
        hulls.setdefault(row["glyph"], []).append(
            (float(row["x_local"]), float(row["y_local"])))

os.makedirs(OUTD, exist_ok=True)
for g in ("I", "R", "A"):
    out = f"{OUTD}/{g}_prism.obj"
    if os.path.exists(out):
        print("exists, skipping", out)
        continue
    V = hulls[g]
    n = len(V)
    tris = []
    # convex hull -> fan triangulation for top/bottom caps
    for i in range(1, n - 1):
        tris.append((i + 2, i + 1, 1))              # bottom, normal -z
        tris.append((1 + n, i + 1 + n, i + 2 + n))  # top, normal +z
    for i in range(n):  # side walls (hull is CCW -> outward normals)
        a, b = i + 1, (i + 1) % n + 1
        tris.append((a, b, b + n))
        tris.append((a, b + n, a + n))
    with open(out, "w") as f:
        f.write(f"# {g}-glyph hull prism (icra_glyph_obstacles.csv local hull)\n")
        for z in (-ZH, ZH):
            for (x, y) in V:
                f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        for t in tris:
            f.write("f %d %d %d\n" % t)
    print("wrote", out, len(tris), "tris")
