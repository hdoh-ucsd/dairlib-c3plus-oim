#!/usr/bin/env python3
"""Exact C-glyph union surface mesh for the kMeshNormal sampler.

10-vertex concave outline (spine + two bars + mouth), extruded to the C's
0.025 m thickness. Every side wall is a true exterior C face (no internal
seams), so face-normal sampling never lands inside the object.
"""
import os
WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
OUTD = f"{WT}/examples/sampling_c3/urdf/c_glyph_base"
os.makedirs(OUTD, exist_ok=True)
ZH = 0.0125
V = [(-0.0483, -0.0515), (-0.0163, -0.0515), (0.0483, -0.0515),
     (0.0483, -0.0195), (-0.0163, -0.0195), (-0.0163, 0.0195),
     (0.0483, 0.0195), (0.0483, 0.0515), (-0.0163, 0.0515),
     (-0.0483, 0.0515)]
n = len(V)
tris = []
# bottom (z-) faces, wound so normal points -z (order reversed vs CCW)
FACE = [(1, 2, 9), (1, 9, 10), (2, 3, 4), (2, 4, 5), (6, 7, 8), (6, 8, 9)]
for (a, b, c) in FACE:
    tris.append((c, b, a))              # bottom layer indices 1..n
    tris.append((a + n, b + n, c + n))  # top layer indices n+1..2n
for i in range(n):
    a, b = i + 1, (i + 1) % n + 1
    tris.append((a, b, b + n))
    tris.append((a, b + n, a + n))
with open(f"{OUTD}/c_glyph_base.obj", "w") as f:
    f.write("# exact C-glyph union prism (icra_sign pushed object)\n")
    for z in (-ZH, ZH):
        for (x, y) in V:
            f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
    for (a, b, c) in tris:
        f.write(f"f {a} {b} {c}\n")
print("wrote", f"{OUTD}/c_glyph_base.obj", len(tris), "tris")
