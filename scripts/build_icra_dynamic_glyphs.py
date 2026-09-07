#!/usr/bin/env python3
"""Build faithful dynamic object models for ICRA glyphs I, R, A.

Sources of truth:
- Hull vertices: results/final_oim_c3plus_comparison/scene_fidelity/icra_glyph_obstacles.csv
  (10 verts/glyph, LOCAL coords, provenance scenes.py L282/L290/L298).
- Physical convention: examples/sampling_c3/urdf/push_c_glyph.sdf (mass 0.1 kg,
  thickness 0.025 m, mu 0.3). Density derived = C mass / C footprint union area
  / thickness. Upstream (icra_sign.xml) treats I/R/A as STATIC mesh obstacles
  (z-scaled x3, no mass), so dynamics here are DERIVED: uniform density x hull
  area x thickness; planar inertia from polygon second moments; COM = centroid.
"""
import csv
import itertools
import os

import numpy as np
import trimesh

REPO = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant"
CSV = os.path.join(REPO, "results/final_oim_c3plus_comparison/scene_fidelity/icra_glyph_obstacles.csv")
OUT_MESH = os.path.join(REPO, "examples/sampling_c3/urdf/icra_glyphs/dynamic")
OUT_SDF = os.path.join(REPO, "examples/sampling_c3/urdf")
RESULTS = os.path.join(REPO, "results/icra_glyph_open_table")

THICKNESS = 0.025           # C glyph z size (push_c_glyph.sdf boxes)
MU = 0.3                    # C glyph friction
C_MASS = 0.1                # push_c_glyph.sdf
WITNESS_R = 0.001
WITNESS_Z = -THICKNESS / 2 + WITNESS_R  # -0.0115, matches C controller
WITNESS_INSET = 0.002       # C witness spheres sit 0.002 inside extremes

# ---------------------------------------------------------------- C footprint
def c_union_area():
    # spine 0.032 x 0.103 + two bars 0.0966 x 0.032, each bar overlapping the
    # spine over 0.032 x 0.032 (spine x [-0.0483,-0.0163] within bar x range,
    # bar y band [0.0195,0.0515] within spine y range).
    return 0.032 * 0.103 + 2 * 0.0966 * 0.032 - 2 * 0.032 * 0.032

DENSITY = C_MASS / (c_union_area() * THICKNESS)

# ---------------------------------------------------------------- hull loading
def load_hulls():
    hulls = {}
    with open(CSV) as f:
        for row in csv.DictReader(f):
            g = row["glyph"]
            hulls.setdefault(g, []).append(
                (int(row["vertex_index"]), float(row["x_local"]), float(row["y_local"]), row["source"]))
    out = {}
    for g in ("I", "R", "A"):
        vs = sorted(hulls[g])
        out[g] = (np.array([(x, y) for _, x, y, _ in vs]), vs[0][3])
    return out

def polygon_props(pts):
    """Area, centroid, second moments about centroid (CCW polygon)."""
    x, y = pts[:, 0], pts[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y1 - x1 * y
    A = 0.5 * cross.sum()
    cx = ((x + x1) * cross).sum() / (6 * A)
    cy = ((y + y1) * cross).sum() / (6 * A)
    Ix = ((y**2 + y * y1 + y1**2) * cross).sum() / 12  # about origin, int y^2 dA
    Iy = ((x**2 + x * x1 + x1**2) * cross).sum() / 12
    Ix_c = Ix - A * cy**2
    Iy_c = Iy - A * cx**2
    return A, cx, cy, Ix_c, Iy_c

def min_width(pts):
    """Min feature width of convex polygon (directional extent sweep)."""
    best = np.inf
    for ang in np.deg2rad(np.arange(0, 180, 0.25)):
        c, s = np.cos(ang), np.sin(ang)
        R = np.array([[c, -s], [s, c]])
        r = pts @ R.T
        ext = r.max(axis=0) - r.min(axis=0)
        best = min(best, ext[0], ext[1])
    return best


def make_prism(pts, thickness):
    """Watertight convex prism from CCW polygon, z centered on 0."""
    n = len(pts)
    bot = np.column_stack([pts, np.full(n, -thickness / 2)])
    top = np.column_stack([pts, np.full(n, thickness / 2)])
    verts = np.vstack([bot, top])
    faces = []
    for i in range(1, n - 1):  # bottom fan (facing -z: reverse winding)
        faces.append([0, i + 1, i])
    for i in range(1, n - 1):  # top fan (facing +z)
        faces.append([n, n + i, n + i + 1])
    for i in range(n):         # sides
        j = (i + 1) % n
        faces.append([i, j, n + j])
        faces.append([i, n + j, n + i])
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)

def witness_corners(pts, cx, cy):
    """3 hull vertices maximizing triangle area, inset toward centroid."""
    best, tri = -1, None
    for i, j, k in itertools.combinations(range(len(pts)), 3):
        a, b, c = pts[i], pts[j], pts[k]
        ar = abs(np.cross(b - a, c - a)) / 2
        if ar > best:
            best, tri = ar, (pts[i], pts[j], pts[k])
    out = []
    for p in tri:
        d = np.array([cx, cy]) - p
        d = d / np.linalg.norm(d)
        out.append(p + WITNESS_INSET * d)
    return out

# ---------------------------------------------------------------- SDF writing
def sdf_text(g, mass, cx, cy, I, mesh_uri, controller, witnesses):
    name = f"push_{g.lower()}_glyph" + ("_controller" if controller else "")
    Ixx, Iyy, Izz = I
    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append(f'<!-- Dynamic ICRA glyph {g}: 10-vertex convex hull (OIM scenes.py) extruded')
    lines.append(f'     to the pushed C\'s thickness {THICKNESS} m at its uniform density')
    lines.append(f'     {DENSITY:.4f} kg/m^3. Upstream treats {g} as a STATIC obstacle; mass,')
    lines.append('     COM and inertia here are DERIVED (density x hull area x thickness,')
    lines.append('     polygon second moments, centroid COM).')
    if controller:
        lines.append('     Controller model: + 3 ground-witness spheres (anything-branch wiring).')
    lines.append('-->')
    lines.append('<sdf version="1.7">')
    lines.append(f'  <model name="{name}">')
    lines.append('    <link name="c_glyph_base">')
    lines.append(f'      <inertial><pose>{cx:.6f} {cy:.6f} 0 0 0 0</pose><mass>{mass:.9f}</mass>')
    lines.append(f'        <inertia><ixx>{Ixx:.9e}</ixx><iyy>{Iyy:.9e}</iyy><izz>{Izz:.9e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
    lines.append('      <visual name="hull_visual"><pose>0 0 0 0 0 0</pose>')
    lines.append(f'        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>')
    lines.append('      </visual>')
    lines.append('      <collision name="hull_collision"><pose>0 0 0 0 0 0</pose>')
    lines.append(f'        <geometry><mesh><uri>{mesh_uri}</uri><drake:declare_convex/></mesh></geometry>')
    lines.append(f'        <surface><friction><ode><mu>{MU}</mu><mu2>{MU}</mu2></ode></friction></surface>')
    lines.append(f'        <drake:proximity_properties><drake:mu_static>{MU}</drake:mu_static><drake:mu_dynamic>{MU}</drake:mu_dynamic></drake:proximity_properties>')
    lines.append('      </collision>')
    if controller:
        for k, (wx, wy) in enumerate(witnesses):
            lines.append(f'      <collision name="witness_{k}"><pose>{wx:.6f} {wy:.6f} {WITNESS_Z} 0 0 0</pose>')
            lines.append(f'        <geometry><sphere><radius>{WITNESS_R}</radius></sphere></geometry></collision>')
    lines.append('    </link>')
    lines.append('  </model>')
    lines.append('</sdf>')
    return "\n".join(lines) + "\n"

def main():
    os.makedirs(OUT_MESH, exist_ok=True)
    for d in ("object_models", "physics", "provenance", "initial_scene"):
        os.makedirs(os.path.join(RESULTS, d), exist_ok=True)

    hulls = load_hulls()
    print(f"C union footprint area = {c_union_area():.7f} m^2, density = {DENSITY:.4f} kg/m^3")

    inv_rows = []
    props = {}
    for g, (pts, src) in hulls.items():
        A, cx, cy, Ix_c, Iy_c = polygon_props(pts)
        assert A > 0, f"{g}: polygon not CCW"
        mass = DENSITY * A * THICKNESS
        # Solid prism inertia about COM (z centered):
        Ixx = DENSITY * THICKNESS * Ix_c + mass * THICKNESS**2 / 12
        Iyy = DENSITY * THICKNESS * Iy_c + mass * THICKNESS**2 / 12
        Izz = DENSITY * THICKNESS * (Ix_c + Iy_c)
        stroke = min_width(pts)
        w = pts[:, 0].max() - pts[:, 0].min()
        h = pts[:, 1].max() - pts[:, 1].min()

        mesh = make_prism(pts, THICKNESS)
        assert mesh.is_watertight and mesh.is_convex, g
        obj_path = os.path.join(OUT_MESH, f"{g}_prism.obj")
        mesh.export(obj_path, include_normals=True)

        wits = witness_corners(pts, cx, cy)
        uri = f"icra_glyphs/dynamic/{g}_prism.obj"
        for ctrl in (False, True):
            fn = os.path.join(OUT_SDF, f"push_{g.lower()}_glyph{'_controller' if ctrl else ''}.sdf")
            with open(fn, "w") as f:
                f.write(sdf_text(g, mass, cx, cy, (Ixx, Iyy, Izz), uri, ctrl, wits))
            print("wrote", fn)
        props[g] = dict(mass=mass, com=(cx, cy, 0.0), I=(Ixx, Iyy, Izz), area=A)
        inv_rows.append([g, f"{mass:.6f}", f"{cx:.6f}", f"{cy:.6f}", "0.0",
                         f"{Ixx:.6e}", f"{Iyy:.6e}", f"{Izz:.6e}",
                         f"{w:.4f}", f"{h:.4f}", f"{stroke:.4f}", 1, MU, MU,
                         "n/a (limit-surface model not used; point-contact LCS)",
                         f"examples/sampling_c3/urdf/push_{g.lower()}_glyph.sdf", src])
        print(f"{g}: area={A:.6f} mass={mass:.5f} com=({cx:.5f},{cy:.5f}) "
              f"I=({Ixx:.3e},{Iyy:.3e},{Izz:.3e}) stroke={stroke:.4f}")

    # C row from its SDF (authoritative values)
    inv_rows.append(["C", "0.100000", "-0.011240", "0.0", "0.0",
                     "1.237060e-04", "8.255100e-05", "1.958410e-04",
                     "0.0966", "0.1030", "0.0320", 3, MU, MU,
                     "n/a (limit-surface model not used; point-contact LCS)",
                     "examples/sampling_c3/urdf/push_c_glyph.sdf", "push_c_glyph.sdf L8-L9"])

    with open(os.path.join(RESULTS, "object_models/glyph_model_inventory.csv"), "w", newline="") as f:
        wtr = csv.writer(f)
        wtr.writerow(["glyph", "mass", "com_x", "com_y", "com_z", "Ixx", "Iyy", "Izz",
                      "width", "height", "stroke", "collision_parts", "table_mu",
                      "pusher_mu", "limit_surface_radius", "source_file", "source_line"])
        wtr.writerows(inv_rows)

    with open(os.path.join(RESULTS, "physics/glyph_physics_manifest.csv"), "w", newline="") as f:
        wtr = csv.writer(f)
        wtr.writerow(["glyph", "object_mu_in_sdf", "expected_pair_mu_with_ground",
                      "expected_pair_mu_with_ee", "semantics"])
        for g in ("I", "R", "A", "C"):
            wtr.writerow([g, MU, 0.3, 1.5,
                          "--matched_mu: pair mu set by sim from ground=0.3 / EE=1.5 tables; object SDF mu 0.3 matches C convention"])
    print("wrote inventory + physics manifest")
    return props

if __name__ == "__main__":
    main()
