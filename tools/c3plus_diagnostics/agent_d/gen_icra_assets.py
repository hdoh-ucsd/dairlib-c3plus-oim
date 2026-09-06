#!/usr/bin/env python3
"""Generate the faithful icra_sign assets for the C++/Drake stack.

Reads the upstream OIM checkout (scenes.py hulls + icra_sign.xml C boxes),
emits:
  - examples/sampling_c3/urdf/icra_glyphs/glyph_*.obj  (extruded 10-vert hulls)
  - examples/sampling_c3/urdf/scene_icra_sign.sdf      (7 static glyph obstacles)
  - examples/sampling_c3/urdf/push_c_glyph.sdf         (sim model, 3 boxes)
  - examples/sampling_c3/urdf/push_c_glyph_controller.sdf (3 boxes + 3 witness spheres)
  - results_icra_port CSVs: layout, hulls, C model params, pose variants
  - the SAMPLING_C3_OBS_POLYS env string (printed)
"""
import ast, os, math, csv, json, re

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
OIM = "/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX"
OUT = f"{WT}/results_icra_port"
URDF = f"{WT}/examples/sampling_c3/urdf"
os.makedirs(f"{URDF}/icra_glyphs", exist_ok=True)

# ---- 1. Parse hulls from scenes.py with line provenance
src = open(f"{OIM}/oim/utils/scenes.py").read().splitlines()
hulls = {}
hull_lines = {}
name = None
buf = ""
for i, line in enumerate(src, 1):
    m = re.match(r"_GLYPH_(\w) = \(", line)
    if m:
        name = m.group(1)
        hull_lines[name] = i
        buf = ""
        continue
    if name is not None:
        buf += line
        if line.strip() == ")":
            hulls[name] = ast.literal_eval("(" + buf.rstrip()[:-1].strip() + ")")
            name = None
assert set(hulls) == {"I", "R", "A", "2", "0", "6"}, hulls.keys()

ROW = [("I", -0.55), ("R", -0.25), ("A", -0.10), ("2", 0.15),
       ("0", 0.30), ("2b", 0.45), ("6", 0.60)]
GLYPH_Z_HALF = 0.0375        # 0.0125 base half-thickness x3 z-scale (MJCF)
GROUND_TOP = -0.029          # dairlib world: ground top plane
GLYPH_ZC = GROUND_TOP + GLYPH_Z_HALF
BASE = (0.0, 0.0, 0.09)      # planner-only robot-base circle (scenes.py:333)

# ---- 2. Layout + hull CSVs
with open(f"{OUT}/scene/icra_sign_layout.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["glyph", "x", "y", "z", "roll", "pitch", "yaw", "scale_x",
                "scale_y", "scale_z", "static", "source_file", "source_line"])
    for g, y in ROW:
        key = g.rstrip("b")
        # hull coords are stored ALREADY in placed orientation (scenes.py _glyph
        # translates only), so the Drake pose carries no extra yaw.
        w.writerow([g, 0.5, y, round(GLYPH_ZC, 4), 0, 0,
                    "0 (hull pre-rotated; MJCF euler 0 0 90 baked into hull)",
                    1, 1, "3 (baked into z extrusion)", True,
                    "oim/utils/scenes.py + icra_sign.xml",
                    hull_lines[key]])
    w.writerow(["C (pushed)", 0.3, 0.4, round(GROUND_TOP + 0.0125, 4), 0, 0, 0,
                1, 1, 1, False, "icra_sign.xml body block / scenes.py object_start", 139])

with open(f"{OUT}/obstacles/planner_glyph_hulls.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["glyph", "vertex_index", "x_local", "y_local",
                "source_file", "source_line"])
    for g, y in ROW:
        key = g.rstrip("b")
        for vi, (vx, vy) in enumerate(hulls[key]):
            w.writerow([g, vi, vx, vy, "oim/utils/scenes.py", hull_lines[key]])

# ---- 3. Extruded-hull OBJs (convex; CCW bottom, CCW top)
def signed_area(pts):
    return 0.5 * sum(pts[i][0]*pts[(i+1) % len(pts)][1] -
                     pts[(i+1) % len(pts)][0]*pts[i][1] for i in range(len(pts)))

for key, pts in hulls.items():
    p = list(pts)
    if signed_area(p) < 0:
        p = p[::-1]
    n = len(p)
    fn = f"{URDF}/icra_glyphs/glyph_{key}.obj"
    with open(fn, "w") as f:
        f.write(f"# extruded 10-vertex convex hull of OIM glyph {key}\n"
                f"# source: oim/utils/scenes.py line {hull_lines[key]}\n")
        for z in (-GLYPH_Z_HALF, GLYPH_Z_HALF):
            for (x, y) in p:
                f.write(f"v {x:.4f} {y:.4f} {z:.4f}\n")
        # normals: bottom, top, then one per side edge
        f.write("vn 0 0 -1\nvn 0 0 1\n")
        import math as _m
        for i in range(n):
            ex = p[(i+1) % n][0] - p[i][0]; ey = p[(i+1) % n][1] - p[i][1]
            L = _m.hypot(ex, ey) or 1.0
            f.write(f"vn {ey/L:.6f} {-ex/L:.6f} 0\n")
        # bottom face (points down): reversed
        f.write("f " + " ".join(f"{i}//1" for i in range(n, 0, -1)) + "\n")
        # top face
        f.write("f " + " ".join(f"{i + n}//2" for i in range(1, n + 1)) + "\n")
        for i in range(n):
            a, b = i + 1, (i + 1) % n + 1
            f.write(f"f {a}//{i+3} {b}//{i+3} {b + n}//{i+3} {a + n}//{i+3}\n")

# ---- 4. scene_icra_sign.sdf
links = []
for g, y in ROW:
    key = g.rstrip("b")
    links.append(f"""    <link name="glyph_{g}"><pose>0.5 {y} {GLYPH_ZC:.4f} 0 0 0</pose>
      <visual name="glyph_{g}_v"><geometry><mesh><uri>icra_glyphs/glyph_{key}.obj</uri></mesh></geometry>
        <material><diffuse>0.85 0.2 0.2 1.0</diffuse></material></visual>
      <collision name="glyph_{g}_c"><geometry><mesh><uri>icra_glyphs/glyph_{key}.obj</uri>
        <drake:declare_convex/></mesh></geometry></collision>
    </link>""")
with open(f"{URDF}/scene_icra_sign.sdf", "w") as f:
    f.write(f"""<?xml version="1.0"?>
<!-- Faithful OIM icra_sign obstacle row: "ICRA 2026" minus the pushed C.
     Each glyph collides as the extruded 10-vertex convex hull of its COMPILED
     MJX mesh (oim/utils/scenes.py) — the same geometry MJX collides and the
     same polygons the planner holds, so planner == simulation by construction.
     The robot-base circle (0,0,r=0.09) is planner-only, as upstream. -->
<sdf version="1.7">
  <model name="scene_icra_sign">
    <static>true</static>
{os.linesep.join(links)}
  </model>
</sdf>
""")

# ---- 5. C SDFs (3 boxes; MJCF half-sizes -> SDF full sizes)
# icra_sign.xml: spine (0.0160,0.0515,0.0125)@(-0.0323,0,0) m=0.0348
#                top   (0.0483,0.0160,0.0125)@(0,+0.0355,0) m=0.0326
#                bot   (0.0483,0.0160,0.0125)@(0,-0.0355,0) m=0.0326
BOXES = [("spine", (-0.0323, 0.0, 0.0), (0.032, 0.103, 0.025), 0.0348),
         ("top_bar", (0.0, 0.0355, 0.0), (0.0966, 0.032, 0.025), 0.0326),
         ("bot_bar", (0.0, -0.0355, 0.0), (0.0966, 0.032, 0.025), 0.0326)]

def box_inertia(m, l):
    return (m/12*(l[1]**2 + l[2]**2), m/12*(l[0]**2 + l[2]**2),
            m/12*(l[0]**2 + l[1]**2))

def link_xml(with_witness):
    total_m = sum(b[3] for b in BOXES)
    comx = sum(b[3]*b[1][0] for b in BOXES)/total_m
    # composite inertia about COM (parallel axis)
    ixx = iyy = izz = 0.0
    for _, c, l, m in BOXES:
        bx, by, bz = box_inertia(m, l)
        dx, dy = c[0]-comx, c[1]
        ixx += bx + m*(dy*dy)
        iyy += by + m*(dx*dx)
        izz += bz + m*(dx*dx + dy*dy)
    geoms = ""
    FR = ("        <surface><friction><ode><mu>0.3</mu><mu2>0.3</mu2></ode></friction></surface>\n"
          "        <drake:proximity_properties><drake:mu_static>0.3</drake:mu_static>"
          "<drake:mu_dynamic>0.3</drake:mu_dynamic></drake:proximity_properties>\n")
    for nm, c, l, m in BOXES:
        for tag in ("visual", "collision"):
            fr = FR if tag == "collision" else ""
            geoms += f"""      <{tag} name="{nm}_{tag}"><pose>{c[0]} {c[1]} {c[2]} 0 0 0</pose>
        <geometry><box><size>{l[0]} {l[1]} {l[2]}</size></box></geometry>\n{fr}      </{tag}>\n"""
    if with_witness:
        # 3 ground-witness spheres at bottom outer corners (z = -0.0125+0.001)
        for nm, (wx, wy) in [("witness_spine", (-0.0483 + 0.002, 0.0)),
                             ("witness_top", (0.0483 - 0.002, 0.0515 - 0.002)),
                             ("witness_bot", (0.0483 - 0.002, -0.0515 + 0.002))]:
            geoms += f"""      <collision name="{nm}"><pose>{wx} {wy} -0.0115 0 0 0</pose>
        <geometry><sphere><radius>0.001</radius></sphere></geometry></collision>\n"""
    return total_m, comx, (ixx, iyy, izz), geoms

for fname, with_w in (("push_c_glyph.sdf", False),
                      ("push_c_glyph_controller.sdf", True)):
    m, comx, (ixx, iyy, izz), geoms = link_xml(with_w)
    with open(f"{URDF}/{fname}", "w") as f:
        f.write(f"""<?xml version="1.0"?>
<!-- Faithful OIM icra_sign pushed C: block capital-C from three boxes, exactly
     icra_sign.xml's c_spine/c_top_bar/c_bot_bar (half-sizes doubled), 0.1 kg.
     {'Controller model: + 3 ground-witness spheres (anything-branch wiring).' if with_w else 'Simulation model.'} -->
<sdf version="1.7">
  <model name="{fname[:-4]}">
    <link name="c_glyph_base">
      <inertial><pose>{comx:.6f} 0 0 0 0 0</pose><mass>{m}</mass>
        <inertia><ixx>{ixx:.9f}</ixx><iyy>{iyy:.9f}</iyy><izz>{izz:.9f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{geoms}    </link>
  </model>
</sdf>
""")

# ---- 6. C model CSVs
with open(f"{OUT}/object/c_glyph_model_parameters.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["parameter", "value", "upstream_source"])
    m, comx, (ixx, iyy, izz), _ = link_xml(False)
    w.writerow(["total_mass_kg", m, "scenes.py mass=0.1 / icra_sign.xml geom masses"])
    w.writerow(["com_x", round(comx, 6), "computed from box masses/centers"])
    w.writerow(["Ixx", round(ixx, 9), "computed (composite, about COM)"])
    w.writerow(["Iyy", round(iyy, 9), "computed"])
    w.writerow(["Izz", round(izz, 9), "computed"])
    for nm, c, l, bm in BOXES:
        w.writerow([f"box_{nm}", f"center={c} full_size={l} mass={bm}",
                    "icra_sign.xml c_" + nm.replace("_bar", "_bar")])
    w.writerow(["overall_extent", "0.0966 x 0.1030 x 0.025 m", "icra_sign.xml comment"])
    w.writerow(["stroke", "0.032 m", "half_stroke 0.016 (scenes.py footprint_kwargs)"])
    w.writerow(["rest_z_center_world", GROUND_TOP + 0.0125, "ground top -0.029 + half 0.0125"])

# ---- 7. Pose variants (verbatim from examples/poses/icra_sign.yaml)
import yaml as _yaml
pv = _yaml.safe_load(open(f"{OIM}/examples/poses/icra_sign.yaml"))
clear_re = dict(s={}, g={})
for kind, key in (("s", "starts"), ("g", "goals")):
    txt = open(f"{OIM}/examples/poses/icra_sign.yaml").read()
    for m2 in re.finditer(r'"(\d)": \[([^\]]+)\] # ([\d.]+) cm', txt):
        pass
with open(f"{OUT}/scene/icra_sign_pose_variants.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["variant", "start_x", "start_y", "start_yaw",
                "goal_x", "goal_y", "goal_yaw",
                "min_start_clearance_upstream", "min_goal_clearance_upstream"])
    clr = {"1": (0.100, 0.051), "2": (0.123, 0.056), "3": (0.085, 0.042),
           "4": (0.097, 0.049), "5": (0.075, 0.042)}
    for k in "12345":
        s, g = pv["starts"][k], pv["goals"][k]
        w.writerow([k] + s + g + list(clr[k]))

# ---- 8. env string for polygon obstacles (indices 0-6; index 7 = base disc)
polys = []
for oi, (g, y) in enumerate(ROW):
    key = g.rstrip("b")
    verts = "|".join(f"{0.5+vx:.4f},{y+vy:.4f}" for vx, vy in hulls[key])
    polys.append(f"{oi}|{verts}")
env = ";".join(polys)
with open(f"{OUT}/obstacles/obs_polys_env.txt", "w") as f:
    f.write(env + "\n")
# scenario disc fallbacks: circumscribed radius per glyph
discs = []
for g, y in ROW:
    key = g.rstrip("b")
    r = max(math.hypot(vx, vy) for vx, vy in hulls[key])
    discs.append([0.5, y, round(r, 4)])
discs.append(list(BASE))
print("discs:", discs)
print("obs_polys chars:", len(env))
print("assets written")
