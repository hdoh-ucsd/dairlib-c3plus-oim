# Upstream sources for the dynamic ICRA glyph models (I, R, A)

Built 2026-09-07 by `scripts/build_icra_dynamic_glyphs.py`; validated and
rendered by `scripts/validate_icra_dynamic_glyphs.py`.

## Hull geometry (source of truth)

- `results/final_oim_c3plus_comparison/scene_fidelity/icra_glyph_obstacles.csv`
  — 10 local-frame vertices per glyph, provenance column recorded per row.
- Cross-checked against
  `/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX/oim/utils/scenes.py`:
  `_GLYPH_I` at **L282**, `_GLYPH_R` at **L290**, `_GLYPH_A` at **L298**
  (verified byte-for-byte against the CSV `x_local,y_local` columns).
  Those tuples are generated from the COMPILED MJX scene by
  `oim/models/xarm6_pusht_tabletop/glyph_hulls.py`, simplified to 10 vertices
  by least-area-loss vertex dropping; each polygon stays inside the true hull
  and covers >85% of it (contract enforced by `tests/test_scenes.py`).

## What upstream defines vs what is derived here

Upstream (`oim/models/xarm6_pusht_tabletop/icra_sign.xml`) treats I/R/A as
**STATIC obstacles**: `<geom class="tabletop_obstacle" type="mesh">` with no
mass and z-scale x3 (`scale="1 1 3"`, 0.075 m tall) for visibility/robustness
reasons documented in the XML (L54-L84, L177-L200). Upstream therefore
defines only their footprint outline; it defines NO mass, inertia, friction,
or dynamic thickness for these glyphs.

The dynamic models here DERIVE everything physical from the pushed C's
conventions (`examples/sampling_c3/urdf/push_c_glyph.sdf`):

| quantity | value | derivation |
|---|---|---|
| thickness | 0.025 m | C glyph box z size (upstream source glyphs are also 25 mm thick per icra_sign.xml comments; the x3 z-scale is obstacle-only) |
| density | 538.3290 kg/m^3 | C mass 0.1 kg / C footprint union area 0.0074304 m^2 / 0.025 m |
| mass | density x hull area x thickness | uniform-density prism |
| COM | polygon centroid, z = 0 | uniform density |
| Ixx/Iyy/Izz | polygon second moments about centroid x density x thickness (+ m t^2/12 for the in-plane axes) | solid prism, z centered |
| friction | mu_static = mu_dynamic = 0.3 | identical tags to push_c_glyph.sdf |
| witness spheres (controller SDFs only) | r = 0.001 at z = -0.0115, 3 spread hull corners inset 0.002 toward centroid | same style/values as push_c_glyph_controller.sdf |

Note the MJX pushed C weighs 2.0 kg (icra_sign.xml `block` body); the port's
C SDF is 0.1 kg. The 0.1 kg SDF is the declared source of truth for density
here, per the campaign spec.

## Fidelity note: the A's counter (internal hole)

The 10-vertex hull is CONVEX; the A's enclosed triangular counter and the
notch under its crossbar are NOT representable and the physical A here is its
convex hull (a wedge). This is faithful to upstream's own obstacle
representation: MJX collides mesh geoms as their convex hulls
(`maxhullvert=32`), so the A obstacle in the OIM scene is already convex and
`scenes.py`'s `_GLYPH_A` is that same convex hull. The same applies to R
(its bowl/leg concavities convexify to a rounded rectangle). Consequence for
the inventory's `stroke` column: for I it is a true stroke width (0.0298 m);
for R/A the rotating-calipers minimum width of the convex hull equals the
overall glyph dimension (0.1006 / 0.1030 m), not a typographic stroke.

## Resting pose on the AddXarm6ToPlant table

`ground.urdf` top surface is at z = 0 in the ground frame; `AddXarm6ToPlant`
(and `scripts/render_xarm6_3d.py`) welds ground at z = -0.029 under the xarm6
base, so the table top is world z = -0.029. Resting link pose for every
glyph (z-centered prisms and the C): **z = -0.0165** (= -0.029 + 0.025/2).
Validated signed distance to table within [-1e-6, 1e-3] for I, R, A, C.

## Files produced

- Meshes: `examples/sampling_c3/urdf/icra_glyphs/dynamic/{I,R,A}_prism.obj`
  (watertight convex prisms, vertex normals included)
- Sim SDFs: `examples/sampling_c3/urdf/push_{i,r,a}_glyph.sdf`
- Controller SDFs: `examples/sampling_c3/urdf/push_{i,r,a}_glyph_controller.sdf`
- Inventory: `results/icra_glyph_open_table/object_models/glyph_model_inventory.csv`
- Sim-vs-controller audit: `results/icra_glyph_open_table/object_models/glyph_sim_vs_controller.csv` (0 mismatches; witness spheres are the only, documented, delta)
- Friction manifest: `results/icra_glyph_open_table/physics/glyph_physics_manifest.csv`
- Stills: `results/icra_glyph_open_table/initial_scene/{I,R,A,C}.png` (top-down | 3/4)
