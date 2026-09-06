# C-glyph planner footprint design

**Choice: exact union-of-three-box boundary sampling** — `CFootprint()` in
`systems/controllers/sampling_based_c3_controller.cc`, selected at runtime by
`SAMPLING_C3_OBJECT_FOOTPRINT=c_glyph` (scene configuration, not a controller
change; every other task keeps `TFootprint()` untouched).

The three rects are exactly `push_c_glyph.sdf`'s collision boxes = upstream
`icra_sign.xml`'s `c_spine`/`c_top_bar`/`c_bot_bar` = upstream
`c_shape_footprint(half_width=0.0483, half_height=0.0515, half_stroke=0.016)`:

| piece | center (x, y) | full size (x, y) |
|---|---|---|
| spine | (−0.0323, 0) | 0.032 × 0.103 |
| top bar | (0, +0.0355) | 0.0966 × 0.032 |
| bottom bar | (0, −0.0355) | 0.0966 × 0.032 |

Each rect is sampled at 11 points per edge pair, the same density and scheme
as the existing `TFootprint()`, so **obstacle SDF queries, closest-point
witness generation, contact Jacobians (r×n), nonpenetration checks, and route
feasibility all consume the C boundary with no code-path changes** — only the
point table differs. The **concavity (the C's mouth) is captured**: the mouth
edge points are the spine's inner face and the bars' inner faces, so a witness
inside the mouth resolves to the true nearest C surface, not a convex hull.

Fidelity: every sample lies on a simulation collision-box face → planner-vs-sim
boundary discrepancy **0.0 mm** (`c_glyph_footprint_validation.png`). Samples
in the spine∩bar overlap are interior to the union (max depth 15.9 mm); the
same property exists in `TFootprint()` and is conservative only (an interior
point can never report a larger clearance than the boundary).

Provenance: box dimensions from `icra_sign.xml` (half-sizes doubled), cross-
checked against `planar_pushing.py::c_shape_footprint` defaults; masses from
the same MJCF geoms (0.0348/0.0326/0.0326 = 0.1 kg).
