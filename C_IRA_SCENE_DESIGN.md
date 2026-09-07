# C-object / I-R-A-obstacle scene — design (2026-09-07)

**Concept:** the push-anything letter **C** is the manipulated object; the letters **I, R, A**
are static obstacles on the OIM-fidelity open table. Demo: `open_table_c_ira_obstacles_xarm6`.
Task: push C from (0.30, +0.40, yaw 0) to (0.50, −0.40, +π/2) — the letters form a slalom
across the corridor.

## How each element is designed and loaded

### The manipulated C (dynamic)
- **Sim model** `urdf/C_shape_texture/C_shape_texture.sdf`: one free body, mass 0.05 kg with
  mesh-derived inertia; geometry = **10 VHACD convex pieces** (`C_shape_texture_convex_*.obj`)
  each carrying both `<visual>` (purple diffuse) and `<collision>` with compliant-hydroelastic
  contact (modulus 3e7, dissipation 10, μ 0.3 — overridden by `--matched_mu` to the OIM pair
  values 0.3/0.5).
- **Controller model** `C_shape_texture_controller.sdf`: the anything-lineage **universal
  object model** — 1.0 kg, diag(0.003, 0.003, 0.006) inertia, the same convex pieces, plus
  **3 ground-witness spheres** for the LCS ground contact. The sim/controller mass mismatch is
  the lineage's deliberate robustness convention.
- Loaded by `franka_sim` from `sim_params.yaml` (spawn z = −0.029 − mesh min-z = −0.0060) and
  by the planner via `base_names: [C_shape_texture]`, which also feeds the **kMeshNormal
  sampler (strategy 7)**: EE approach samples are generated from the C's actual mesh normals,
  standoff = mesh surface + EE radius (now the 5.55 mm stick tip, read from the plant).

### The I, R, A obstacles (static)
One generated SDF, `urdf/scene_ira_letter_obstacles.sdf` (generator committed alongside):
- Reuses the letters' **own VHACD convex pieces** — identical geometry to their dynamic
  variants — but in a single `<static>true</static>` model, one link per letter, posed at each
  letter's measured resting height on the table (I −0.0078, R −0.0102, A −0.0056 in the task
  frame whose table top is −0.029). Rendered amber to distinguish them from the purple C.
- Poses (x, y, yaw): **I (0.30, 0.15, 90°)** — lies across the corridor just below the spawn;
  **R (0.55, 0.10, 0°)** — guards the right flank; **A (0.38, −0.18, 34°)** — sits between
  the corridor and the goal. I was originally at (0.20, 0.05) but that is radius 0.206 from
  the robot base — inside the planner's 0.25 minimum-radius ring — and EE paths around it
  tripped the workspace assert; it was moved to (0.30, 0.15), r = 0.335.
- The sim welds the whole model via `scenario_params.obstacle_model` (loaded whenever set);
  contact pairs pick up matched-μ (object–obstacle 0.5).

### What the planner knows about the obstacles
The C3+ planner does not consume the obstacle meshes. It sees three **planar discs** from
`scenario_params.obstacles`: centers at the letter poses, radius 0.085 m = each letter's
max half-extent (I 0.162×0.062, R 0.164×0.110, A 0.136×0.162 → half-diagonals ≈ 0.081–0.082)
with `obstacle_cost_weight: 5000, decay 0.04` — the same disc-cost mechanism as the matched
single-obstacle scene. So collision avoidance is conservative-circular while the physics is
exact-mesh.

### Robot / scene (unchanged frozen stack)
xArm6 5-joint velocity executor, prelift release, OIM white table (0.80×1.523, long axis y),
flush stick EE (r 5.55 mm, tip 179.4 mm), glyph conformance fixes carried over
(`ee_z_close: false`, `z_height: −0.012`, z floor −0.024). One code change: the planner demo
whitelist gained the `open_table_c_ira` prefix (anything branch).

## Files
- `examples/sampling_c3/open_table_c_ira_obstacles_xarm6/` (demo, cloned from the C glyph task)
- `examples/sampling_c3/urdf/scene_ira_letter_obstacles.sdf` (+ generator script)
- Whitelist: `examples/sampling_c3/franka_sampling_c3_controller.cc` (two dispatch sites)

## Load verification
30 s smoke: sim logs matched-μ for all 44 letter geoms (C's 10 + obstacles' 30 + witness/EE),
objects rest at spawn without penetration, planner cycles C3/repositioning cleanly, zero
aborts after the I reposition. Video: `c_ira_scene.mp4` (green ghost = C's goal pose).
