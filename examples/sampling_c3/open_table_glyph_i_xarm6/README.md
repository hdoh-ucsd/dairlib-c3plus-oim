# open_table_glyph_i_xarm6

Open-table glyph I push task on the FROZEN xArm6 baseline. Copy of
push_t_bt010_open_table_xarm6 with object-specific parameters only (no
controller modification, no obstacles, no route logic). The manipulated
object is the push-anything lineage letter I_shape_texture (fig8-campaign
wiring, per the anything_I demo pattern) — NOT the derived hull prism.

- Object: examples/sampling_c3/urdf/I_shape_texture/I_shape_texture.sdf (sim) /
  I_shape_texture_controller.sdf (planner; VHACD convex pieces + 3 ground-witness
  spheres). object_body_name: body; base_names: [I_shape_texture].
- Spawn: (0.30, 0.40), yaw 0, quaternion (1,0,0,0); resting center
  z = -0.007789 (table top -0.029 minus collision-mesh min z; verified
  empirically — the letter neither falls nor penetrates at spawn).
- EE start: q_init_franka base joint rotated -0.7 rad vs the T demo
  (tip starts near (0.508, 0.096) instead of (0.327, 0.401)) so the
  initial descent does not plow through the object spawned at (0.30, 0.40).
- Goal: fixed target (0.50, -0.40, -0.007789), yaw +pi/2 =
  quaternion [cos(pi/4), 0, 0, sin(pi/4)] (w-first).
- Sampling: strategy 7 (anything-lineage kMeshNormal / multi-object mesh
  sampler; sampling_params.yaml copied verbatim from anything_I) —
  contacts are driven by the controller-SDF mesh geometry.
- Robot side kept from the xarm6 T demo: 5-dim q_init_franka,
  osc_params_xarm6.yaml, robot_radius_limits [0.25, 0.70], xarm6
  workspace limits.

## Env contract

NO environment variables are required:
- NO SAMPLING_C3_OBJECT_FOOTPRINT — the mesh sampler drives contacts; the
  default T footprint table is inert on an obstacle-free open table.
- NO obstacle env vars (no SAMPLING_C3_OBSTACLE_MODE, no route mode).

## Run (sim trio, xArm6)

    bazel-bin/examples/sampling_c3/franka_sim --demo_name=open_table_glyph_i_xarm6 --robot_model=xarm6 --matched_mu=true
    bazel-bin/examples/sampling_c3/franka_osc_controller --demo_name=open_table_glyph_i_xarm6 --robot_model=xarm6 --xarm6_five_joint=true --prelift_release=true
    bazel-bin/examples/sampling_c3/franka_sampling_c3_controller --demo_name=open_table_glyph_i_xarm6 --robot_model=xarm6
