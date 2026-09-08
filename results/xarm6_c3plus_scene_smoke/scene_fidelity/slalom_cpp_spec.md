# matched_slalom_xarm6_t1 — implementation spec (spec only, dir NOT created)

Upstream authority: OIM @ d6d80a6, scene added by 80a6fde. Extracted numbers in
`slalom_upstream.yaml` (same directory). Coordinate frame is identical to the
existing matched scenes (arm base at origin, corridor x = 0.381, y +0.4 -> -0.4);
port table/platform top at z = -0.029 (oimframe convention, see
`examples/sampling_c3/urdf/scene_shelf_gap_oimframe.sdf` header).

## 1. Copy-from template

Copy `examples/sampling_c3/matched_shelf_gap_xarm6_t1/` -> `matched_slalom_xarm6_t1/`
wholesale. The shelf_gap and single_obstacle dirs differ ONLY in
`parameters/scenario_params.yaml` and the path lines of
`parameters/sampling_c3_controller_params.yaml`, so:

1. `sed -i s/matched_shelf_gap_xarm6_t1/matched_slalom_xarm6_t1/` across
   `parameters/sampling_c3_controller_params.yaml`, `BUILD.bazel`,
   `franka_sim_t.pmd`, `franka_hardware_t.pmd`, `README.md`.
2. Keep FROZEN xarm6 baseline bits untouched: 5-dim `q_init_franka`,
   `osc_params_xarm6.yaml`, `robot_radius_limits: [0.25, 0.70]`,
   `pwl_waypoint_height: 0.10`.
3. `goal_params.yaml` and `sim_params.yaml` need NO edits for trial 1:
   slalom's nominal start (0.381, +0.4, yaw 0) and goal (0.381, -0.4, yaw pi)
   are byte-identical to shelf_gap's
   (`q_init_object: [1, 0, 0, 0, 0.381, 0.4, 0.0008]`,
   `fixed_target_position: [0.381, -0.4, 0.0008]`,
   `fixed_target_orientation: [-3.67321e-06, 0, 0, 1]`).
4. Success thresholds: leave the port's `position_success_threshold: 0.02` /
   `orientation_success_threshold: 0.1` unless matching upstream's looser eval
   (upstream `goal_pos_tol: 0.05`, `goal_theta_tol: 0.1`,
   oim/configs/robots/xarm6.yaml:108-109) is desired for the benchmark.

## 2. New `parameters/scenario_params.yaml`

Planner obstacles are planar discs `[x, y, radius]` (struct
`SamplingC3ScenarioParams`, `examples/sampling_c3/parameter_headers/scenario_params.h`;
consumed by the ranking proximity cost and the env-gated LCS-contact mode in
`systems/controllers/sampling_based_c3_controller.cc`). Cover each fin
(half-extents in x listed, half-thickness 0.02 in y) with r = 0.045 discs
(r = hypot(0.04, 0.02) rounded up) spaced <= 0.07 in x so inter-disc coverage
never dips below the fin surface:

```yaml
scenario_name: slalom
obstacles:
# fin_1_near: box c=(0.05, 0.22) he=(0.10, 0.02)   x-span [-0.05, 0.15]
- [-0.02, 0.22, 0.045]
- [0.05, 0.22, 0.045]
- [0.12, 0.22, 0.045]
# fin_1_far: box c=(0.57, 0.22) he=(0.18, 0.02)    x-span [0.39, 0.75]
- [0.435, 0.22, 0.045]
- [0.505, 0.22, 0.045]
- [0.575, 0.22, 0.045]
- [0.645, 0.22, 0.045]
- [0.715, 0.22, 0.045]
# fin_2_near: box c=(0.245, 0.0) he=(0.095, 0.02)  x-span [0.15, 0.34]
- [0.185, 0.0, 0.045]
- [0.245, 0.0, 0.045]
- [0.305, 0.0, 0.045]
# fin_2_far: box c=(0.665, 0.0) he=(0.085, 0.02)   x-span [0.58, 0.75]
- [0.615, 0.0, 0.045]
- [0.665, 0.0, 0.045]
- [0.715, 0.0, 0.045]
# fin_3_near: box c=(0.05, -0.22) he=(0.10, 0.02)
- [-0.02, -0.22, 0.045]
- [0.05, -0.22, 0.045]
- [0.12, -0.22, 0.045]
# fin_3_far: box c=(0.57, -0.22) he=(0.18, 0.02)
- [0.435, -0.22, 0.045]
- [0.505, -0.22, 0.045]
- [0.575, -0.22, 0.045]
- [0.645, -0.22, 0.045]
- [0.715, -0.22, 0.045]
obstacle_cost_weight: 5000.0
obstacle_cost_decay: 0.04
obstacle_model: examples/sampling_c3/urdf/scene_slalom_oimframe.sdf
```

Notes:
- 22 discs total. The shelf_gap precedent deliberately over-covers (its r=0.133
  discs stick 0.053 past the 0.08 shelf half-depth); the exponential decay 0.04
  makes over-cover soft. The r=0.045 discs protrude only 0.025 past the 0.02
  fin half-thickness in y, preserving the 0.240 m openings (>=0.19 m effective
  clear width vs the T's 0.131 m greatest width).
- The upstream planner also carries the robot base disc (r 0.09 at origin,
  scenes.py:371). The port's existing matched scenes do NOT list the base as a
  scenario obstacle (the arm model provides its own collision); keep that
  convention.

## 3. New model file `examples/sampling_c3/urdf/scene_slalom_oimframe.sdf`

Static model, 6 box links, SDF `<box><size>` = FULL extents; z-center =
platform top (-0.029) + fin half-height (0.035) = **0.006**; orange
material to match the family:

| link       | pose (x y z)        | size (full, x y z) |
|------------|---------------------|--------------------|
| fin_1_near | 0.05  0.22  0.006   | 0.20 0.04 0.07     |
| fin_1_far  | 0.57  0.22  0.006   | 0.36 0.04 0.07     |
| fin_2_near | 0.245 0.00  0.006   | 0.19 0.04 0.07     |
| fin_2_far  | 0.665 0.00  0.006   | 0.17 0.04 0.07     |
| fin_3_near | 0.05 -0.22  0.006   | 0.20 0.04 0.07     |
| fin_3_far  | 0.57 -0.22  0.006   | 0.36 0.04 0.07     |

Header comment should mirror scene_shelf_gap_oimframe.sdf's (cite the MJX
geoms, slalom.xml:85-90). `<static>true</static>`, one `<model name="scene_slalom">`.

## 4. Friction

No yaml key — run the sim with `--matched_mu=true` (franka_sim.cc:65-160):
object 0.3, ground/platform 0.3, EE 1.5 (harmonic pairs: T-table 0.3, EE-T 0.5),
obstacles handled so object-obstacle pair realizes the MJX default 0.5.
This matches upstream exactly: tee.xml:150-151 explicit T-table pair mu 0.3,
common.xml:50 default geom mu 0.5 for fins/EE.

## 5. The 5 starts / 5 goals (per-trial overrides)

Per trial, set in `sim_params.yaml` `q_init_object(s)` = `[1,0,0,0, x, y, 0.0008]`
rotated (quaternion [cos(yaw/2), 0, 0, sin(yaw/2)]) and in `goal_params.yaml`
`fixed_target_position(s)` / `fixed_target_orientation(s)`:

| trial | start (x, y, yaw)          | start quat (w,x,y,z)          | goal (x, y, yaw)            | goal quat (w,x,y,z)            |
|-------|----------------------------|-------------------------------|-----------------------------|--------------------------------|
| 1     | 0.3810, 0.4000, 0.0000     | 1, 0, 0, 0                    | 0.3810, -0.4000, 3.1416     | -3.7e-06, 0, 0, 1              |
| 2     | 0.3660, 0.4310, 0.0579     | 0.99958, 0, 0, 0.02895        | 0.4040, -0.3930, 2.7924     | 0.17390, 0, 0, 0.98476        |
| 3     | 0.3910, 0.4370, -0.4082    | 0.97923, 0, 0, -0.20276       | 0.3630, -0.4100, 3.5684     | -0.21172, 0, 0, 0.97733       |
| 4     | 0.3610, 0.3700, 0.3282     | 0.98657, 0, 0, 0.16337        | 0.3670, -0.3800, 2.7338     | 0.20259, 0, 0, 0.97926        |
| 5     | 0.4060, 0.3930, 0.2806     | 0.99017, 0, 0, 0.13984        | 0.4040, -0.4180, 3.3892     | -0.12332, 0, 0, 0.99237       |

(z stays 0.0008; upstream jitter clearances 8.1/11.3/12.2/5.3/7.4 cm starts,
8.1/7.5/10.7/6.3/9.9 cm goals — examples/poses/slalom.yaml.)

## 6. Launch (unchanged from shelf_gap README)

```
bazel-bin/examples/sampling_c3/franka_sim --demo_name=matched_slalom_xarm6_t1 --robot_model=xarm6 --matched_mu=true
bazel-bin/examples/sampling_c3/franka_osc_controller --demo_name=matched_slalom_xarm6_t1 --robot_model=xarm6 --xarm6_five_joint=true --prelift_release=true
SAMPLING_C3_OBSTACLE_MODE=lcs_contact bazel-bin/examples/sampling_c3/franka_sampling_c3_controller --demo_name=matched_slalom_xarm6_t1 ...
```

## 7. Risk notes for the implementer

- slalom.py's docstring (y = +-0.21, 0.200 m openings) is STALE prose; the
  compiled scene and scenes.py both say +-0.22 / 0.240 m. Use the latter.
- Fins are 0.07 m tall (half the family's 0.14): upstream relies on the arm
  crossing fins repeatedly. The port's reposition PWL lift is 0.10 m
  (`pwl_waypoint_height: 0.10`), which clears 0.07 — but only by 0.03 m;
  check EE clearance during repositions (shelf memory: 0.06 lift < shelf top
  caused the P5 deadlock).
- Gate-2 near slot (0.059 m between fin_2_near at x=0.15 and the base shell)
  must stay blocked for the planner; the r=0.045 disc at x=0.185 plus the arm
  self-collision covers it.
