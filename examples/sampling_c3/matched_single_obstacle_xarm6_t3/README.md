# matched_single_obstacle_xarm6_t3

xArm6 (5-joint, joint6 welded) variant of the C3+ single_obstacle scene.
Constructed as a copy of `push_t_bt010_open_table_xarm6` (FROZEN baseline,
see `results/panda_to_xarm6_final/baseline/xarm6_panda_equivalent_baseline.yaml`)
merged with ONLY the scene bits of `push_t_bt010_single_obstacle`:

- `parameters/scenario_params.yaml`: obstacle disc [0.5, 0.0, r=0.0707],
  `obstacle_model: examples/sampling_c3/urdf/single_obstacle_box.sdf`
  (verbatim from the franka scene demo).
- goal_params.yaml and q_init_object are byte-identical between the franka
  scene demo and the xarm6 open_table demo (start [0.5, 0.3], goal
  [0.5, -0.3]), so no merge was needed.

Kept from the xarm6 baseline (FROZEN — do not tune):
- 5-dim `q_init_franka` in sim_params.yaml
- `osc_params_xarm6.yaml` pointer in sampling_c3_controller_params.yaml
- `robot_radius_limits: [0.25, 0.70]` in sampling_c3plus_options.yaml
- `pwl_waypoint_height: 0.10` in reposition_params.yaml

## Obstacle-mode env contract (campaign uses lcs_contact)

The inner-QP obstacle terms are env-gated in
`systems/controllers/sampling_based_c3_controller.cc` (see `ObsCfg()`,
`std::getenv("SAMPLING_C3_OBSTACLE_MODE")`). The campaign contract is:

```
# on the planner process (franka_sampling_c3_controller) ONLY:
export SAMPLING_C3_OBSTACLE_MODE=lcs_contact
```

- Unset -> frozen baseline: NO obstacle code runs in the planner
  (only the ranking cost from scenario_params obstacle_cost_weight).
- `lcs_contact` -> obstacle enters the LCS as a frictionless normal contact
  on the object (fixed slots, closest-footprint witness).
- Must NOT be combined with `SAMPLING_C3_INNER_OBS_MODE` or
  `SAMPLING_C3_OBJ_NONPEN=1` — the controller throws at startup on the
  conflict (by design).
- Optional overrides (campaign uses the defaults):
  `SAMPLING_C3_OBS_SLOTS` (N_closest, default 2),
  `SAMPLING_C3_OBJ_MARGIN` (obs_margin, default 0.01 m).

## Launch (sim trio)

```
bazel-bin/examples/sampling_c3/franka_sim \
  --demo_name=matched_single_obstacle_xarm6_t3 --robot_model=xarm6 --matched_mu=true
bazel-bin/examples/sampling_c3/franka_osc_controller \
  --demo_name=matched_single_obstacle_xarm6_t3 --robot_model=xarm6 \
  --xarm6_five_joint=true --prelift_release=true
SAMPLING_C3_OBSTACLE_MODE=lcs_contact \
bazel-bin/examples/sampling_c3/franka_sampling_c3_controller \
  --demo_name=matched_single_obstacle_xarm6_t3 --robot_model=xarm6
```

(all with a shared `--lcm_url`, e.g. a private
`udpm://239.255.76.67:7991?ttl=0`.)

Workspace fit (xarm6 declared radius <= 0.70 from base): object start
(0.5, 0.3) r = 0.583, goal (0.5, -0.3) r = 0.583, obstacle center (0.5, 0.0)
r = 0.500 — all inside.
