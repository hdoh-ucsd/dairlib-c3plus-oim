# Panda controller layer map (robot-independent vs robot-specific)

Source audit of the working Franka C3+ stack (demo `push_t_bt010_open_table`). Classification:
A = robot-independent policy; B = robot-specific execution; C = mixed (port must swap).

## Headline

`systems/controllers/sampling_based_c3_controller.{h,cc}` (5,094 lines) contains **zero** Franka
references (verified by grep for franka/panda/joint/IK). Its `plant_` is the LCS plant (floating
3-DOF sphere EE + object + ground). **The entire planner class is layer A.** All robot dependence in
the planner process lives in `franka_sampling_c3_controller.cc` main() wiring:

1. plant build — `:87-90` `AddFrankaToPlant` (`sampling_c3_utils.cc:15-58`) — **B**
2. state decode — `:371-372` `RobotOutputReceiver(plant_franka)` — **C** (DOF count + joint names)
3. FK — `:385-391` `FrankaKinematics` (`systems/franka_kinematics.cc:130-155`): EvalBodyPoseInWorld /
   EvalBodySpatialVelocityInWorld of body `end_effector_tip` → LCS state `[p_EE, q_obj, v_EE, v_obj]`,
   world frame — **C** (FK only; no Jacobian, no IK, no joint-limit or manipulability check anywhere
   in the planner process)

Workspace feasibility is purely declarative Cartesian yaml (workspace_limits, robot_radius_limits)
consumed in Cartesian arithmetic (layer A code; robot-calibrated numbers).

## Stage table (condensed; full detail in panda_function_classification.csv)

| Stage | Site | Class |
|---|---|---|
| robot plant build | sampling_c3_utils.cc:15-58 | B |
| robot state ingest | franka_sampling_c3_controller.cc:371-372 | C |
| FK → LCS state | franka_kinematics.cc:130-155 | C |
| goal generator + lookahead + hysteresis | goal_generator.cc:107-434 | A |
| target mux | franka_sampling_c3_controller.cc:414-493 | A |
| predicted x0 | sampling_based_c3_controller.cc:3109 | A |
| workspace check | :3179-3197 | A (yaml numbers robot-calibrated) |
| cost update (incl. quat-dependent 510) | :3201 | A |
| candidate generation (kRandomOnPerimeter) | generate_samples.cc:24-867 | A |
| goal-reached park pose (0.3,0.4,0.1) | :2440-2449 | C (Cartesian constant) |
| LCS per sample + C3+ solve | :3330-3352, :2184 | A |
| rollout/cost (type 5) + buffers + selection | :1385, :3777-3936, :2453-2736 | A |
| progress detector (kConfigCostDrop 1%/180) | :3983-4080 | A |
| C3 exec trajectory (z override, force traj) | :3441-3577 | A |
| EE orientation synthesis (tilt ≤20° from workspace center) | :3516-3549 | C (Franka wrist-singularity heuristic) |
| reposition PWL trajectory | reposition.cc:13-394 | A |
| LCM emission (TRACKING_TRAJECTORY_ACTOR) | :4622-4651 | **A — the robot-independent interface** |
| OSC ingest (3 named trajectories) | franka_osc_controller.cc:98-109 | A (names) |
| OSC tasks/torque | franka_osc_controller.cc:89-235 | B |
| sim | franka_sim.cc:81-162 | B |

## The Cartesian interface (must be preserved exactly)

Channel `TRACKING_TRAJECTORY_ACTOR`, `lcmt_timestamped_saved_traj`, three named 5-knot trajectories
with absolute times `t_ctx + filtered_solve_time + i*dt`:
`end_effector_position_target` (3x5, m, world) / `end_effector_orientation_target` (4x5 quat wxyz) /
`end_effector_force_target` (3x5, N = C3 u_sol). This is the CartesianExecutionCommand of the brief;
a 6-DOF port re-implements only plant build, state decode, FK, and the OSC/sim below this line.
