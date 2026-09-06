# Robot-independent execution contract

The preserved dairlib C3+ outer loop (planner process, byte-identical for both robots) emits its
complete execution intent on ONE LCM channel; nothing robot-specific crosses it.

## Channel

`TRACKING_TRAJECTORY_ACTOR` — `dairlib::lcmt_timestamped_saved_traj` containing an `LcmTrajectory`
with three named 5-knot sub-trajectories, absolute times `t_ctx + filtered_solve_time + i·dt`:

| name | shape | meaning | frame/units |
|---|---|---|---|
| `end_effector_position_target` | 3×5 | pusher tip position reference (C3 knots in push mode; PWL lift/traverse/descend in reposition mode) | world, m |
| `end_effector_orientation_target` | 4×5 | tool tilt heuristic quaternion (wxyz; never commands stick roll) | world |
| `end_effector_force_target` | 3×5 | C3 u_sol feed-forward force | world, N |

Implicit in the stream (recoverable, not separately messaged): selected candidate and contact
sector (the trajectory terminal approaches the sampled contact), reposition target (PWL terminal),
push direction (knot differences), and mode (C3 vs reposition trajectory shape).

## Consumers

- **Panda executor**: 7-DOF torque OSC (EE translation task + orientation task + joint-2 posture
  redundancy task + external-force task) — the original dairlib implementation, unchanged.
- **xArm6 executor (faithful)**: 5-joint velocity-control adapter — the position trajectory is
  converted to a 5-D task velocity [v_xy free-tracking, v_z height regulation, tilt_x/tilt_y
  regulation to vertical; stick yaw dropped], mapped through a damped square 5×5 tip Jacobian to
  joint velocity commands (ctrl ±0.5 rad/s), realized by per-joint velocity servos
  (τ = clamp(kv(q̇_cmd−q̇), ±effort) + gravity compensation) — mirroring the MJX benchmark's
  actuator model. The orientation and force sub-trajectories are not consumed (the intended
  architecture regulates tilt internally and has no force feed-forward).
- **xArm6 pre-lift release** (robot-specific execution correction, below this contract): when the
  incoming trajectory commands a lift while the tip is still mechanically wedged against the
  object, the executor first retreats planarly along the outward direction until the contact is
  released, then follows the original trajectory unchanged. No policy message is altered.

## Guarantees

Both executors receive the SAME logical intent for the same controller state. Divergent robot
behavior below the contract is attributable to robot/execution differences only — the requirement
of the fair comparison.
