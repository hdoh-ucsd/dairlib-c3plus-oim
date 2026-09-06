# xArm6 execution adapter (below the Cartesian interface)

Commit: 000eef388. The robot-independent interface is the existing LCM contract
(`TRACKING_TRAJECTORY_ACTOR`: 5-knot EE position / orientation / feed-forward-force trajectories,
world frame). Nothing above it changed; the planner binary is the same and its policy class has zero
robot references.

## Components

1. **Model asset** `urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml` — xarm6.xml minus the stick
   tool (the Franka end-effector is welded on instead so contact geometry is IDENTICAL across
   robots), and joint damping 50 → 1 (see fix 1).
2. **`AddXarm6ToPlant`** (sampling_c3_utils) — MJCF parse; 6 JointActuators (efforts
   {50,50,32,32,32,20} N·m, vel ±3.1416 rad/s; Drake does not import MJCF actuators); weld
   `end_effector_flange` → `xarm6_link6` with the exact Franka transform (RPY(π,0,0),
   kToolAttachmentFrame); identical ground/platform scene branch.
3. **`--robot_model` switch** in franka_sim / franka_osc_controller / franka_sampling_c3_controller
   at their single AddFrankaToPlant call sites (plus a hardcoded object-offset fix in franka_sim that
   assumed 7 robot positions).
4. **OSC task hierarchy for 6-DOF** (osc_params_xarm6.yaml + one gate):
   - `panda_joint2` posture task (the Panda's redundancy resolver) gated off for xarm6;
   - `EndEffectorRotW` tool-axis (roll) diagonal → 0 — tool roll is physically irrelevant (sphere
     tip; the policy never commands roll), recovering the 1 slack DOF;
   - W_accel [0.01×6], W_input_reg [1,1,1,1,1,10];
   - EE translation gains unchanged except Kd 20 → 40 (fix 3).
5. **Demo** `push_t_bt010_open_table_xarm6` — byte-copy of the Franka demo except: 6-dim q_init from
   IK matching the Franka EE start tip to ≤0.1 mm ([0.8876, 0.268, −1.1745, 0.003, 0.9179, 0.0] →
   tip [0.3266, 0.4007, 0.0732], tool vertical), the osc_params_xarm6 pointer, and neutral_position
   = start tip (fix 2). Every planner/sampling/cost/progress/buffer parameter untouched.

## Execution-layer fixes (all robot-specific, all below the interface)

1. **Damping model mismatch**: the OSC inverse-dynamics QP models M·v̇ + bias − gravity only
   (inverse_dynamics_qp.cc:172-175) — MJCF joint damping 50 (tuned for MuJoCo velocity servos) is an
   unmodeled 50·q̇ load that saturated every effort limit (joints plateau at τ_max/50 ≈ 1 rad/s).
   Damping set to 1 in the policyport model, consistently across sim/OSC/planner plants.
2. **Startup transient**: the shared neutral_position sits 0.23 m from the start tip; the initial
   swing swept the tip past the planner's workspace DRAKE_DEMAND. xarm6 neutral = start tip.
3. **Deceleration overshoot**: with the Franka Kd=20 the xarm6's lower effort ceilings saturate
   during braking → ~2× step overshoot through the workspace wall. Kd=40 (ζ≈1.4).

## What was NOT done

No change to candidate generation, ranking, buffers, progress, hysteresis, reposition policy,
C3/Q/R, route logic, thresholds, or the orientation-command heuristic. No new outer-loop mechanism.
