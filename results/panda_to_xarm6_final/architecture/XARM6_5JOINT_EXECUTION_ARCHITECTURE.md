# xArm6 intended execution architecture (source-traced)

## Verdict on the §1 questions

1. **Actively controlled joints (intended MJX benchmark): joints 1–5 only.** All five are MuJoCo
   `<velocity>` servos (ctrl = target joint velocity, rad/s): kv = [300,300,200,200,200], ctrl
   ±0.5 rad/s, force limits [50,50,32,32,32-] N·m (`oim/models/xarm6/xarm6.xml:227-272`).
2. **Joint 6 does not exist in the intended model** — welded out (`xarm6.xml:201-205`: "joint6
   (wrist roll) is fixed, not actuated — rolling a symmetric stick about its own axis has no effect
   on pushing"; matches the vendor xarm6_stick.urdf). Joint 4 is actuated AND carries a passive
   spring 175 N·m/rad (`:191-192`); MPPI holds its mean at 0 noise (`configs/robots/xarm6.yaml:19`).
3. **Action space = 5 joint velocities.** MPPI samples ctrl directly (`oim/algs/mppi.py:206-215`);
   not Cartesian.
4. **No IK anywhere.** A tip Jacobian exists only as a noise-shaping/bias device
   (`oim/worlds/sim3d/run.py:46-166`): 5×5 damped inverse over task rows [dx,dy,dz,wx,wy]; z and
   tilt are *regulated* by a bias term (−α·error), x/y are free; exploration noise is confined to
   the null space of the z/tilt rows. It never computes the command.
5. **No torque control on the intended path** — velocity servos + `gravcomp="1"` on all links.
   Steady-state velocity tracking is kv/(kv+damping) ≈ 80–86% (damping 50, doc comment
   `xarm6.xml:242-244`).
6. **Joint 6 role:** none in MJX. The C++ oim_t port re-enabled it (6 torque actuators emulating
   velocity servos) "for posture/null-space regulation" — a deliberate port deviation; the
   "five joints" wording in RUN_2026-08-29.md is stale.
7. **Commanded Cartesian DOFs: 5** — x/y (pushing, free), z (regulated to push height), two tilts
   (regulated to vertical).
8. **Constrained/relaxed orientation:** stick yaw dropped entirely (axisymmetric capsule) — this
   is what makes the Jacobian square 5×5.

## Consequence for the port

The previous 6-DOF torque-OSC adapter is **NOT faithful** (answers §19-Q4: no). The faithful
xArm6 executor below the robot-independent Cartesian interface is:

- plant: 5 revolute joints (joint 6 welded/held), joint damping 50, armature 1, joint-4 spring 175,
  gravcomp;
- command: joint velocities, ctrl ±0.5 rad/s, realized by per-joint velocity servos
  τ = clamp(kv·(q̇_cmd − q̇), ±effort) + gravity compensation (the same software-servo construction
  the oim_t bridge already validated against MuJoCo ordering);
- mapping from the policy's Cartesian reference (position trajectories) to q̇_cmd: an outer
  Cartesian loop v_task = ṗ_des + Kc·(p_des − p_tip) for [x,y,z] plus tilt-regulation rows
  [−α·tilt_y, +α·tilt_x], then q̇_cmd = J_damped⁻¹(q) v_task on the 5×5 tip Jacobian — the same
  damped square inversion the MJX harness uses for its bias/noise map. This mapping is the port's
  robot adapter (the MJX policy commanded joint velocities natively; ours must convert the
  preserved dairlib Cartesian contract).

See `xarm6_joint_control_inventory.csv` for the full per-joint table with file:line citations.

## Independent cross-check (Agent D @323e3d8b7)

The 5-joint velocity verdict was independently confirmed against the OIM checkout (5 velocity
actuators xarm6.xml:263-272, ctrl = target qdot, uniform ±0.5 ctrlrange, joint6 FIXED at :201,
qdot written directly to ctrl in sim3d/run.py:520,786; the real arm publishes 5 joint velocities
with joint6=0, real3d/interface.py:102,168; no per-scene kv/ctrlrange overrides). Nuance
acknowledged: upstream's damped 5×5 Jacobian is MPPI noise/task *shaping* — the optimizer's
action IS qdot; our Cartesian-target → damped-J → qdot adapter is an equivalent-intent
realization of the same kinematic map required because the preserved dairlib policy emits
Cartesian references rather than joint velocities.
