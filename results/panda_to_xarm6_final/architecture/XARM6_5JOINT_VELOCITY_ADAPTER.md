# xArm6 5-joint velocity adapter — final formulation

Location: `examples/sampling_c3/franka_osc_controller.cc` (`Xarm6FiveJointVelocityExecutor`,
active with `--robot_model=xarm6 --xarm6_five_joint=true`). Plant: `xarm6_policyport.xml`
(joint 6 welded, damping 50, armature 1, joint-4 spring 175 via RevoluteSpring, 5 torque
actuators as the Drake substrate under software velocity servos; Franka EE welded to link6 with
the Franka transform — contact geometry identical to the Panda's).

## Equations (final, round-5 state)

Task rows (5-D, stick yaw dropped — the MJX construction):
```
p_des(t), v_des(t)  from end_effector_position_target (FirstOrderHold; planar radius of p_des
                    clamped direction-preservingly to 0.68 m — reach safety)
v_task[0:3] = v_des + Kc (p_des − p_tip),        Kc = 4.0 1/s
a = R_tip ez ; s = sign(a_z)
v_task[3] = s·α·a_y ; v_task[4] = −s·α·a_x,      α = 2.0 1/s   (tilt-to-vertical bias)
```
Weighted damped square solve (tilt = soft bias, translation primary — MJX semantics):
```
W = diag(1,1,1,0.2,0.2);  Jw = W J   (J = rows [vx vy vz wx wy] of the 6×5 tip Jacobian)
q̇_cmd = (Jwᵀ Jw + λ² I)⁻¹ Jwᵀ W v_task,          λ = 0.05
direction-preserving saturation: q̇_cmd *= 0.5 / max|q̇_cmd,i|  if that max > 0.5 rad/s
NaN guard: q̇_cmd = 0 if non-finite
```
Velocity servo + gravity comp (MuJoCo gravcomp ordering, oim_t-bridge validated):
```
τ_i = clamp(kv_i (q̇_cmd,i − q̇_i), ±effort_i) + τ_grav,i
kv = [300,300,200,200,200] ; effort = [50,50,32,32,32] N·m
```
Rates: runs at the robot state rate (1 kHz); trajectory updates at the planner's 62.5 Hz stream.

## Measured lessons encoded above

- Per-joint q̇ clamping distorts the task direction (r1: descent stall) → uniform scaling.
- Equal-weight tilt rows pin the solve in kinematic traps (r4: 6 cm stall at a reachable target,
  saturated press → sim NaN) → W tilt 0.2.
- Candidates near the reach shell are unreachable at lift height (r2: assert crash chase) →
  executor planar clamp 0.68 (declared policy workspace stays 0.70; 0.66 starves the sampler, r3).
- The pre-lift release phase (separate system upstream) retreats to ≥0.095 m before any commanded
  lift; with matched μ it eliminated the topple class (0 in the last 10 trials).
