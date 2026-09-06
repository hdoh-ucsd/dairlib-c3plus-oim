# xArm6 OIM-T command pipeline: C3 solution → object motion (sections 3–4)

All paths rooted at the audit worktree (branch `audit/xarm6-plant-fidelity`, base `0abc0eb7c`).
Config source of truth: `examples/sampling_c3/oim_t/parameters/oim_t.yaml`.

## 0. Process topology and rates

| Block | File | Rate / trigger |
|---|---|---|
| `xarm6_sim` | `xarm6_sim.cc:333-460` | discrete plant 0.002 s (500 Hz); `XARM_STATE_SIMULATION` @500 Hz; `OIM_T_STATE_SIMULATION` @**20 Hz**; contact receipt at sim rate |
| `xarm6_osc_controller` | `xarm6_osc_controller.cc:1137-1142` | LcmDrivenLoop on robot state → 500 Hz, one OSC QP per state; publishes `XARM_INPUT_SIMULATION` (kForced) |
| `xarm6_sampling_c3_controller` | `xarm6_sampling_c3_controller.cc:1459+` | full-plan loop @50 Hz (20 ms); live reduced loop default 500 ms; plan dt 0.05 s, N=5, admm_iterations 3 |

Clock: LCM utime (µs) ×1e-6 = s; trajectory times stamped with the state message utime, so plan and OSC share the sim time base (`systems/robot_lcm_systems.cc:113`, `operational_space_control.cc:426,914`).

## 1. C3/C3+ solve → trajectories

**Full spatial C3+** (`BuildSpatialLcsProblem`, `xarm6_full_sampling_c3plus.cc:1400-1449`): LCS via `LCSFactory::LinearizePlantToLCS`, dt 0.05, N=5, 5 contact pairs, 2 friction dirs, Anitescu. State ∈ R^19 world frame (pusher p, obj quat+p, vels); input u ∈ R^3 = world force on pusher, |u|≤50 N. Workspace box [0.15,-0.6,-0.01]→[0.9,0.6,0.3] ±0.02 margin on pusher AND object; EE vel ±0.14 m/s. Sampled-contact equality band: n̂ᵀ(p_push−p_obj) ∈ n̂·point + r_pusher ± 0.003 (`:1493-1527`).

**Reduced one-contact C3+** (default; `SolveOneContact`, `xarm6_sampling_c3_controller.cc:981-1078`): hand-built 10-state planar LCS, u ∈ R^2 planar force, fictitious m_pusher=1 kg. Sign convention: n̂_W points OUT of the object; λ≥0 pushes pusher along +n̂, object along −n̂. Q=diag(0.01,0.01,200,200,50,1,1,0.05,0.05,0.05), R=0.01·I. Returns `pusher_target = states[2][0:2]` (planned pusher xy at t₀+0.10 s — the "x-pred") and `object_prediction = states[4][2:5]`; `gap=(Ex₀+c)[0]` (m, ≤0 ⇒ loaded).

**KEY: `u` is never sent to the joints on the live path.** Only the state prediction is consumed; u appears downstream only on the full-plan path as an OSC ExternalForceTrackingData reference.

## 2. Candidate selection → task-space plan

`BuildXarmFullSamplingC3TaskSpacePlan` (`xarm6_full_sampling_c3plus.cc:2365-2415`): takes `buffer.successful.front()` (ranked by dynamic rollout cost); `time_vector[k]=k·0.05`; positions = x_k[0:3]; forces = u with ZOH on the terminal input. Lateral-guarded variant pre-filters candidates by terminal |x_obj−goal_x| ≤ 0.005 m (`:2417-2441`). `EvaluateXarmFullSamplingC3PredictedContactPrefix` rejects prefixes separating from the sampled face by > 0.003 m.

## 3. Reposition / lift PWL

`BuildXarmFullSamplingC3OscExecutionPlan` (`:2443-2500`) prepends 5 acquisition knots:
current tip → planar → elevated (z=0.07738) → standoff (contact + n̂·(0.020+tol/2)) → contact = C3 col 0; then the C3 tail. Knot times: t_k = t_{k-1} + max(0.05, ‖Δp‖/0.18). Forces are ZERO on knots 0–3 (no feed-forward during lift/traverse). Waypoint geometry in `xarm6_sampling_c3_controller.cc:679-724`; `lift.z` floored by capsule clearance heights and never lowers an already-high tip; list {lift,lift,high,standoff,contact} (7-waypoint with neutral anchor).

## 4. Publishing on `OIM_XARM_TRAJECTORY`

- **4a One-shot full plan** (`:1998-2041`): `end_effector_position_target` (absolute times), stick-axis target, `end_effector_force_target` = forces_W.
- **4b Gated measured subtargets @50 Hz** (`:2340-2436`): per update, Δ to the current waypoint clipped to a step limit (approach 0.05 m / descent 0.01 / task 0.01), published as a 2-knot PWL [t_state, tip] → [t_state + max(0.05, ‖Δ‖/0.18), tip+Δ]. Loop blocks until state utime advances 20 ms and drains LCM to the newest state (message age ≲2 ms).
- **4c Live reduced loop** (`publish_target`, `:7479-7539`): 2-knot [t_state, col0] → [t_state+0.05, task_target]. **No force target ⇒ λ_des = 0** (`xarm6_osc_controller.cc:461-467`).

**Live target construction** (`solve_live_target`, `:8839-8963`) — direction-preserving Cartesian step:
```
step  = pusher_target − live_tip_xy                    # x-pred step, world m
step += (tangential_error − step·t̂)·t̂                  # tangential comp ← measured face-center error
recovery/corridor: freeze or clamp step.x ±0.01
loaded (gap≤0) & recovery: remove inward comp
else if step·inward < 0.002: add inward to 0.002       # minimum_contact_normal_step
if ‖step‖ > 0.01: rescale (GATE_380 direction-preserving limiter)
task_target = [tip_xy + step, z_obj]                   # z pinned to object plane
swept-capsule clearance fail ⇒ hold tip one update
```

## 5. Joint-space limiter (GATE_380)

`LimitXarmFullSamplingC3JointStepPreservingDirection` (`xarm6_full_sampling_c3plus.cc:226-244`): uniform scaling s = max_i |Δq_i|/(0.5·0.05); Δq/s if s>1 — replaced the componentwise clamp that distorted joint ratios. Used by `SolveVerticalContactPostureStep` (tilt band 0.05 rad + tilt cost w=80, nearest-q regularizer, swept-validated at 8 samples) and `SolveCollisionAwarePostureStep`.

## 6. OSC: target → torque

- Trajectory sources use `FirstOrderHoldWithTerminalHold` (`xarm6_osc_controller.cc:75-89`): zero desired velocity past the end; **staleness is silent** — a dead planner leaves the last target held forever.
- Three OSC QPs muxed by `OscPhaseMux` (`:585-706`): conditioned (EE Kp=200·I, Kd=20·I), translation-only (latched monotonically when target z < tip z − tol), collision-aware posture (kp 200/kd 20). All have ExternalForceTrackingData at the tip and accel reg 1e-7·I. Cartesian accel clamp ±Kp·0.05 = ±10 m/s².
- QP: ÿ_cmd = ÿ_des + Kp e + Kd ė (clamped ±10); cost Σ‖J v̇ + J̇v − ÿ_cmd‖²_W + ‖λ_e − λ_des‖²_W; dynamics M v̇ + C − τ_g = B u + J_eᵀ λ_e; |u_i| ≤ effort_limit ([50,50,32,32,32,20] N·m). OSQP; **a failed solve is not checked** before flowing to the bridge.
- Reposition branch adds damping feed-forward τ_i += d_i·clamp((q_tgt−q_start)/T, ±0.5).

## 6e. Velocity-servo bridge (`XarmVelocityServoBridge`, `:735-846`) — the only true saturation
```
τ_des  = τ_OSC − τ_g
q̇_cmd  = clamp(q̇ + τ_des/kv, ±0.5 rad/s)      kv = [300,300,200,200,200,200]
τ_cmd  = clamp(kv(q̇_cmd − q̇), ±effort) + τ_g   (gravity OUTSIDE the clamp, MuJoCo gravcomp order)
```
Optional CSV log via `--control_log_period` (0.02 s) records q, v, τ_osc, τ_servo, q̇_cmd, τ_gravity, τ_cmd — **this is the passive OSC execution log for section 5**.

## 7. Sim: applied torque → object motion

`xarm6_sim.cc:378-390`: torque enters the 2 ms plant as ZOH; no actuator delay (yaml `actuator_delay` parsed but unused), no extra saturation. Table Box μ=0.3 code-registered. `XarmObjectContactMonitor` publishes live contact point/force/depth on `OIM_T_STATE_SIMULATION_CONTACT` — **the passive physical-contact log for section 6**. Object pose returns to the planner at 20 Hz (up to 50 ms age — the dominant feedback lag).

## 8. Mapping chain (condensed)

```
x-pred p̂_push = states[2][0:2] (W, m, t₀+0.10s)
s = p̂ − p_tip; tangential swap; inward floor 0.002; ‖s‖ ≤ 0.01
p_des = [p_tip+s, z_obj] → 2-knot PWL (0.05 s) → OSC ÿ_cmd = Kp e + Kd ė (±10 m/s²)
→ QP torque (effort box) → servo bridge (vel clamp 0.5 rad/s, effort clamp, gravity outside)
→ 2 ms ZOH → plant → point contact μ_combined 0.4615 (pusher-T), 0.3 (T-table) → obj pose @20 Hz
```

## 9. Delay budget

| Hop | Latency |
|---|---|
| sim → state publish | ≤2 ms |
| state → OSC → input | one 500 Hz iteration |
| input → applied torque | ≤2 ms ZOH |
| object pose → planner | ≤50 ms (20 Hz) — dominant |
| planner → new target | 20 ms (full) / 500 ms (live default) |
| target expiry | none — terminal hold forever |

## 10. Confirmed gaps

1. λ_e sign on the full-plan path: u_C3 (force on pusher) is fed as positive external force on the robot — reaction convention unverified; only affects the full-plan path.
2. u_C3 otherwise unused — live loop is a pure position servo on the knot-2 prediction; the planned contact force never reaches the arm.
3. No message-age gate anywhere; stalled planner ⇒ silent hold.
4. `actuator_delay` yaml silently ignored.
5. OSQP failure in OSC unchecked.
