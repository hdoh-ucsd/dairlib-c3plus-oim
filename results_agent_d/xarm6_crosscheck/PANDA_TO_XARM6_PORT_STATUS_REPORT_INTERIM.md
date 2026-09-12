# Panda 7-DOF → xArm6 Faithful Port — Full Status Report (interim)

**Date:** 2026-09-06. **Status: CLOSED @ ab8ba8b12 — FINAL VERDICT B_POLICY_EQUIVALENT_BUT_5JOINT_EXECUTION_NOT_YET_RELIABLE; baseline NOT frozen (gate not met).**
Two coordinated sessions: the *xarm6 execution fidelity* session (worktree `audit-xarm6-plant`,
branch `audit/xarm6-plant-fidelity`, latest `01def72fe`) owns the implementation, lock-bound
campaigns, and final report; this session (worktree `audit-ycb-icra`) owns the upstream OIM
cross-checks and offline forensics. The final
`PANDA_C3PLUS_TO_XARM6_5JOINT_FAITHFUL_PORT_REPORT.md` will be assembled by the peer after
round 3; this document is the complete current picture.

---

## 1. Architecture question — RESOLVED (§1–§3 of the brief)

**The intended OIM xArm6 controller is 5-joint velocity control; the prior 6-DOF torque-OSC
port was NOT faithful and has been replaced.**

Source-traced verdict (peer's `XARM6_5JOINT_EXECUTION_ARCHITECTURE.md`, independently
confirmed by my OIM cross-check @`323e3d8b7`):

| # | Question | Answer | Evidence |
|---|---|---|---|
| 1 | Active joints | joints 1–5, MuJoCo `<velocity>` servos, ctrl = target q̇ (rad/s) | `xarm6.xml:227-272`: kv [300,300,200,200,200], ctrl ±0.5, force [50,50,32,32,32] |
| 2 | Joint 6 | **does not exist** — welded out ("rolling a symmetric stick has no effect on pushing") | `xarm6.xml:201-205` |
| 3 | Action space | 5 joint velocities, sampled directly by MPPI/ADMM — not Cartesian | `mppi.py:206-215`, `sim3d/run.py:520,786` (`ctrl[:] = u`) |
| 4 | IK / Jacobian | **no IK on the command path**; the damped 5×5 tip Jacobian [dx,dy,dz,wx,wy] is a noise-shaping/bias device only | `sim3d/run.py:46-166` |
| 5 | Torque control | none — velocity servos + gravcomp; steady-state tracking kv/(kv+damping 50) ≈ 80–86% | `xarm6.xml:121-124, 242-244` |
| 6 | Joint 6 role | none in MJX; the older C++ oim_t port's 6th torque actuator was a deliberate deviation | port note |
| 7 | Cartesian DOFs | 5: x/y free (pushing), z regulated to push height, two tilts regulated to vertical | `run.py:119-131` |
| 8 | Orientation | stick yaw dropped entirely (axisymmetric capsule) → square 5×5 Jacobian | same |
| — | Per-scene overrides | none: kv/ctrlrange global; ±0.5 is a 2026-08-18 uniform safety ceiling | `scenes.py`, `xarm6.yaml` — zero hits |
| — | Real arm | publishes the same 5 joint velocities, joint6 = 0 | `real3d/interface.py:102,168` |

**Implemented faithful executor** (peer): plant with 5 revolute joints (joint 6 welded), joint
damping 50, armature 1, joint-4 passive spring 175 N·m/rad, gravcomp; per-joint software
velocity servos τ = clamp(kv·(q̇_cmd − q̇), ±effort) + gravity comp (MuJoCo-conformant
ordering); robot adapter converts the preserved dairlib Cartesian reference via
v_task = ṗ_des + Kc(p_des − p_tip) (+tilt-regulation rows) → q̇_cmd = J_damped⁻¹ v_task
(Kc=4, α=2, λ=0.05), q̇ limit ±0.5. Acknowledged nuance: upstream's optimizer action IS q̇ —
the Cartesian→q̇ adapter is an equivalent-intent realization of the same kinematic map,
needed because the preserved policy emits Cartesian references.

**Policy preservation:** no candidate generation, ranking, lookahead, state-machine, progress,
buffer, hysteresis, or success-threshold code was touched; the robot boundary is the Cartesian
execution reference (peer's `ROBOT_INDEPENDENT_EXECUTION_CONTRACT.md`).

## 2. Execution-layer fixes landed (§5, §9)

- **Pre-lift contact release** (`--prelift_release`): on REPOSITION_REQUESTED with active
  contact, retreat along the contact normal until the gap check clears, then run the original
  Panda PWL lift/lateral/descend. Addresses the measured 6-DOF-era failure (lift while wedged →
  friction scoops the T → topple in <0.5 s).
- **Topple guard** (tilt > 0.5 rad terminates trial as TOPPLE) + finite iteration caps (10k) on
  the PerimeterSampling loops, both robots — robustness only.
- **Direction-preserving q̇ scaling** (round-2 root cause): the original per-joint ±0.5 clamp
  distorted the task direction (descent stalled at z=+0.05 while xy tracked); replaced by
  uniform scaling so max|q̇ᵢ| = 0.5. Tracking now mm-class at all heights. (Same fix family as
  the Python-side GATE_380 direction-preserving limiter — independent convergence.)
- **Reach fixes** (round 3): declared reach 0.66 m + executor planar-reach clamp 0.68 m
  (round-2 trial2 died on a planner radius assert chasing an unreachable r=0.67 lift target).

## 3. Matched physics (§7)

`friction_provenance.csv` (peer) + my OIM extension: intended benchmark values are
**T–table μ = 0.3** (explicit MJCF pair, `tee.xml:150-151` — all tabletop scenes incl.
ycb_clutter), **EE–object 0.5** (default geom, `common.xml:50`); the old Drake sim ran
effective ~1.0/0.4615 due to omitted SDF friction. `--matched_mu` implements the benchmark
values. Also recorded for the obstacle stage: **object–obstacle μ = 0.5** (default geom).

## 4. Campaign history (§13 gate: ≥3/5 required — NOT yet reached)

**Round 1** (5-joint executor, first build): 0/5-track — all zero-object-motion; root-caused to
the per-joint clamp direction distortion. Labeled ablation.

**Round 2** (direction-preserving scaling): 0/5, but the failure anatomy transformed
(my forensics, `r2_trial1_trial4_classification.md`):

| trial | outcome | class |
|---|---|---|
| 1 | 0.446/2.52, contact fraction **1.1%**, EE at the 2.0–2.5 cm candidate standoff 86% of the run, 25 reposition timeouts | **X5 acquisition (standoff conversion)** — the *same* class Agent A measured on the Franka; shared, so comparison-fair |
| 2 | planner radius assert at r=0.67 → OSC chase | X4 workspace (fixed in r3) |
| 3 | topple t=70 | X9 stability |
| 4 | **reclassified**: one 0.2 s tilt blip (0.37 rad @ t=272.6, survived), then **best 0.009 m @ t=367**, ended t=389 at 0.017/0.33 **still converging** | **X10 wall-clock truncation** (not topple) |
| 5 | 0.204/0.255 truncated, no topple | X10 wall-clock |

FK checks: the executor descends to push height correctly (EE z p10 = −0.003; 0.087 is hover
altitude); trial4's surviving scoop blip is positive evidence for the release mechanism.

**Round 3 — clean negative (labeled reach-starvation ablation)**: declaring reach 0.66
starved the perimeter sampler around the start pose (unsuccessful-buffer overflow, reposition
timeouts, 0/5 zero motion). Instructive: the declared workspace participates in candidate
viability, so it cannot double as the safety clamp.

**Round 4 (acceptance five) — RUNNING**: declared workspace restored to 0.70 (candidates
viable) + executor-side 0.68 planar reach clamp retained for safety + 1200 s wall budget —
r2's productive configuration plus the wall-clock and reach-chase fixes. Trials land in
`results/panda_to_xarm6_port_final_r4/xarm6_trial{1..5}`. Peer's release logs confirm active
pre-lift retreats (START→DONE to 0.095 m), completing the §5/§6 causal evidence for the
release mechanism (trial4's survived scoop).

## 5. Remaining §-items pending round 3

Pre-lift release A/B (§4/§6), matched exact-contact replays (§6/§11), contact-conditioned
prediction on xArm6 data (§7/§12 — my analyzers are xArm6-FK-ready), five-joint velocity
fidelity CSV (§8), acceptance summary + gate decision (§12), baseline freeze **only if ≥3/5**
(§13), final report + handoff (§16). Next-stage obstacle manifest (§14) is already written
(`next_stage_obstacle_manifest.md`): single_obstacle / shelf_gap / ycb_clutter (2 of 4 OIM
obstacles still unported) / faithful icra_sign (complete, my port), with per-scene obstacle-top
heights and the object–obstacle μ=0.5 requirement.

## 6. Current honest bottom line

- Architecture faithfulness: **proven and independently corroborated** (5-joint velocity,
  joint 6 welded, 6-DOF OSC retired).
- Policy: **unchanged** — every fix so far lives below the robot boundary (executor scaling,
  release, reach clamp, guards, physics matching).
- Reliability: **gate not yet passed**; round-2 anatomy says the remaining failure mass is
  wall-clock (fixed), workspace assert (fixed), one genuine topple, and the acquisition class
  both robots share. Round 3 is the decision campaign.
- **FINAL (r5, closure @ ab8ba8b12)**: 0/5 but **zero topples across 10 straight ~800 s
  trials — the stability/topple class is closed for good**. 2/5 still progressing at the
  1200 s budget, 3/5 kinematic-trap stalls at reachable targets, one residual sim-NaN.
  Gate not met → baseline correctly NOT frozen. **Verdict:
  B_POLICY_EQUIVALENT_BUT_5JOINT_EXECUTION_NOT_YET_RELIABLE.**
  Ranked remaining work (peer handoff): X6/X2 kinematic-trap escape in the executor;
  X4 shared acquisition class; **X9 fair-budget decision — NEEDS THE USER** (the 0.5 rad/s
  velocity-servo arm runs the task ~10× slower than the Panda; the comparison time budget
  must be chosen deliberately); X7 planner-ingestion finiteness guard. Obstacle scenes
  untouched per §20; the obstacle manifest is ready for when the gate passes.
