# Section 2 — Three-model plant comparison (xArm6 oim_t stack)

Branch: `audit/xarm6-plant-fidelity` @ base `0abc0eb7c` (c3plus-channel-route-v1 head).
Scope note: the C3+ shelf/obstacle campaign executes on the **Franka** sampling-C3 stack; the
xArm6 `oim_t` stack below is the MJX-port C++ equivalent the brief asks about. Both are covered
in `command_pipeline.md`.

## Key structural finding

There are **two** C3/LCS models, and the "ranking rollout" is not a separate simulator in the
xArm6 stack:

- **A1 — full spatial LCS** (opt-in `--planner_mode full`): `xarm6_full_sampling_c3plus.cc:1350-1447`
  `BuildSpatialLcsProblem`, derived from a real Drake MultibodyPlant via `LCSFactory::LinearizePlantToLCS`.
- **A2 — reduced "exact-T" LCS (the DEFAULT**, `xarm6_sampling_c3_controller.cc:35`): matrices
  `A,B,D,E,F,H,c` are **hand-written** (`xarm6_sampling_c3_controller.cc:981-1076`), not derived
  from any plant.
- **B — ranking** = the same `SolveOneContact` C3+ solve scored by `CandidateCost` on the terminal
  object state (`:968-979,1074-1076`); the full path additionally rolls the SAME LCS forward via
  `TrajectoryEvaluator::SimulateLCSOverTrajectory` (`xarm6_full_sampling_c3plus.cc:1754-1845`).
  The generic dairlib `sampling_based_c3_controller.cc` PD-rollout ranking is NOT a dependency of
  any oim_t target (`examples/sampling_c3/oim_t/BUILD.bazel:26-33,71-85`).
- **C — sim**: `xarm6_sim.cc:337-354`.

## Comparison table

| Property | C3/LCS (A2 default; A1 full in parens) | Ranking rollout | Full xArm6 sim |
|---|---|---|---|
| state | A2: n=10 planar (pusher xy, obj x/y/yaw + vels) `xarm6_sampling_c3_controller.cc:986-988`; (A1: 19 spatial `xarm6_full_sampling_c3plus.h:650-658`) | identical (same solve) | 6 arm q + 6 v + 7+6 floating T = 25; no pusher DOFs |
| input | A2: k=2 planar force, B(5,0)=B(6,1)=dt/m `:989-992`; (A1: 3, ±50 N) | same | 6 joint torques (`xarm6_process_common.cc:158-168`) |
| pusher representation | A2: point mass, **mass=1.0 hard-coded** `:990`, radius only as scalar offset in c[0] `:1019-1021`; (A1: floating sphere r=0.00555 m=0.057 `xarm6_lcs_pusher.urdf:9-39`) | same | capsule 0.00555×0.08415 mass 0.05 on link6 (`xarm6.xml:215-218`), tip [0,0,0.1794] |
| object mass/inertia | A2: mass 0.1, scalar planar Izz 1.24043e-4 `:994,1000`; (A1: full SDF inertia) | same | full 3×3, m=0.1, COM offset (0,-0.0149,0) (`t_block.sdf:6-13`) |
| object geometry | A2: **none** — one linearized contact plane from a sampled (point,normal); samples in `ExactTSamples()` `:915`; (A1: true two boxes) | same | two boxes: crossbar 0.089×0.0198×0.0596, stem 0.0198×0.0794×0.0596 (`t_block.sdf:19-33`) |
| pusher-object friction | A2: **frictionless** — D purely along normal `:993-996`; (A1: μ=0.4615, `oim_t.yaml`) | same | MJCF default μ=1.0 stick × 0.3 T → Drake combined 0.4615 (matches A1) |
| table friction | A2: **no ground contact at all**; (A1: object-ground 0.4615, pusher-ground 1.0) | same | table Box CoulombFriction(0.3,0.3) code-registered (`xarm6_sim.cc:345-352`) → object-table combined **0.3** (A1 MISMATCH: assumes 0.4615), stick-table 0.4615 (A1 assumes 1.0) |
| obstacle contact | **absent from every model** (open_table scenario); `simulation.scene_model` yaml key is DEAD — never parsed by xarm6_sim.cc | n/a | none |
| timestep | plan dt 0.05 s, N=5 (A2 `:986,988`; A1 `:1421-1423`); LCS plants continuous, discretized by LCSFactory | same | discrete 0.002 s (`oim_t.yaml`; `xarm6_sim.cc:337-338`) |
| contact model | A2: single complementarity row + F=1e-3·I regularizer `:1015`; (A1: Anitescu, 2 friction dirs, 5 pairs `:1410-1432`) | same | Drake default TAMSI (`set_discrete_contact_approximation` never called in oim_t); MJCF solref/solimp ignored |
| force/velocity limits | A2: **none**; (A1: workspace box + u∈±50 N + EE vel ±0.14 m/s `:1458-1476`) | same | joint effort [50,50,32,32,32,20] N·m, vel ±0.5 rad/s, joint4 spring k=175 (`xarm6_process_common.cc:163-176`) |
| arm kinematics | **absent** — reach enforced OUTSIDE C3 via IK NLPs on arm-only plant (`sampling_c3_controller.cc:383-414,840-857`) | same | full 6-DOF xArm6, damping 50, armature 1 |
| delay/rates | planner publishes @50 Hz (20 ms period); control_loop_delay 0 | n/a | robot state 500 Hz; **object state 20 Hz** (50 ms latency = one full plan step); OSC 500 Hz LcmDrivenLoop; actuator_delay 0 |

## OSC layer (xarm6_osc_controller.cc)

- Three OSC instances muxed by `OscPhaseMux` (`:616-660`): conditioned/approach, translation-only
  descent (latched monotonically), collision-aware reposition-posture.
- Tracks: EE position (kp 200/kd 20), wrist-roll joint (10/3), 6-joint posture targets (200/20),
  ExternalForceTrackingData; orientation only under `--track_tip_orientation` with z-row zeroed.
- Torque path (`XarmVelocityServoBridge` `:791-839`): OSC ID torque → subtract gravity →
  qdot_cmd = clamp(qdot + τ/kv, ±vel_limit) → τ_servo = clamp(kv·(qdot_cmd−qdot), ±effort_limit)
  → gravity re-added OUTSIDE the clamp (MuJoCo gravcomp=1 ordering). kv=[300,300,200,200,200,200].
- Reposition adds joint-damping feed-forward (`:660-670`).

## Reposition lift / reach filtering / DLS

- Lift waypoint chain {lift,lift,high,standoff,contact} (7-waypoint with neutral anchor) in
  `EvaluateMeasuredCandidateAcquisition` (`sampling_c3_controller.cc:653-830`); lift z floored by
  capsule clearance heights; reposition_speed 0.18 m/s; waypoint height 0.07738.
- Reach filtering = per-waypoint IK feasibility `ik_reached` + capsule/table/T AABB swept checks
  with geometry constants hard-copied from SDF/MJCF.
- **No DLS anywhere** — singularity handled by Drake DiffIK QP (centering 0.1, max_vel 0.1) and
  IK NLP with tilt cost w=80 in a 0.05 rad band.
- **None of these three layers are represented in either LCS model.**

## Fidelity gaps flagged

1. Default planner (A2) is frictionless and gravity-free with a fictitious 1.0 kg point pusher —
   cannot represent the 0.4615 pusher-object friction or 0.3 object-table drag that dominate
   the real push.
2. A1 friction mismatches: object-ground 0.4615 vs sim 0.3; pusher-ground 1.0 vs sim 0.4615.
3. `simulation.scene_model` yaml is dead configuration (sim builds its table in C++).
4. 25× rate gap: object feedback 20 Hz vs plan step 50 ms vs sim 2 ms.
