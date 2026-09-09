# Obstacle Contact Pipeline Trace — lcs_contact mode (frozen xArm6 C3+ baseline)

Run: results/xarm6_c3plus_scene_smoke/runs/single_obstacle/pair01.
planner.log:26 confirms `[OBS-LCS] lcs_contact ACTIVE: n_obs_slots=2 n_lambda=18
obstacle_cost_active=false obstacle_lcs_contact_active=true`.
All cites = systems/controllers/sampling_based_c3_controller.cc unless noted.

## Stage table

| # | Stage | Verdict | Where |
|---|---|---|---|
| 1 | Obstacle geometry query (witness) | PRESENT | ComputeObstacleLcsContacts :685-763; exact AABB branch :707-731 (box from OBS_BOXES), disc fallback :731-740; ObsSdfPoint :419-434 shares the formulas |
| 2 | C3 subproblem: obstacle slots in LCS | PRESENT | n_obs_slots_lcs_=2 appended to n_lambda_ at :1196-1212; per-candidate augmentation AugmentLcsWithObstacleContacts :769-839 called at :3579-3583 (solve LCS); G/U expanded ExpandGUForObstacleSlots :843-869, applied :1205-1207 and :3423-3425 |
| 3 | ADMM solve: obstacle lambda variables | PRESENT (structural) | slots are ordinary columns/rows of D/E/F/H handed to c3::C3; controller comment :1197-1198: "Everything downstream (placeholder, G/U, z slicing, projection) sizes itself from the augmented n_lambda_". Solver core is the external @c3 library (not in this repo) — exact solver lines UNKNOWN here, but sizing is generic |
| 4 | Projection step | PRESENT (structural) | same as 3: projection operates on the full z=[x,lambda,u] with expanded G/U; obstacle lambdas projected like any frictionless normal (cone = lambda>=0) |
| 5 | Ranking rollout (cost_type-5) | PRESENT | the cost-simulation LCS is ALSO augmented with the SAME frozen contacts: :3611-3617 ("solve and rollout share one obstacle-contact model (phi, n, r, J)"); fine-dt options built :3595-3607 |
| 6 | Candidate ranking exp cost | ABSENT (by design) | guard `!ObsCfg().lcs_contact` at :2522-2523 (comment :2519-2521: "In lcs_contact mode NO obstacle objective is applied anywhere"); startup conflict throw :962-970. The Jobs exp term at :3089-3094 is inside the PASSIVE CostLogger reconstruction (:3059-3062, active only with SAMPLING_C3_COST_LOG_DIR — unset in this run) and feeds nothing back |
| 7 | Executed trajectory / OSC | ABSENT | UpdateC3ExecutionTrajectory :3647-3681 just re-packages the C3 state solution knots (EE positions) — no obstacle term; the OSC tracks that trajectory with no obstacle knowledge. Obstacle influence reaches execution ONLY through the plan already produced by the augmented LCS. Stage-2.x pusher-obstacle filter/swept-check are env-gated OFF here (:2585-2588, :3836-3839, need pusher_filter/swept_check envs + obstacle_cost_weight guard AND CostLogger) |

Reposition path veto (repos_path_veto default ON, :236) does consult obstacles for
reposition targets; it is footprint/pusher clearance heuristics, not contact physics.

## Frozen-witness question (asked explicitly)

**Yes — the LCS obstacle complementarity uses a FROZEN measured-pose witness, recomputed
once per planning cycle and held constant for the whole solve.**

- :3546-3552: `obs_contacts = ComputeObstacleLcsContacts(x_lcs_curr, ...)` — computed ONCE
  per controller cycle from the current measured object pose, explicitly "shared by all
  candidates" (comment :3544-3546), and reused unchanged for both the solve LCS and the
  rollout LCS of every candidate.
- Inside the LCS, each slot is a single frictionless row with constant Jacobian
  Jo = [rxn, nx, ny] on (omega_z, v_x, v_y) (:809-815) and constant gap offset phi/dt
  (:832-833). Over the N-knot horizon phi evolves only through the LINEARIZED normal
  velocity at the frozen witness point/normal; the footprint point, normal, and lever arm
  never update within the horizon or across ADMM iterations.

**Can the physical object out-run the witness within one solve interval? Yes, plausibly:**
1. Cadence: the witness refreshes once per planning cycle (plan period dt_ per knot,
   solve latency filtered_solve_time_ used at :3665-3677). At measured C3+ solve costs of
   O(100-500 ms/cycle) on this stack, an object being pushed at a few cm/s moves
   O(0.3-1.5 cm) between witness refreshes — same order as the 1 cm margin
   (nonpen_margin 0.01, :201).
2. Single-point contact: only the ONE closest footprint point per obstacle is constrained
   (argmin over Footprint(), :742-748; 2 slots but obstacle 1 is the far-away base disc,
   so the box effectively gets ONE contact row). A rotating T can swing a corner into the
   box face while the constrained closest point stays clear — rotation about the witness
   point is unconstrained curvature the linear row cannot see.
3. Frozen normal: if the closest feature changes (edge→corner, face→adjacent face) mid-
   horizon, the plan satisfies the stale halfplane while the true SDF is violated.
4. The sim enforces the true geometry regardless (collision pair active, see
   geometry/collision_filter_inventory.csv), so planner-side witness lag manifests as the
   pusher driving the T into the box and the SIM contact force (not the planner) stopping
   it — i.e. real contact with compliant-solver penetration, plus any rendering overdraw.

## Sim contact response config (for "what depth is normal")

- Plant: discrete, `dt: 0.0001` (sim_params.yaml:11) via AddMultibodyPlantSceneGraph
  (franka_sim.cc:122). NO explicit contact model/solver/penetration_allowance is set
  anywhere in franka_sim.cc (grep contact|penetration|stiction = 0 hits) → pure Drake
  defaults: contact model kHydroelasticWithFallback (these SDFs/URDFs declare no
  hydroelastic properties → point contact fallback) with the Drake-default discrete
  solver/approximation for the pinned Drake version.
- Expected NORMAL compliant depth for the ~0.1 kg T pressed laterally into the static box:
  not exactly derivable from this repo (depends on Drake's default discrete contact
  approximation for the vendored version). Order-of-magnitude bounds:
  - SAP-family default (near-rigid regime at dt=1e-4): micrometer scale, <=1e-5 m.
  - TAMSI/penalty with Drake's default penetration_allowance 1e-3 m: ~1 mm under ~1 body
    weight of load; lateral push forces of a few N could reach a few mm.
  Anything >= ~5 mm of geometric overlap is NOT explainable as normal compliance under
  either default and would indicate a real dynamics/solver event or a measurement artifact.

## Contact-force telemetry

MISSING_TELEMETRY_REQUIRES_FUTURE_INSTRUMENTATION — the sim publishes only robot state,
efforts, and object poses (franka_sim.cc:203-231); sim.log (32 lines) contains parser
warnings + MATCHED_MU lines + initial q only; no ContactResults port is connected, no
contact force is logged anywhere in this campaign. Verifying actual penetration depth /
contact force requires future instrumentation (or offline geometric overlap reconstruction
from state_trace.jsonl poses, which IS possible without any rerun).

## lambda_obs availability (question 5)

NOT logged in this campaign. The per-slot obstacle lambda/eta/phi logging exists in the
frozen binary but is env-gated by `SAMPLING_C3_COST_LOG_DIR` (CostLogger :64-90;
obstacle_lcs_contacts.csv writer :110-117 + :2735-2800 incl. per-knot gaptrace
lambda/eta/phi at :2764-2782). run_scene_smoke.sh sets only SAMPLING_C3_OBSTACLE_MODE
(+ scene env file's OBS_BOXES); no COST_LOG_DIR → planner.log has no numbers and no CSVs
exist in pair01. MISSING_TELEMETRY note: a FUTURE frozen-binary replay with
`SAMPLING_C3_COST_LOG_DIR=<dir>` (no source changes) would produce
obstacle_lcs_contacts.csv (slot, d_raw, phi, witness points, normals, lambda0, eta0,
lambda*eta, vn, vt) and the horizon gaptrace (eta_obs, g_pred=eta*dt, lambda per knot) —
exactly the data needed to decide whether the planner ever demanded obstacle force.
