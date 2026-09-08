# C3+ controller cost semantics (frozen xArm6 stack) — source audit 2026-09-08

All line numbers refer to `systems/controllers/sampling_based_c3_controller.cc` (5300 lines) in this worktree unless noted. The C3 core lives in the bazel external `c3+` module (`bazel-oim_c++_anything/external/c3+/core/c3.cc`).

## (a) Local C3 objective — exact expressions

The C3 QP objective is the classic tracking form (c3.cc:137-155):

    J_c3 = sum_{k=0..N} (x_k - x_des_k)' Q_k (x_k - x_des_k)  +  sum_{k=0..N-1} u_k' R_k u_k
           + ADMM augmentation ||z - delta + w||^2_{G_k} and projection-matching U_k

with N = 5, gamma = 1.0 (`sampling_c3plus_options.yaml`). `UpdateCostMatrices` (cc:3405-3432) builds
Q_k = gamma^k * w_Q * diag(q_vector), R_k = gamma^(k+1) * w_R * diag(r_vector); with gamma = 1 the
weights are CONSTANT over the horizon — there is no per-knot ramp and no terminal/final-QP boost in
this campaign's configs (no `final_qp` term exists in sampling_c3plus_options.yaml; the kik_t.yaml
1000-boost belongs to the franka T pipeline, not this stack).

State layout (n_x = 19, 1 object): [ee_pos(0:3) | obj_quat wxyz(3:7) | obj_pos(7:10) | ee_vel(10:13) | obj_ang_vel(13:16) | obj_lin_vel(16:19)].

**Translation cost** (matched_open_table_xarm6_t1, identical in all matched_* and icra dirs):

- Pose-tracking config (active once near goal): q_vector obj-pos entries = [200, 200, 120], scaled by w_Q = 50, so the effective diagonal is Q_xx = Q_yy = 50*200 = **10000**, Q_zz = 50*120 = 6000. Per knot:
  J_trans_k = 10000*(x-x_des)^2 + 10000*(y-y_des)^2 + 6000*(z-z_des)^2.
- Position-tracking config (used while `pose_diff >= 0.5 * cost_switching_threshold_distance`, i.e. object further than 0.25 m from goal — cc:1918-1931, threshold 0.50): q_vector_position obj-pos = [250, 250, 250], w_Q_position = 50 → diagonal 12500 each axis.

Two-phase switch: `crossed_cost_switching_threshold_` latches true once within half the threshold and only resets on a new final goal (cc:1880-1901).

**Orientation cost.** Not a plain quaternion diagonal once the position threshold is crossed. With `use_quaternion_dependent_cost: true` and the threshold crossed, `UpdateCostMatrices` (cc:3434-3487) REPLACES Q block(3:7,3:7) with

    Q_quat = gamma^k * q_quaternion_dependent_weight * ( H + max(0,-lambda_min(H)) I + frac * q_des q_des' )

where H = Hessian of the squared quaternion angle difference between current and desired quaternion,
q_quaternion_dependent_weight = **510**, regularizer_fraction = 0. This is a quaternion-error-based
quadratic: to second order, (x_q - q_des)' H (x_q - q_des) ≈ theta^2, the squared geodesic angle. For a
planar (yaw-only) object, theta = yaw error, so the honest planar-yaw equivalent is
J_rot_k ≈ 510 * (yaw - yaw_des)^2 (NOT multiplied by w_Q; the quat block is overwritten wholesale, not
scaled by 50). Far from goal (position phase) the block is just the diagonal 0.1 * w_Q = 5 per quat
component — effectively negligible orientation drive.

Velocity/EE terms: EE pos 0.01*50=0.5, EE vel 5*50=250, obj ang vel 0.013*50=0.65, obj lin vel 0.05*50=2.5. R = 1*diag(0.01)*... → 0.01 per input.

## (b) OBSTACLE CASE DETERMINATION — verdict: **CASE C** in the frozen baseline (mode-dependent → CASE D only under env overrides that were NOT active in the frozen campaign)

Evidence:
- The nominal per-candidate C3 solve is obstacle-blind: `test_c3_object->Solve(test_state); // PASS 1 (nominal, obstacle-blind)` cc:2337. The optional obstacle-aware PASS 2 block (cc:2339-2457) is guarded by `ObsCfg().inner != kNone || ObsCfg().nonpen` — both DEFAULT OFF (`ObsExtConfig` defaults, cc:186-201; env parsing cc:867-996: inner obstacle Q-injection needs `SAMPLING_C3_INNER_OBS_MODE`, nonpen needs `SAMPLING_C3_OBSTACLE_MODE=qp_halfspace_legacy` or `SAMPLING_C3_OBJ_NONPEN=1`). So the local C3 objective contains NO obstacle term.
- The ranking DOES have an obstacle objective (cc:2515-2536): for every state XX_k of the ranked rollout and every scenario obstacle disc [ox,oy,r],
      J_obs_rank = sum_k sum_obs  w * exp(-d_k / sigma),   d_k = hypot(XX_k(7)-ox, XX_k(8)-oy) - r
  with w = `obstacle_cost_weight` = **5000** and sigma = `obstacle_cost_decay` = **0.04** (scenario_params.yaml; 0 weight on open_table scenes → term vanishes). Note d uses the object CENTER, not the footprint (unlike OIM's boundary-sampled potential).
- Frozen-QP halfspace nonpenetration (prior audit CONFIRMED at current source, cc:2393-2421): when enabled it adds, per obstacle, ONE linear STATE constraint `A(7)=nx, A(8)=ny, lb = n·p_cur + margin - phi` computed once from the MEASURED pose's closest T-FOOTPRINT point — a frozen separating halfspace inside the QP, NOT an LCS complementarity, and NO objective term. The ranking exp cost is NOT disabled in this mode (its guard cc:2522 only checks `!ObsCfg().lcs_contact`), so nonpen mode = constraints + ranking objective. Confirmed.
- `SAMPLING_C3_OBSTACLE_MODE=lcs_contact` (cc:685-858, 1196-1210): appends `n_obs_slots` (default 2) FRICTIONLESS complementarity rows (0 ≤ λ ⟂ φ + Jx ≥ 0 style, via `AugmentLcsWithObstacleContacts`) for the N-closest obstacle discs using the closest-T-footprint witness point — a CONTACT CONSTRAINT inside the LCS, no cost. In this mode NO obstacle objective exists anywhere: the ranking exp term is explicitly skipped (cc:2519-2523 comment + `!ObsCfg().lcs_contact` guard), and startup throws if soft potentials/halfspace are also enabled (cc:962-969). Log line cc:1210 prints `obstacle_cost_active=false obstacle_lcs_contact_active=true`.
- Robot base disc: in icra scenes, obstacle index 7 = `[0.0, 0.0, 0.09]` (robot base, scenario_params.yaml comment "planner-only, upstream Circle") is a plain entry of `scenario_params.obstacles`, so it PARTICIPATES in the ranking exp cost, the repos-path veto, and (if enabled) nonpen/lcs_contact — exactly like the glyph obstacles. In matched_* obstacle scenes there is no base disc entry.
- Obstacles also act through HARD candidate filters (constraint-like, ranking layer): repos PWL path veto DEFAULT ON (cc:2610-2645, cost := 1e12), pusher filter env-gated OFF, plus icra exact convex-polygon SDF override via `SAMPLING_C3_OBS_POLYS` (cc:931-960) which replaces the disc SDF for both ranking clearance and veto geometry.

So: frozen baseline (no SAMPLING_C3_* obstacle env vars) = CASE C — no local-C3 obstacle objective; ranking carries `5000 * exp(-d/0.04)` summed over rollout knots and obstacles, plus the hard path-veto filter.

## (c) Ranking cost J_rank and what the outer loop compares

For each candidate i (index 0 = current EE location, 1..K = sampled contact locations, each solved with its own C3 from its own LCS):

    J_rank(i) = J_c3rollout(i) + travel_cost_per_meter * ||Δxy_EE||    (cc:2474-2478; travel weight = 0 → inert)
              + J_obs_rank(i)                                          (cc:2515-2536; obstacle scenes only, not lcs_contact)
              + finished_reposition_cost (1e9) on the just-finished repos target slot (cc:2538-2543)
              [+ route dV credit / lexicographic re-selection — env-gated, OFF frozen]
              [1e12 overwrite from path-veto / pusher-filter / empty-slot exclusion]

J_c3rollout uses `cost_type: 5` = kSimImpedanceObjectCostOnly (progress_params*.yaml; CalcCost cc:1573-1806): the candidate's planned trajectory is re-simulated through the cost-LCS under a PD+feedforward law (Kp=[100,100,50], Kd=[0.5,0.5,0.5]), then scored with the SAME Q_k as the local C3 but with the EE-position block, EE-velocity block, and R zeroed — i.e. only object orientation/position/velocity errors count, over k = 0..N (N+1 knots including the LCS-rolled terminal state).

Outer-loop comparison (cc:2646-2662, 2823-2975): `best_other_cost = min_{i>=1} J_rank(i)` vs `curr_cost = J_rank(0)`. With `use_relative_hysteresis: true`:
- C3 → reposition when unproductive (kConfigCostDrop: current config cost must drop 1% over 180 loops) AND `curr_cost > best_other_cost + 0.4*curr_cost` (0.6 in position phase);
- repos → repos retarget only if new candidate beats pursued target by 30% (10% position);
- repos → C3 when `curr_cost(0) < best_other_cost + hyst` with frac 0.9 (0.5 position).
The unproductivity metric itself is `curr_pos_and_rot_cost = e' Q[0].block(3:10,3:10) e` at the MEASURED state vs the FINAL goal (cc:4198-4214).

## (d) Time-varying weights in the CONTROLLER?

None in the frozen stack. The only per-knot variation mechanism is `gamma^k` discounting (cc:3415-3418), and gamma = 1.0 in every campaign yaml, so Q_k and R_k are constant across the horizon and across control steps (except the state-dependent quaternion block and the one-time position→pose config switch at 0.25 m). The ramp `min(1 + 0.005k, 30)` (Q_RAMP_PER_STEP/Q_RAMP_MAX, `tools/scene_smoke/postprocess_run.py:61,187`) is the OIM evaluation-side recompute and is **evaluation_only** — nothing resembling w_p(k) exists in the controller.

## Per-scene constants

| family | Q obj-pos diag (eff.) | quat weight | mu (ee-gnd, ee-obj, obj-gnd) | obstacle w / decay | obstacles |
|---|---|---|---|---|---|
| matched_open_table_xarm6_t1..t5 | 10000,10000,6000 (pose) / 12500 (pos phase) | 510 | 0.4165 / 1 / 0.4615 | 0 / 0.04 (inert) | none |
| matched_single_obstacle, slalom, shelf_gap, ycb_clutter (xarm6 t1..t5) | same | 510 | same | 5000 / 0.04 | scene discs |
| anything_icra_c_matched_xarm6_t1..t5 | same (identical q_vector) | 510 | 0.4165 / 1 / **0.3** (obj-gnd) | 5000 / 0.04 | 7 glyph discs (polys via SAMPLING_C3_OBS_POLYS override) + base disc [0,0,0.09] |

## Reconstructability

An offline script evaluating local-C3 semantics at the MEASURED state x(k) each control step needs only:
- **J_trans(k)** = 10000*(x_obj - x_goal)^2 + 10000*(y_obj - y_goal)^2 + 6000*(z_obj - z_goal)^2 when ||p_obj-p_goal|| < 0.25 m (pose config), else 12500*Σ(axis errors²) (position config). Constants: w_Q=50, q_pos=[200,200,120] / [250,250,250], threshold = 0.5*0.50 m with latching.
- **J_rot(k)** = 510 * theta_err(k)^2 with theta_err = geodesic quaternion angle (= |wrap(yaw - yaw_goal)| for planar motion), valid only after the 0.25 m latch; before that J_rot = 5 * Σ(quat component errors)^2 ≈ negligible.
- **J_obs_rank(k)** = Σ_obstacles 5000 * exp(-(hypot(x_obj-ox, y_obj-oy) - r)/0.04) — object CENTER vs disc from `scenario_params.yaml` (icra: use the exact convex-polygon signed distance from `results_icra_port/obstacles/obs_polys_env.txt` for glyphs 0-6, disc for the base at index 7). Note the controller sums this over the N+1 predicted rollout knots per candidate; the per-step measured-state version is the k=0 slice.

## Surprises / notes
- Ranking obstacle clearance uses the object CENTER minus disc radius, not the T footprint (footprint is used only for nonpen/lcs_contact witnesses and the route/veto geometry) — the exp potential therefore under-penalizes near-edge grazing relative to OIM's boundary-sampled version.
- `travel_cost_per_meter = 0`: reposition distance costs nothing in J_rank; reposition time enters selection only through hysteresis (or transaction_v1, env-gated OFF).
- The quaternion-dependent orientation weight (510) bypasses w_Q entirely — it is not 50*something.
- CalcCost scores k=0..N including an extra LCS-rolled terminal knot, so J_rank has N+1 = 6 state terms.
