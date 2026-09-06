# C3+ xArm6 plant & execution fidelity report (Agent B)

**Date:** 2026-09-05. **Branch:** `audit/xarm6-plant-fidelity`, base `0abc0eb7c` (c3plus-channel-route-v1 head; umbrella repo HEAD at audit launch: `4316b1ea`).
**Outputs:** `results/parallel_c3plus_audit/agent_b_xarm6_plant/` (this worktree).
**No candidate-cost, ranking, or outer-loop code was modified.** No model/friction/gain tuning was performed (§13 honored). No new source instrumentation was required — the latent passive instruments are inventoried in `DATA_AVAILABILITY.md`.

## Scope note (important)

Two stacks exist under the "xArm6/C3+" umbrella:
1. the **xArm6 oim_t open-table stack** (`examples/sampling_c3/oim_t/`) — the MJX-port C++ equivalent; audited statically in `plant_model_comparison.md` + `command_pipeline.md`;
2. the **Franka sampling-C3 stack**, which is what actually executes the shelf/obstacle campaigns audited empirically here (confirmed: every campaign `osc.log` boots `sampling_c3_franka_osc_controller`).

## §2 Three-model inventory — headline differences (`plant_model_comparison.md/.csv`)

- The DEFAULT xArm6 planner LCS (`reduced_exact_t`) is **frictionless, gravity-free, table-free**, with a fictitious 1.0 kg point pusher and no object geometry (one linearized contact plane per sampled point/normal). The ranking layer is the SAME solve, not a separate simulator.
- The opt-in full spatial LCS mis-assumes **object-table μ = 0.4615 vs the sim's 0.3** and pusher-table 1.0 vs 0.4615.
- **Arm kinematics, reposition lift, reach filtering, and tilt handling are absent from every planning model** — they are post-C3 execution-layer filters only.
- Rate structure: sim 2 ms, OSC 500 Hz, plan knots 50 ms, **object feedback only 20 Hz** (up to 50 ms pose age = one full plan step).
- The Franka ranking rollout (`kSimImpedanceObjectCostOnly`) **teleports the pusher to the candidate contact** and models no acquisition time, no arm, no latency (established in the 2026-09-05/06 fidelity study; reconfirmed as the structural cause below).

## §3–4 Command pipeline (`command_pipeline.md`)

Full trace with file:line, frames, units, rates. Key confirmed facts:
- On the live path the **C3 input force u is never sent anywhere** — execution is a pure position servo on the knot-2 predicted pusher state, with a direction-preserving 0.01 m Cartesian step limiter and a 0.002 m minimum inward-normal step.
- Only true torque saturation is the velocity-servo bridge (vel clamp ±0.5 rad/s, effort clamp, gravity outside the clamp — MuJoCo-conformant ordering).
- Defects worth tickets (none load-bearing for the plateau): no message-age gate (stalled planner = silent hold), `actuator_delay` yaml silently ignored, OSC OSQP failure unchecked, λ_e sign on the full-plan force path unverified.

## §6/§9 Shelf plateau — measured (`shelf_plateau_divergence.md`, `contact_acquisition.csv`)

Populations: p5-fix validation (draw0 SUCCESS 0.018 m vs draw1/2 plateau, same settings) + the 2026-09-06 comparison arms (9 draws total).

- **Contact acquisition, not transport, separates success from failure**: productive-window fraction 0.131 (success) vs 0.017–0.049 (all failures). Raw contact fraction does NOT separate (two failures are parked-on-object with proxy-contact ≈0.5 and 0.02 mm/s transport).
- **Earliest divergence is at corridor-gap entry, in object yaw**: the success arrives stem-aligned (+3.09 rad) and transits in one 10 s, 22.9 mm/s burst; the plateaus arrive ~20–25° misaligned (opposite sign) and contact fraction drops to ~0.000 permanently from onset (first divergent bin t=80–90 s). Post-onset: reposition storm (45–75 events) with `Unsuccessful sample buffer overflow` on every cycle — **candidate-pool starvation during re-acquisition**.
- **Obstacle lambda post-plateau = 0.0000 in every draw** — the planner predicts no obstacle interaction after stall; the object parks at the mouth and the pusher never re-engages.
- **OSC exonerated**: zero tracking/limit/saturation warnings in every osc.log/sim.log across all 9 draws.

## §7 Prediction-layer comparison (`prediction_vs_execution.csv`)

Median ρ(realized/predicted route progress over matched 0.5 s windows) ≈ **0.00** in the only arm that logs predictions (route full stack), in contact and out — consistent with the 2026-09-05 shared-dishonest-predictor verdict. From the prior fidelity study (same lineage): 74% of promises issued mid-travel realize exactly 0; in-contact windows realize **ρ ≈ 0.61**.

## §8/§10 Micro-tests and repeated-miss classes (`normal_lateral_microtest.csv`)

Deterministic scripted-pusher tests (prior study, verified constants match sim: combined μ 0.4165/0.4615): normal 3.84 cm and lateral **5.15 cm** object transport per 10 cm pusher travel, 5/5 identical — **lateral pushing is stronger, not weaker**; B5 (sticking/sliding mismatch) excluded as primary. Shelf-south transport demonstrated in-vivo by the success draw's 22.9 mm/s corridor burst. Dominant repeated-miss class = **NO_CONTACT**; secondary = mild in-contact optimism (0.61 gain); OSC_TRACKING_FAILURE not observed.

## §11 Replay (`candidate_replay_results.csv`)

State-restore replay remains unbuilt (no sim save/restore). Population-matched windows + scripted-pusher tests cover variants A/B/C; the post-P5 committed-selection ablation covers D.

## §12 Workspace & stability (`workspace_abort_analysis.json`, `topple_analysis.json`)

No aborts and no topples in any audited shelf/obstacle draw. The chase-out-of-reach aborts belong to the jack campaign; the ranking rollout cannot predict recoverability loss because it contains no arm/reach model (structural, documented). B7/B8 are not shelf-plateau mechanisms.

## Verdict

**MULTIPLE_CAUSES_RANKED:**
1. **B4_CONTACT_ACQUISITION_FAILURE** (dominant) — the rollout starts the pusher teleported to contact; execution must acquire it through lift/traverse/IK/swept-clearance layers absent from every planning model. At the shelf corridor mouth, with the object ~20° yaw-misaligned, acquisition fails permanently (candidate-pool starvation, reposition storm).
2. **B6_INSUFFICIENT_ACTIVE_CONTACT_DURATION** — productive-contact windows are ~2–5% of all windows in failures vs 13% in the success; transport is duty-cycle-limited, not force-limited.
3. **B2_C3_TO_RANKING_MODEL_MISMATCH** (contributing, bounded) — in-contact realization is 0.61 of promise (rollout PD tracks tighter than the real arm); plus the enumerated static model gaps (frictionless default LCS, μ-table mismatch in the full LCS, 20 Hz object feedback).
4. Excluded as primary: B1 (ΔV projection verified correct), B3/OSC tracking (zero anomalies, 0.61 bound), B5 (lateral physics stronger), B7/B8 (no events).

**Questions handed off:** see `agent_b_handoff.json`.

---

**The selected candidate predicted motion at the ranking forward rollout (and inner C3 solve), but that motion disappeared between candidate selection and physical pusher–object contact, because the rollout starts the pusher teleported onto a contact the real arm has not acquired — and at the shelf corridor mouth, yaw-misaligned arrival makes re-acquisition fail permanently (candidate-pool starvation and reposition storms), while the windows that do achieve contact deliver only ~0.61 of the promised motion.**
