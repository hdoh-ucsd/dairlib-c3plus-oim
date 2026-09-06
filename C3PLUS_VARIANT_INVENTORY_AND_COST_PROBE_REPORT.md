# C3+ Variant Inventory and Cost Probe

Date: 2026-09-05. No controller changes were made for this probe; no live L1 cost was implemented; no weights were tuned; no success-rate campaign was run.
Data: `results/c3plus_variant_cost_probe/` (registry, probe states, candidate banks, replays, figures, manifest).

## 1. Repository and commit provenance

Worktree `/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/obstacle-lcs-contact`, branch `c3plus-channel-route-v1`, HEAD `43d5efb96` (clean), remote `standalone` = hdoh-ucsd/dairlib-c3plus-oim, C3 dependency DAIRLab/c3 @ `5c08cb2e14b1`. Build: `//systems/controllers:sampling_c3_controller` + the three `//examples/sampling_c3:franka_*` executables. Active config: per-demo `sampling_c3plus_options.yaml` (w_Q=50, q=[.01×3 EE | .1×4 quat | 200/200/120 obj pos | 5×3 EE vel | .013×3 ω | .05×3 v], R=1·diag(.01), N=5), progress detector drop 0.5 over 16 C3 loops. **All historical mechanisms live on this single branch and are env-reachable at HEAD** — no other worktree is needed. Landmark commits: inner-QP obstacle costs + halfspace `840c8c618`; lcs_contact `2440d28c6`; box-SDF v2 `589be6bbf`; transaction_v1 `b7364606e`; channel_v1 + transaction_v1_1 `e17cf46ea`; approach fix `78195e85f`; delta-V selection `5acb731e3`; fidelity fixes `e9426f61f`; P5 fixes `5fbec41ef` + `43d5efb96`.

## 2-3. Variant registry and layer classification

`variant_registry/variant_registry.csv` (30 entries) and `variant_stack_matrix.csv`. 23 env switches were found; per-switch defaults, parse and use sites, and interactions are in the CSV. Behavioral-vs-logging classification: `SAMPLING_C3_COST_LOG_DIR`, `SWEPT_CHECK`, `OBS_TRUST`, and `ROUTE_SELECTION_MODE=shadow_delta_v` are logging-only; everything else is behavioral. Key interaction facts: all `ROUTE_*` sub-switches are no-ops without `ROUTE_MODE=channel_v1`; `calibrated_v1` matters only in selection mode 2; `lcs_contact` hard-conflicts with the soft potentials (throw at cc:785); env parsing is one-shot and silently ignores typos.

## 4. Historical obstacle formulations (verified from source; not rerun)

- **H0 frozen baseline**: obstacle-blind inner solve + center-to-disc exponential ranking penalty — reachable as "no OBSTACLE_MODE, INNER_OBS_MODE unset". Confirmed.
- **H1 reciprocal-square**: `INNER_OBS_MODE=inverse_square_psd` PSD potential in the inner QP + matching recip ranking shape (`RANK_OBS_MODE=inverse_square`). Confirmed.
- **H2 projected-exponential**: `INNER_OBS_MODE=exponential_psd` + exact exponential ranking. Confirmed.
- **H3 qp_halfspace_legacy**: frozen closest-footprint separating halfspace in the QP; exponential ranking cost stays active. Confirmed.
- **H4 lcs_contact**: closest-footprint frictionless contact in the LCS (λ_obs/η_obs, lever-arm yaw coupling via (r×n)_z, same contact in solve/projection/rollout), **no obstacle objective** — verified: both cost paths guarded (cc:2133, cc:2317); `J_rank_obstacle_total` is a passive diagnostic column (never added to costs; it inflates the logged `diff` column in lcs_contact mode — logging artifact only).
Existing results reused for the known-result column are labeled with their commits in §17.

## 5. Current canonical variants

V0 plain / V1 route / V2 route+ΔV(calibrated) / V3 transaction_v1_1 only / V4 route+ΔV+txn — exact env settings in `configuration_diff_matrix.csv`. **Audit outcome**: V0→V1, V0→V3, V2→V4 are clean single-factor deltas. V1→V2 differs by TWO switches (SELECTION_MODE + PREDICTION_MODE) by design of the fidelity arc — flagged, treated as one package. **V5 (approach fix) is NOT an independent factor**: it is unconditional inside channel_v1 (cc:1966-1982, no opt-out), so it cannot be compared separately without a code change — reported, not implemented.

## 6-9. Exact objectives, reposition scores, hard filters

Full equations in `variant_registry/exact_objectives.md`. Highlights: the ranking cost (cost_type 5, kSimImpedanceObjectCostOnly) is an object-only quadratic tracking cost of a PD-impedance forward simulation — effective per-knot weights quat 5, obj pos 10000/10000/6000, ω 0.65, v 2.5; travel term weight is **0**; no terminal weighting. Hard filters (never cost terms): candidate-object collision, P5 swept-path veto (+4 cm escape zone, placeholder exclusion), workspace radius aborts, repos timeout 1100 loops, finished-reposition 1e9 one-loop marker, no-progress detector 0.5/16. There is **no topple filter and no contact-acquisition requirement**; arrival is a 5 mm EE-position check.

## 7. Hidden interactions found (configuration audit)

1. **Placeholder hazard is unguarded on obstacle-free scenes**: the (0,0,0) empty-slot exclusion lives inside the path-veto block, which is gated on obstacles being present — on open_table the placeholder can still win the argmin (observed: S0 draw0 targeted the origin at t=2.2 s and the run died at t=7.3 s on the inner-radius abort). Registered as a defect; not fixed during this probe.
2. The stale-cost buffer candidate is argmin-eligible in `current` mode but skipped in transaction modes — a two-factor difference between V0 and V3 beyond the exit gate itself (inherited from transaction_v1's design; documented).
3. `J_rank_obstacle_total` diff-column artifact in lcs_contact mode (see §4).
No arm-to-arm success-threshold differences; P5 stayed ON in every arm.

## 10-11. Probe states and candidate banks

- **S0** open_table reposition churn (t=9.0 s of a run that later SUCCEEDED at 86.5 s; 46 % reposition in the window). Object (0.636, 0.063) yaw 2.07.
- **S1** single_obstacle north-face fixed point (real plateau run, t=119.7 s). Object (0.534, 0.098) yaw −1.81, goal south of the obstacle, yaw partially solved.
- **S2** shelf_gap corridor plateau (real post-P5 run, t=150.0 s). Object (0.532, −0.011) in the corridor, pushing south, zero progress.
Each state: full snapshot JSON (object pose from the sim trace, controller mode, sub-goal, all logged candidates with predicted terminal states and cost components, obstacle contacts). Banks: 17 deterministic candidates per state (13 dense face contacts on every reachable T face + the logged controller candidates + current contact), identical across every scoring comparison. Limitation (by construction of a no-change probe): bank candidates have no controller rollouts, so Probe A covers the logged candidates; the bank feeds the replay ground truth.

## 12-13. Score comparison and deterministic replay

Replay rig: pydrake scripted kinematic pusher, exact object SDF + exact static obstacle SDFs, 2.0 s push at 0.05 m/s, 3 reps (deterministic, spread ~0). CSVs: `candidate_rank_comparison.csv`, `candidate_rank_correlation.csv`, `actual_replay_performance.csv`, `predicted_vs_actual_candidate_rank.csv`, `cost_component_breakdown.csv`.

Replay ground truth (route progress per 10 cm push):
- **S0**: many productive contacts (stem-east up to +37 mm; stem-tip +26 mm); logged candidates: two counterproductive, one productive (−17 mm to goal).
- **S1**: only stem-tip (+8 mm, pushing the object east AROUND the obstacle) is productive; every other contact including all logged ones is neutral-to-negative. The best available action here is a small circumnavigation step, and it was never in the drawn candidate set. Moreover at this cycle ALL three alternates were hard-filtered to 1e12 — the pool was empty, the controller pinned to its current contact.
- **S2**: productive contacts EXIST — stem-tip +24 mm south out of the corridor, stem-west +22 mm — while the candidate the controller actually pursued moved the object 25 mm BACKWARD. Two crossbar contacts topple the T (90°) — usable as intentionally-poor controls, and a warning that no topple filter exists.

## 14+16. Cost failure vs prediction failure — the decisive result

Winner-per-score at each state (all scores computed on the SAME logged rollouts):
| State | J_rank | ΔV_cal | R_rate | L1 task | L1 route | L1 txn | Actual best (replayed) |
|---|---|---|---|---|---|---|---|
| S0 | logged_0 | logged_0 | logged_0 | logged_0 | logged_0 | logged_0 | **logged_2** |
| S1 | logged_0 | logged_0 | logged_0 | logged_0 | logged_0 | logged_0 | logged_0 ✓ |
| S2 | logged_1 | logged_1 | logged_1 | logged_1 | logged_1 | logged_0 | (only logged_2 replayable; it was counterproductive) |

At S0 the actually-best candidate is ranked LAST by the current cost **and by every shadow L1 form** — identically, because every score consumes the same predicted terminal state, and the prediction is wrong. At S2 predictions are FLAT (max predicted displacement 9 mm, tracking-cost spread 2 % across candidates) — all scores rank noise. The predicted-vs-actual displacement figure shows predictions of up to 340 mm against realized motion under 80 mm. **The scores cannot be distinguished by quality because they are all downstream of the same dishonest predictor.** This reproduces the fidelity report's in-contact ρ≈0.61 / traveling-promise≈0 findings at the single-state level.

Decision-table classification per state:
- S0: CASE B (all scores share dishonest predictions), self-healing via outer-loop retries (the run succeeded).
- S1: CASE D/generation — the one productive contact was never drawn AND all drawn alternates were hard-filtered; the fixed point is enforced by an empty feasible pool, not by mis-ranking.
- S2: CASE B + partial C — predictions flat; productive contacts exist in the dense bank but weren't in the drawn set at this cycle; the pursued contact was actively counterproductive.

## 16. Short serial online validation (sanity check, NOT success rates; one 300 s draw per arm)

single_obstacle: V0 plain 0.056 m (near-tight, best) | V1 route 0.437 | V2 route+ΔV 0.425 (died t=37 s) | V3 txn 0.421 | V4 full stack 0.413 (died t=13 s). shelf_gap: V0 0.482 (died t=91 s) | V2 0.426 | V4 0.491. Consistent with the 26-run campaign: plain beats every route/selection variant; the early deaths are the known abort/topple lottery classes.

## 17. Variant summary table

| Variant | Status | What changes | Cost used | Known result | Main failure |
|---|---|---|---|---|---|
| H0 frozen baseline | HISTORICAL | obstacle-blind solve + exp ranking penalty | quadratic tracking + exp obstacle | c3ab ablation era (14d7991b6) | obstacle penetration risk |
| H1 reciprocal-square | HISTORICAL | PSD recip inner-QP potential | + recip ranking | 840c8c618 era | soft, tunable, no contact realism |
| H2 projected-exponential | HISTORICAL | PSD exp inner-QP potential | + exp ranking | 840c8c618 era | same class |
| H3 qp_halfspace_legacy | HISTORICAL (env-reachable) | frozen halfspace in QP | exp ranking active | pre-2440d28c6 | +0.027 m conservatism; margin+disc |
| H4 lcs_contact plain (V0) | CURRENT | contact in LCS, no obstacle objective | object tracking only | best arm everywhere (0.301 campaign avg; 0.056 here) | P3 plateaus, abort/topple lottery |
| V1 route carrot | CURRENT | channel_v1 reference | current tracking | 0.41-0.44 | carrot hurts; approach fix inseparable |
| V2 route+ΔV | CURRENT | + lexicographic ΔV/T selection (calibrated) | ΔV primary | 0.42-0.43 | predictions unrealizable |
| V3 transaction_v1_1 | CURRENT | reposition exit gating + timeout | current tracking | first-ever success in its arc; 0.42 here | single-draw lottery |
| V4 full stack | CURRENT | route+ΔV+txn | ΔV + txn | 0.41-0.49 | compounded, worst aborts |
| post-P5 current (=V0 @ HEAD) | CURRENT | + veto/timeout/escape/placeholder | unchanged | shelf deadlock eliminated; first shelf+open_table successes | plateau/abort/topple remain |
| L1-A/B/C | SHADOW ONLY | offline scores on logged rollouts | L1 norms, normalized | mis-rank identically to J_rank at S0/S2 | same dishonest inputs |
| delta_v_weighted | DEPRECATED (pilot) | additive dV credit | J_rank − 20000·dV | pilot only | unvalidated |

## 18. Layer attribution (no mixed conclusions)

- S1 single_obstacle fixed point → **REFERENCE/CANDIDATE GENERATION** (productive circumnavigation contact never drawn; pool emptied by hard filters).
- S2 shelf plateau → **PREDICTION** (flat rollouts make selection noise) with candidate-generation contribution; push-production physics is NOT the blocker at the contact level (stem-tip replays +24 mm).
- S0 churn → **PREDICTION**, recoverable by the outer loop's retry lottery.
- Open_table draw0 abort → **RECOVERY STATE MACHINE / WORKSPACE** (placeholder unguarded on obstacle-free scenes — defect logged).
- Shelf deadlock (already fixed) → REPOSITION EXECUTION. Topples in replay → STABILITY (no topple filter exists).

## Final decision gate → PRIMARY VERDICT

**B, scene-modulated by E: COST VARIANTS DIFFER, BUT NONE PREDICTS ACTUAL ACTION QUALITY** — every existing and shadow score consumes the same rollout predictions, and where those predictions are wrong (S0) or flat (S2) all scores fail together and identically; where they are honest (S1) all scores already agree with reality. The scenes differ in which layer binds (E): S1 is candidate generation/filter starvation, S2 is prediction flatness, S0 is prediction error healed by retries.

**Gate outcome: do NOT implement a live L1 cost.** The shadow L1 scores did not rank actually-productive candidates above unproductive ones anywhere the current cost failed to — they inverted the actual order at S0 exactly as J_rank did. Per the gate, the justified next work is prediction/execution fidelity and candidate generation, not cost-function form: (1) make rollout predictions honest at plateau states (the S2 flat-landscape mechanism), and (2) ensure route-forward contacts (stem-tip class) actually enter the candidate pool at S1/S2-type states.
