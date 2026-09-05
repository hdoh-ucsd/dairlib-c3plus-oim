# Route prediction vs execution fidelity — forensics & correction report

**Date:** 2026-09-05/06. **Branch:** `c3plus-channel-route-v1` from `5acb731e3`. **Frozen and verified untouched:** LCS obstacle contact, geometry, Q/R/ADMM/horizon, margins, force limits, tolerances, route shape, channel selection. **Results:** `results/c3plus_route_prediction_fidelity/` + offline audits over `c3plus_route_progress_selection/`.

## 1. Starting point

Prior verdict: route progress was selected (72–96% of decisions, predicting 2–3 cm per plan) but not realized (0.058 vs 0.059 m/min). This study answers *why*.

## 2. Route-value validation (§2 of the brief) — ΔV is CORRECT

Per selected candidate across all five lexicographic draws: median (ΔV − raw route-tangent displacement of the predicted terminal) = **−1.6 to −5.0 mm** — ΔV slightly *under*-reports raw tangential displacement (the cross-track term), never inflates it. Median predicted raw displacement 2.8–3.8 cm accompanies median ΔV 1.7–3.0 cm. **No projection artifact: verdict A ("ΔV artificially created by route projection") is excluded.** Safe-window projection, jump caps, and rejection logging were active throughout.

## 3–4. Four layers and fidelity ratios (matched 0.5 s windows = the rollout horizon N·dt)

- Stage 1 (inner QP): not separately instrumented in this pass (the ranking rollout, Stage 2, is the quantity selection uses; QP vs rank consistency was validated structurally in the v2 campaign — same augmented LCS).
- Stage 2 (ranking rollout): predicts **median 2.8–3.8 cm object displacement per 0.5 s** when its candidates are selected.
- Stage 3 (command-level execution model): **does not exist in the codebase** — stated explicitly; the OSC has no object-motion predictor.
- Stage 4 (actual, same 0.5 s windows): **median 0.0000 m; ρ_tangent median = 0.0.**

Split by execution context (the decisive cut):

| Context of the selected promise | share of ΔV-selected cycles | median predicted | median realized (same window) | fidelity |
|---|---:|---:|---:|---:|
| pusher already pushing (C3 mode) | 26% | 7.7 mm | 4.7 mm | **ρ ≈ 0.61** |
| pusher traveling (reposition mode) | **74%** | **26.8 mm** | **0.0 mm** | **0** |

And across *all* sustained-push windows (146k): the object moves >5 mm in only **2%** of 0.5 s windows — push production itself is sparse.

## 5–8. State mismatch, rollout model, OSC, acquisition (audited)

The ranking rollout (`kSimImpedanceObjectCostOnly` → `SimulatePDControlWithLCS`, fine-dt LCP): **starts the pusher teleported to the candidate contact pose** with contact available on the first step; models **no contact acquisition time, no contact loss, no arm/OSC, no joint limits, no command latency**; PD gains `Kp_for_cost = [100,100,50]` track the planned EE far more tightly than the real arm; friction/obstacle contacts/timestep otherwise match the sim (same plant constants, same augmented LCS — verified in v2). So for any candidate the EE has not yet reached, the prediction is unconditionally optimistic by construction; and my lexicographic selector was, until this fix, **re-issuing those unreachable promises every cycle during reposition** — churn on top of optimism. A per-joint OSC audit (Stage-3 instrumentation) was not built in this pass; the in-contact ρ≈0.61 bounds what it could explain.

## 9–13. Force/wrench, micro-tests, replay, time-scale, contact-mode

- Time-scale (§12): predicted 2–3 cm/0.5 s ⇒ 3–7 cm/s implied transport; realized campaign medians ≈ 0.001 cm/s equivalent — but normalized per *active productive push window*, realized reaches 0.94 cm/s vs 1.5 cm/s predicted (the ρ≈0.61). **The headline mismatch is duty cycle, not in-contact physics.**
- Contact-mode (§13): the rollout's promise implicitly assumes contact from t=0 (sticking-capable); reality spends most windows unengaged. In-contact windows realize the right *direction* with ~0.6 gain (class D calibration, not class E dynamics error, as the primary in-contact story).
- Deterministic micro-tests (§10) and state-restore single-candidate replay (§11) were **not built** — the sim stack has no state save/restore; documented as the main gap. The matched-window population analysis (thousands of natural "replays") substitutes for the A-variant; B (ideal Cartesian pusher) and C (forced contact) remain future work.
- Reference regime measurement: the two historical successful runs sustained **≈0.5 m/min** realized transport (natural rear-contact forward pushing) — 10× the all-arm median — proving the plant/controller *can* transport at prediction-consistent rates when contact is continuous and aligned.

## 14–16. Corrections implemented (smallest set, per §17)

1. **Commit during travel** — lexicographic re-selection now runs only in push mode; during reposition the pursued target is held (kills the 74% unrealizable-promise decisions).
2. **Yaw as a genuine secondary objective** (§16 rule, all four steps): eligibility requires terminal |yaw error| ≤ current + 0.15 rad; near-best route set (η = 0.20); tracking cost selects within it.
3. **calibrated_v1** (`SAMPLING_C3_ROUTE_PREDICTION_MODE`): eligibility and transaction rates use ΔV × 0.61 (the measured in-contact fidelity; class-resolved online learning left as shadow-model future work since the two measured classes are 0.61 and 0).

## Live ablation (3 draws per arm, 2-lane, ~290–350 s sim)

| Arm (plain language) | Success | Final XY median | Realized route progress |
|---|---:|---:|---:|
| Route + old ΔV selection (previous campaign) | 0/5 | 0.44 | 0.058 m/min |
| Route + committed selection + yaw bound | 0/3 | 0.47 | 0.035 m/min |
| Same + calibrated ΔV | 0/3 | 0.41 | 0.055 m/min |

**The corrections did not raise realized transport.** With hindsight this is the expected result of the audit itself: the corrections fix *selection honesty*, but the binding constraint revealed by the window statistics is **push production throughput** — the executed pushes (any selection policy) move the object >5 mm in only ~2% of windows, an all-campaign ceiling of ~0.05 m/min except when the natural rear-contact forward-push regime engages (0.5 m/min). No selection policy can choose its way past a production ceiling.

## Answers (§24)

1. **ΔV correct?** Yes (−2 to −5 mm conservative vs raw tangent).
2. **Inner QP predicts lateral motion?** Yes (shared augmented LCS; rollout carries it).
3. **Ranking preserves it?** Yes — 2.8–3.8 cm/0.5 s predicted.
4. **Does the EE execute the predicted trajectory?** Not instrumented per-joint; bounded: even successful contact windows deliver 0.61 of promise.
5. **Is contact acquired?** In only ~26% of promise windows was the pusher even pushing; across all push windows productive contact is ~2%.
6. **Maintained for the predicted duration?** No — this is the dominant loss.
7. **Predicted tangential force realized?** Partially (0.61 in contact); mostly never applied (no contact).
8. **Where does progress disappear?** Between selection and physical contact (74% of promises), then a 0.61 haircut in contact.
9. **Lateral-specific?** Partially — forward rear-contact pushing reaches 0.5 m/min; lateral legs never sustain contact long enough to measure a clean lateral ρ.
10/11. **Forced-contact / ideal-pusher replay?** Not built (no sim state restore) — the top remaining instrumentation gap.
12. **Corrected rollout improves correlation?** The commitment fix removes the zero-realization promise class by construction (predictions are now only issued when realizable), but did not raise transport.
13. **Calibrated ΔV improves realized progress?** No (0.055 vs 0.058).
14. **Yaw bound prevents regression?** Partially — two of six corrected draws still ended with large yaw (2.6–2.9), from pre-existing rotation drift, not selection.
15. **Transaction reposition after correction?** Not re-enabled — prediction fidelity for *new* contacts remains unproven, per the spec's own gate.

## Verdict

**G. MULTIPLE CAUSES — RANKED WITH MEASURED EVIDENCE:**
1. **F — active-contact duration vs prediction window**: 74% of selected route promises were issued while the pusher was traveling (realized exactly 0), and only ~2% of all push windows produce >5 mm of motion.
2. **D — contact acquisition/maintenance**: acquisition is unmodeled in the rollout (pusher teleported to contact) and unmeasured in execution; productive contact is the scarce resource.
3. **C — mild rollout optimism in contact**: realized/predicted ≈ 0.61 even when pushing (PD-rollout tracks tighter than the real arm).
4. Excluded: A (ΔV projection — verified correct), E as primary (in-contact direction agrees; it's a gain, not a mode error).

**"The selected candidate predicted route progress at the sample-ranking forward rollout, but that progress disappeared between candidate selection and physical pusher contact, because 74% of the promise windows had no pusher–object contact at all (the rollout starts the pusher teleported to the contact it has not yet reached), and the remaining in-contact windows delivered only 61% of the promised motion."**

**Consequence for the roadmap:** further selection/scoring work is exhausted; the next leverage is *push production* — contact-acquisition modeling in the rollout (§17-B), sim state save/restore for the replay experiments (§11), and increasing sustained-contact throughput (contact-rich execution or longer committed pushes) so that the 0.5 m/min regime measured in successful runs becomes the norm rather than the lottery.
