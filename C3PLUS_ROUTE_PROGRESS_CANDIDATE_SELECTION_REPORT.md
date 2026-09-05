# Route-progress-aware candidate & reposition selection — report

**Date:** 2026-09-05. **Branch:** `c3plus-channel-route-v1` (on top of the channel_v1 supervisor). **Untouched, verified:** LCS obstacle contact, obstacle geometry, Q/R/ADMM/horizon, channel construction & latching, success tolerances; no obstacle proximity cost anywhere. **Results:** `results/c3plus_route_progress_selection/` (+ retro-shadow over the recorded `c3plus_channel_route_v1` draws).

## What was built

Three selection modes via `SAMPLING_C3_ROUTE_SELECTION_MODE` (default `current` = unchanged):
- **shadow_delta_v** — computes/logs per-candidate route progress, never alters selection.
- **delta_v_lexicographic** — when a bypass/corridor channel is latched: hard filters first, then primary criterion `R_i = ΔV_i / (T_repos_i + T_push + ε)` (current contact at `T_repos = 0`; new contacts rejected below 2 mm ΔV), secondary tie-break within a 5% R band by the existing tracking rank. DIRECT mode is untouched. The same R comparisons drive the transaction_v1_1 reposition gates (leave C3 only for a real rate advantage; return when staying is at least as good).
- **delta_v_weighted** — the stage-2 weighted pilot, kept behind a flag, not run (the spec's lexicographic-first rule; my earlier weighted attempt was superseded before producing data).

Route value uses the spec definition `V = remaining arc length + 1.0·d_perp` with **safe segment-window projection** (only the confirmed segment +2, monotone confirmation, 0.30 m jump cap, rejections logged) — a candidate north of the obstacle cannot steal a south-segment value. ΔV comes from the ranking rollout's terminal object pose. New log: `candidate_route_progress.csv` (event, candidate, ΔV, V_pred, V_current, terminal xy, rejection reason).

## Shadow study (recorded route_only draws, single_obstacle; the §5 gate)

| Question | Answer |
|---|---|
| Did positive-ΔV candidates exist? | **Yes** — in 11–99% of events per draw; never absent for a whole run (so selection integration was justified; the "stop" condition did not trigger) |
| Did the old ranking select them? | Erratically — selected-positive rates ranged 7%–97%; the two worst draws chose zero/negative-ΔV candidates most of the time (medians −0.356 and 0.000) |
| Was a better-ΔV candidate available when it mattered? | Yes — in up to 57% of events another candidate predicted ≥5 mm more route progress than the winner |
| Did lottery successes ride high-ΔV contacts? | Yes — the successful draws are exactly the high selected-positive-rate ones (draw0: 97% positive, median +0.109) |

## Live experiment (13 runs, 2-lane; ~280–340 s sim each; plain-language table)

| Variant (what it actually does) | Success | Final error, median [m] | Selected candidates predicting route progress | **Realized route progress per minute** |
|---|---:|---:|---:|---:|
| Plain controller — no route guidance at all | 1/5 (the known lottery draw) | 0.29 | n/a | ≈0.06 m/min (lottery-dependent) |
| Route guidance, old contact ranking | 0/5 | 0.33 | 7–97% (erratic) | **0.059 m/min** |
| Route guidance + approach-first carrot | 0/3 | 0.43 | 25–57% | 0.062 m/min |
| Route guidance + ΔV-lexicographic contact selection | 0/5 | 0.44 | **72–96% (median ΔV +1.7 to +3.0 cm)** | **0.058 m/min — no better** |
| Same + ΔV-based reposition transactions | 0/5 | 0.44 | mixed (12–88%) | 0.046 m/min — worse |
| shelf_gap, ΔV + transactions (3 diagnostics) | 0/3 | 0.31 | 9–19% (corridor is DIRECT; route selection mostly dormant, as designed) | n/a |

Side effect, reported plainly: demoting the tracking cost to a tie-break let **yaw regress badly** (final yaw 1.2–3.1 rad in the ΔV arms vs ≤1.6 typical before) — orientation needs to stay a real secondary objective, not a 5%-band afterthought. Safety unchanged (no penetrations; clearances as in prior campaigns). No planner aborts in these arms.

## The causal finding

The intervention did exactly what it was designed to do at the selection layer — **and that proves the selection layer was not the binding constraint**:

- Selection shifted decisively toward predicted route progress (acceptance criteria 1–2 met: median selected ΔV positive at +1.7…+3.0 cm, positive-rates 72–96%).
- **Realized route progress did not move** (0.058 vs 0.059 m/min; criterion 3 failed). The ranking rollout's sideways-push predictions are systematically optimistic: it predicts 2–3 cm of lateral object transport per plan that the executed pushes do not deliver (prediction error ≈ the full predicted value at the medians).
- Portal/exit reach and success did not improve; no arm's outcome depended on more than the usual lottery.

## Failure decision tree

- **Not Case A** — positive-ΔV candidates exist in abundance (shadow study).
- **Old ranking was partially Case B** — it under-selected them; that is now fixed and measurable.
- **The live result is Case C: ROUTE PROGRESS WAS SELECTED BUT NOT REALIZED.** The mismatch sits between the ranking rollout's LCP prediction of sideways transport and what OSC-executed pushing physically achieves (contact acquisition, force realization, and the rollout's PD-tracking optimism for lateral pushes). Channel continuation (Case D) and final convergence (Case E) were never reached.

## Answers (spec §16)

1. **Positive-ΔV candidates in the failed campaigns?** Yes, abundantly (up to 99% of events).
2. **Did the old ranking fail to select them?** Partially — selection was erratic (7–97%) and the worst draws picked negative-ΔV contacts.
3. **Did lottery successes ride unusually useful tangential contacts?** Yes — they coincide with the high-ΔV-selection draws.
4. **Did ΔV-aware selection change chosen contacts?** Yes, decisively (positive-rate 72–96%, median +2–3 cm predicted).
5. **Did predicted ΔV correlate with realized ΔV?** **No** — realized per-minute progress was flat; predictions are optimistic for lateral transport.
6. **Did route progress per push minute improve?** No (0.058 vs 0.059; transaction arm 0.046).
7. **Did the object reach the exit portal?** No arm reached it within the run budget.
8. **Did transaction scoring improve reposition efficiency?** Not here — with predictions unrealizable, faster switching to "better-predicted" contacts only added churn.
9. **Did shelf_gap progress through the corridor?** Partially (finals 0.23–0.47) under DIRECT-mode traversal; route selection correctly stayed dormant there.
10. **Remaining limitation?** **Execution/prediction fidelity for lateral pushes** — not candidate generation, not ranking, not channel continuation. The next lever is making the ranking rollout's lateral-transport predictions honest (or the pushes more effective): e.g., validating rollout ΔV against realized ΔV online and discounting contacts with persistent prediction error, and/or restoring orientation as a true secondary objective.

## Verdict

**B. ROUTE PROGRESS WAS SELECTED BUT NOT REALIZED.**

(With the shadow-study corollary that the *old* ranking additionally under-selected existing route progress — fixing that was necessary but measurably not sufficient.)
