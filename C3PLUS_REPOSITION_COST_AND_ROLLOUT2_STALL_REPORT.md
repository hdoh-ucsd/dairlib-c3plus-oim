# C3+ reposition cost & rollout(2) stall report

**Date:** 2026-09-05. **Branch:** `c3plus-obstacle-lcs-contact`. **Results:** `results/c3plus_lcs_contact_v2/single_obstacle/{lcs_contact_long,lcs_contact_txn}/`. Obstacle formulation frozen: `lcs_contact` exactly as validated in `C3PLUS_LCS_CONTACT_V2_VALIDATION_REPORT.md` (λ_obs/η_obs in the LCS, closest-footprint witness, r×n, projection/consensus/rollout participation, no obstacle objective, no halfspace) — nothing in this study touches it.

## 1. Video/run mapping (`rollout_run_mapping` equivalent)

| Upload | Overlay title | Run dir | Final XY / yaw |
|---|---|---|---|
| rollout(1) | lcs_contact long trial2 | `lcs_contact_long/trial2` | 0.381 / 1.556 |
| **rollout(2)** | **lcs_contact long trial1** | `lcs_contact_long/trial1` | **0.374 / 0.270** |
| rollout(3) | lcs_contact long trial0 | `lcs_contact_long/trial0` | 0.376 / 0.116 |

All three: branch `c3plus-obstacle-lcs-contact` @ `589be6bbf`, demo `push_t_bt010_single_obstacle` (campaign config snapshot), 0.1 kg T, disc obstacle (0.5, 0, 0.0707), N=5, planning dt 0.05 (pose) / 0.1 (position), goal (0.5, −0.30, π), ~227–247 s sim each, no successes, nondeterministic draws (no seeds). Video-derived checkpoints for rollout(2) verified against `state_trace.jsonl` to within 0.01 m / 0.12 rad at every quoted time; the sustained stall begins at **t ≈ 89 s** (best-XY improves only 0.8 mm over the following 138 s). The naive "<3 mm/10 s" detector fires already at t=19.1 s because the yaw-dominated phase also translates slowly.

## 2. Current reposition state machine (traced from source)

C3 push → `KeepTrackOfC3ModeProgress` (only while in C3) → exits: (a) **kToReposUnproductive** when the 180-loop config-cost window drops <1%; (b) **kToReposCost** when `best_other < 0.6·curr` (relative hysteresis 0.4, pose branch); → candidate selection `argmin over indices ≥1` (**index 0 = staying at the current contact is excluded**); → reposition execution (piecewise-linear lift 0.06 m / lateral / descend at 0.18 m/s, **no timeout**); arrival = remaining path < one planning step → `finished_reposition_flag_` → +1e9 on the reached target next cycle → forced return to C3 ("reached repositioning target") + `AddToUnsuccessfulBuffer`; progress window resets on every transition. Buffered good samples are pruned when the object moves >3 mm/2°; the buffer's best is appended to the C3-mode candidate pool **with its stored stale cost (no fresh solve)** and is deleted from the buffer if chosen. `travel_cost_per_meter = 0` — reposition distance/time enters no score anywhere.

## 3–6. Cost inventory

- **Inner C3+ cost:** tracking + input + contact ADMM terms incl. obstacle slots (validated earlier; per-component logs in `selected_candidate_qp_costs.csv`).
- **Ranking cost (lcs_contact mode):** object pos/orientation/velocity tracking of the rollout + travel (=0) + finished-reposition penalty; obstacle potential verified absent from the applied total.
- **Progress score:** object config cost (first-knot Q 7×7 block, error to the FINAL goal), 180-loop window, 1% relative drop, counts only C3-push loops, resets on every mode change and on cost-switch crossing.
- **Reposition-candidate score:** identical `J_rank` argmin — it accounts for **none** of: reposition distance/duration, arm travel, switching cost, acquisition probability, prior failure at the sector (beyond the 10-deep unsuccessful buffer), or cost staleness (the buffer candidate is explicitly stale).

High-priority checks (§5 of the brief): (A) travel cost **is** zero — confirmed inert at all 4 use sites. (B) stale buffer costs — confirmed structurally (stored scalar + stale plan reused) and behaviorally (below). (C) the stay-current candidate **is** excluded from the argmin. (D) the 1e9 penalty hits only index 1 for exactly one loop; behaviorally it forces the "arrived → immediately abandon" pattern. (E) the cost switch (0.50 m threshold in this config) crossed within seconds of start — far from the t≈89 s stall; not implicated. (F) candidate terminal spread at the stall is 0.06/0.15 m (x/y) — candidates are NOT identical; ranking has signal to work with.

## 7. Rollout(2) phase timeline

| Phase | t [s] | pos err | yaw err | note |
|---|---|---|---|---|
| approach + first push | 0–16 | 0.60→0.41 | 3.14→1.35 | productive |
| major rotation | 16–45 | ≈0.40 | 1.35→0.80 | yaw work, slight xy regressions |
| rotation completion | 45–89 | 0.41→0.374 | 0.80→0.247 | last useful repositions (all 6 useful ones are in the first third) |
| sustained stall | 89–247 | 0.374→0.373 | 0.23–0.27 | 74 repositions, zero realized gain |

Mode budget over the whole run: **51% C3 push / 49% reposition; 109 reposition events (mean 125 cycles); 6 produced >5 mm realized XY gain, 1 harmful, 102 neutral.** Selection in the stall ping-pongs between candidate 1 (fresh sample) and candidate 3 (stale buffer best): 6,955 vs 6,888 selections; candidate 3 wins nearly every event in which it appears (6,888 of 8,431).

## 8–11. Stall-point decomposition, predictions, buffer analysis

At the stall (event-level, 16,759 cycles analyzed):

- Yaw is essentially solved (0.23–0.27 rad vs 0.1 tolerance) yet the orientation term still contributes ~40% of the tracking cost (mean 1,434 vs position 2,198) — the ranking keeps paying for yaw polish while translation is the binding error.
- **No candidate — selected or not — predicts approaching the sub-goal**: the object sits 0.150 m from the (through-obstacle) sub-goal; selected plans predict a terminal 0.202 m away (mean), the per-event best predicts ≈0.17. The obstacle-aware rollout honestly reports that pushing toward a sub-goal behind the obstacle yields break-even at best. λ_obs is active and blocks only normal motion (validated separately); tangential velocity is available and used — but tangential motion doesn't reduce distance to a sub-goal straight through the disc.
- The excluded stay-current candidate (index 0) predicts 0.192 — **better than candidate 1 (0.264) which is selected ~half the time.**
- In 51% of stall events an unselected candidate predicted ≥1 cm better terminal XY than the selected one (mean 3.1 cm) — mis-selection is real but bounded: even the best is ≈break-even.
- Stale buffer: removing candidate 3 from the pool collapses "some candidate beats staying" from **74% → 47%** of stall cycles — the stale scalar was manufacturing phantom improvement.
- Buffer hygiene: unsuccessful buffer receives entries only on repos→C3 transitions; with the object nearly static the good buffer isn't pruned, so the stale best persists until chosen (then deleted, then regenerated) — the ping-pong loop.

## 12. Three-run comparison

trial0 (rollout(3)) reached its plateau earliest (yaw ended 0.116 — it finished rotation efficiently and then hit the same translation wall); trial2 (rollout(1)) got trapped with 1.56 rad yaw remaining (its early contact choice produced a yaw-regression cycle on the box face); trial1 (rollout(2)) is intermediate. All three converge to the same 0.37–0.38 m north-face attractor; the divergence point is the first contact face chosen after rotation completes — none of the three ever selects a laterally-clear-and-south candidate in the stall (0 around-and-past among 1,959 late candidates sampled in trial1).

## 13. Offline counterfactual rescoring (16,759 stall cycles, no reruns)

| Score | Reposition trigger rate at the stall | Note |
|---|---:|---|
| A — current (0.4 relative hysteresis, stale buffer in pool) | 35% | plus unproductive-exits on top; ping-pong 1↔3 |
| B — require any predicted improvement (fresh pool) | 47% of cycles have a better candidate at all (was 74% with the stale buffer) | rejecting ΔJ≤0 kills the remaining churn's justification half the time |
| C/E — transaction-scaled hysteresis (ΔJ must exceed 0.4·curr·(1+T_repos/T_push), T_repos from the actual lift-lateral-descend geometry at 0.18 m/s) | 29% | modest direct cut; combined with the fresh pool it also removes the stale-driven exits |

## 14. Reposition-v2 (`transaction_v1`) formulation — implemented, env-gated

`SAMPLING_C3_REPOSITION_SCORE_MODE=transaction_v1` (default `current` = byte-identical behavior):
1. **Fresh candidate costs only** — the stale-cost buffer augmentation is skipped.
2. **Transaction-valued switching** — the C3→repos cost-exit hysteresis scales with predicted reposition time: leave only if `ΔJ > hyst_frac·curr·(1 + T_repos_pred/T_push)`, `T_repos_pred = (2·0.06 + xy_dist)/0.18` from the real generator geometry, `T_push` = 5 s (env-tunable).
3. **Gated unproductive exit** — "no progress" triggers a reposition only if some candidate actually predicts improvement (`best_other < curr`); otherwise stay and keep pushing (repositioning to nothing better is pure churn).
4. Continuing the current contact is thereby compared implicitly every cycle (`curr` is the stay value); collision/workspace feasibility filters unchanged (hard).
Not included in v1 (documented): fresh re-solve of buffered candidates, sector-keyed unsuccessful memory, per-transaction realized-benefit accounting, progress-detector rework.

## 15. A/B results (5 transaction_v1 draws vs current-mode evidence; parallel-load, ~200 s sim each)

| Metric | current (5 campaign + 3 long draws) | transaction_v1 (5 draws) |
|---|---:|---:|
| task success | **0/8** | **1/5** — trial1: 0.020 m / 0.092 rad at t=70.2 s (**first-ever success on this scene, no oracle**) |
| final XY (per draw) | 0.417/0.031*/0.409/0.320/0.392 + 0.376/0.374/0.381 | **0.020✓**/0.247/0.262/0.356/0.404 |
| best XY | 0.030* (near-miss) | **0.019 (success)** |
| C3-push fraction | ~0.51 | success draw 0.62; others 0.02–0.30 (see caveat) |
| real-box penetration | 0 (surface rides at −0.0001) | 0 beyond the same −0.0 surface-ride tolerance (worst −0.0001-class, 4–17 grazing samples in 3 draws) |

\* current-mode draw1 of the campaign was the 0.031 m near-miss.

**Caveat, reported plainly:** transaction_v1 is higher-variance, not uniformly better. Draw 0 exposed a side effect — push fraction collapsed to 2% (a reposition-heavy livelock: with the buffer candidate gone and exits gated, the repos→C3 return hysteresis (0.9 relative) can hold the controller in reposition mode when candidate costs cluster). Draws 2/4 beat every current-mode final (0.247, 0.262) and draw 1 succeeded outright, but one success in five draws is not repeatability, and the mean final XY (0.258) beats current (0.383) largely on the strength of the success. The livelock needs a v1.1 guard (e.g., symmetric transaction gating of the repos→repos/repos→C3 comparisons or a reposition timeout — none exists today, §2).

## 16. Remaining route limitation

Unchanged: at the stall no local candidate predicts closing on a sub-goal that points through the obstacle. transaction_v1 converts wasted churn into sustained pushing — which raises the probability that the rollout's tangential sliding carries the object around the corner (that is what the success draw did, reaching the south side by t≈50 s) — but it does not *plan* the detour. Route/sub-goal generation remains the structural gap (Case C component).

## 17. Recommended final change

Adopt `transaction_v1` behind its env flag (already landed), then: (1) add the symmetric transaction gate + reposition timeout to kill the draw-0 livelock; (2) key the unsuccessful buffer by contact sector and require fresh re-solves for any buffered candidate before it may enter the pool; (3) only then revisit the progress detector (its 180-loop/1% design was NOT the churn driver — the cost-exit path with stale candidates was; 52 of 109 exits were cost-exits, 57 unproductive-exits mostly downstream of resets caused by the churn itself).

## Final answers

1. **Why did rollout(2) stop near 0.373 m?** The sub-goal points through the obstacle; the obstacle-aware rollout honestly predicts break-even at best for every reachable contact, so C3 pushes stopped paying; the selection layer then burned 49% of the runtime ping-ponging between a stale buffer candidate and a fresh sample, neither better than staying put.
2. **Was the remaining yaw error important?** No — 0.23–0.27 rad is near tolerance; but the still-active orientation term (~40% of tracking cost) kept diluting translation preference.
3. **Was the selected contact predicted to reduce position error?** No — selected plans predicted 0.202 m terminal vs 0.150 current (mild regression).
4. **Was that progress realized?** There was no predicted progress to realize; realized XY gain after t≈89 s was 0.8 mm total.
5. **Did repositioning choose useful new contacts?** 6 of 109 repositions produced >5 mm gain, all before the stall; 0 of 74 after.
6. **Did the cost ignore reposition time or distance?** Yes — `travel_cost_per_meter = 0` and no other term prices time/distance; repositions were free to the score.
7. **Were stale buffered costs involved?** Yes, centrally — the buffer candidate's stored cost (no fresh solve) won nearly every event it appeared in and manufactured phantom improvement in 27% of stall cycles.
8. **Did the no-progress detector trigger appropriately?** Its triggers were technically correct (config cost really wasn't dropping) but it never got a fair window: the churn reset it constantly, and its exits fed more churn. It was an amplifier, not the root cause.
9. **Would a transaction-based score have selected different contacts?** Yes — it removes the stale candidate (changing the winner in ~half the stall events), suppresses ~6% of cost-exits outright via time-scaled hysteresis, and blocks unproductive-exits in the 53% of cycles where nothing beat staying.
10. **Can reposition-cost changes solve the failure?** Partially. Empirically transaction_v1 produced the first-ever success (1/5) and two best-ever non-success finals — but the around-route is still discovered by sliding, not planned. Candidate/route generation is still required for reliable success, and v1 introduced a livelock mode of its own to fix.

## Verdict

**MULTIPLE CAUSES — RANKED WITH MEASURED EVIDENCE:**
1. **NO LOCAL CANDIDATE PROVIDED TRANSLATIONAL PROGRESS** at the stall (best predicted terminal 0.17 vs current 0.15 m; 0 around-and-past among 1,959 late candidates) — the outcome cap (Case C).
2. **REPOSITION CANDIDATES WERE MIS-SCORED** (stale buffer cost winning selection, zero travel cost, stay-current excluded, hair-trigger 0.4 relative hysteresis) — 49% of runtime spent on 74 zero-gain repositions (Case A); fixing this produced the first success.
3. **PROGRESS-DETECTOR MODE CHURN** — real but secondary; its resets and exits are mostly downstream of causes 1–2.
