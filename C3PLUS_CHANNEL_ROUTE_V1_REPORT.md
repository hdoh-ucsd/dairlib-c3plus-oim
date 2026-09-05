# C3+ channel_v1 route supervisor — implementation & evaluation report

**Date:** 2026-09-05. **Branch:** `c3plus-channel-route-v1` (worktree `obstacle-lcs-contact`), parent `c3plus-obstacle-lcs-contact` @ `b7364606e`. **Modified:** `systems/controllers/sampling_based_c3_controller.{cc,h}` only; C3 library untouched (DAIRLab/c3 pin unchanged). **Build:** `//examples/sampling_c3:franka_sampling_c3_controller` green. **Results:** `results/c3plus_channel_route_v1/`. Modes preserved: `SAMPLING_C3_ROUTE_MODE={none,channel_v1}` (default none = validated lcs_contact behavior), `SAMPLING_C3_REPOSITION_SCORE_MODE={current,transaction_v1,transaction_v1_1}`.

## 1. Motivation

The measured local deadlock (rollout(2) forensics): with the sub-goal pointing through the obstacle, no candidate predicts translational progress and the controller churns. channel_v1 supplies the missing route decision as **reference shaping only** — the obstacle stays a frictionless LCS complementarity contact; no obstacle potential, halfspace, or clearance cost is added anywhere (§2 asserts of the brief hold: the lcs_contact conflict assert still fails loudly on any soft-potential env, and route mode adds no cost term — it edits `x_desired(7,8)` per candidate, the same hook the oracle diagnostic validated).

## 2–5. Architecture, frame, channels

- **Task-relative frame** latched at the first blockage: `p_hit`, `e_fwd = normalize(goal−p_hit)`, `e_left = perp`; CW/CCW from the sign of the lateral coordinate — never recomputed while latched (TEST H).
- **Geometry source = the LCS-contact source**: same scenario discs / `SAMPLING_C3_OBS_BOXES` exact AABBs, same 88-point T footprint; identical SDF formulas (`ObsSdfPoint` mirrors the contact witness math). Segment feasibility = full-footprint swept sampling every ≤2 cm against `route_margin = obs_margin = 0.01`.
- **Channels**: obstacles project to inflated lateral intervals (inflation = min-width(θ)/2 + margin over 8 yaw samples), merged into clusters; every wide-enough free lateral interval (bounded by the workspace projection) becomes a channel with an entry portal (cluster near edge − 0.08) and exit portal (far edge + 0.08); DIRECT = swept-feasible straight path. 2 gaps → CW/CCW naming; ≥3 → OUTER_LEFT/CENTER_GAP/OUTER_RIGHT. Corridor yaw θ\* = argmin width (feasibility only in v1; yaw reference untouched — documented deviation from the corridor-yaw-hold spec).
- **Route value** V = remaining polyline length through the latched channel (entry + interior + exit legs — never `d(obj,obs)+d(obs,goal)`), recomputed each cycle with passed-node skipping.

## 6–9. Cost, latching, state machine

`C_h = V/V_ref + 0.1·A/θ_ref (CENTER_GAP only) + 0.1·T_repos/5 + 0.25·switch`; discrete argmin; latch stored with frame/exit-portal/selection data; switch only on infeasibility or `C_new + 0.15 < C_active` persisting 10 cycles (both logged). States implemented: NORMAL_DIRECT / CHANNEL_SELECT / CHANNEL_FOLLOW (+ CHANNEL_APPROACH added mid-study, below) / REJOIN. Rejoin requires all of: direct swept path free, global goal distance < hit distance − 0.02, object beyond the exit portal. CHANNEL_RECOVERY (per-channel contact-sector retry) is **not implemented** in v1 — listed as a limitation.

## 10. Sub-goal generation & the measured v1 flaw

v1 as specified used a polyline-lookahead carrot (0.15 m ahead of the object's route projection). **Measured in-loop failure (route-tracking class D):** while the object is still ~0.17 m off-lane, the projection-based carrot points diagonally forward; C3+ pushes the object down the blocked centerline, the projection slides along with it, and the run regresses (group D finals 0.41–0.50 vs current 0.33 median — worse than no supervisor). The offline tests could not see this because it is a closed-loop tracking interaction, not a geometry error. **Amendment (approach-first / CHANNEL_APPROACH):** while `|lat_err| > 0.05` and before the exit portal, the sub-goal is a pure lateral step toward the lane; only in-lane does the carrot advance. This restored safe behavior (below) and matches the CHANNEL_APPROACH state the brief prescribed.

## 11. Offline geometry tests (all PASS, 10/10 before any simulation)

A: 50 randomized box/start/goal scenes — CW+CCW always produced, task-relative, collision-free (0 violations). A2: scene rotation invariance of route lengths. B: symmetric scene V_CW = V_CCW to 1e-6 + deterministic tie-break. C: offset obstacle — shorter side wins. D: shelf scene — channels found: **DIRECT (0.600), CENTER_GAP (0.600), OUTER_RIGHT (0.959)**; OUTER_LEFT correctly removed by the workspace bound (x ≤ 0.75). E: closed gap → CENTER_GAP infeasible. F: workspace-blocked outer removed. G: route value monotone along the route. H: latched-frame side labels stable while rounding. Visualization: `offline_geometry/route_geometry.png`. Load-bearing finding: **in the real shelf_gap scene the DIRECT path through the corridor is feasible for the T**, so the supervisor correctly stays passive there and the corridor walls are handled by the two N_closest LCS contact slots.

## 12–14. Online results (groups A–D; 2-lane parallel, ~280–345 s sim per run; success gate 0.02 m/0.1 rad)

single_obstacle, 5 draws each:

| Group | Modes | Success | Final XY per draw | Median |
|---|---|---:|---|---:|
| A current | route none / repos current | **1/5** (draw3: **0.0035 m / 0.097 rad @68 s** — second-ever lottery success) | .327 .353 .289 **.004✓** .130 | 0.289 |
| B route_only (v1 carrot) | channel_v1 / current | 0/5 | .211 .328 .363 .254 .412 | 0.328 |
| C repos_only | none / transaction_v1_1 | 0/5 (no livelock — timeout+symmetric gate worked; C3 fractions normal) | .478 .399 .180 .385 .395 | 0.395 |
| D combined (v1 carrot) | channel_v1 / transaction_v1_1 | 0/5, **regression** | .495 .432 .502 .411 .445 | 0.445 |
| B/D with approach-first fix (3+3 draws) | | 0/6 | B: .438 .384 .427; D: .362 .420 .511 | 0.424 |

shelf_gap: A 5 draws — 0/5 (best 0.019 m position with 0.86 rad yaw remaining; corridor traversal works under plain lcs_contact); B/C 1 diagnostic each (0.47 both); D 5+1 draws 0/6 (0.24–0.53). Supervisor correctly reports DIRECT feasible and stays passive in shelf_gap; the D-arm deltas there are draw variance plus the v1 carrot in the moments the direct path is transiently blocked by pose.

Channel behavior itself was correct everywhere measured: single latch per run (e.g. `[ROUTE] latched channel CCW V=0.758`), zero oscillation (switch-hold never even engaged), sub-goal marched along the lane, route logs written (`route_state.csv`).

## 15. Failure classification (every failing arm)

- Group B/D with v1 carrot: **D — route tracking failure** (diagonal carrot; root-caused, fixed).
- B/D with approach-first: **D/E mixed — slow lateral tracking + reposition inefficiency**: the reference is now correct (object stays 0.06–0.14 m clear, moves toward the lane) but pushing a T sideways ~0.2 m demands repeated contact reconfigurations; progress ≈0.2 m per ~330 s and runs end mid-route. Not A (channels valid), not B (selected channel is the cheaper one), not C (no premature switches — zero switches observed), not F (zero penetrations campaign-wide), not H (pusher reached its targets).
- Group A successes remain **rollout-lottery**: the around-route appears when sampled contacts happen to slide the object past the corner (2 successes in ~13 all-time draws of plain lcs_contact).
- shelf_gap all arms: **I — final convergence** (position often solved, yaw polish incomplete within the horizon) plus draw variance.

## 16. Runtime overhead

Route supervision costs one swept-feasibility scan + channel build per cycle (~1–3 ms; dominated by the ≤2 cm footprint sampling) — invisible next to the ~100 ms control cycle. No cadence change observed between arms.

## 17. Generalization limits

Planar convex clusters only; AABB/disc SDFs (rotated boxes unhandled); corridor-yaw reference not actively tracked (θ\* used for feasibility only); CHANNEL_RECOVERY and sector-keyed unsuccessful memory not implemented; route-progress ΔV is logged but not yet wired into candidate ranking or the reposition transaction (the §11–12 integration of the brief is the natural next step and is precisely what the class-D/E residual calls for).

## Answers

1. **Random placement & CW/CCW correctness?** Correct — 50 randomized scenes, task-relative labels, zero collisions; rotation-invariant costs (TEST A/A2).
2. **DIRECT, CW, CCW produced on single_obstacle?** Yes — DIRECT infeasible (blocked), CW 0.804 / CCW 0.730 both collision-free.
3. **OUTER_LEFT / CENTER_GAP / OUTER_RIGHT on shelf_gap?** CENTER_GAP and OUTER_RIGHT produced; OUTER_LEFT correctly pruned by the workspace boundary (a feasibility removal, not a failure).
4. **Was CENTER_GAP feasible?** Yes — 0.20 m gap vs 0.099 m min T width + margins; and the DIRECT path through it is itself feasible.
5. **Which channel selected, why?** single_obstacle: CCW (V 0.730 < CW 0.804 under the asymmetric workspace clip); shelf_gap: DIRECT (supervisor passive).
6. **Route value monotone during progress?** Along the route geometry, yes (TEST G); in-run V decreased while the object advanced and plateaued when tracking stalled — correctly reflecting reality.
7. **Did latching prevent oscillation?** Yes — one latch per run, zero switches, switch-hold never triggered.
8. **Did the sub-goal move along the route?** Yes (v1 verbatim), and after the amendment it correctly holds laterally until the lane is reached.
9. **Did C3+ execute without an obstacle objective?** Yes — no obstacle cost active in any route arm (assert-enforced + applied-cost verification carried over from v2).
10. **Did transaction_v1_1 improve reposition efficiency?** It eliminated the v1 livelock (no run below 20% push fraction from that mechanism; timeout + symmetric gate both fired in logs) but did not raise success (0/5 alone).
11. **Did the object pass the obstacle/gap exit?** Only in the lottery successes (group A draw3 here; txn trial1 previously). Route-guided arms reached the lane but not the exit within the run budget.
12. **Remaining failing layer?** **Execution of lateral transport** — converting a correct off-lane reference into efficient sideways pushing (route-aware reposition/candidate integration, §11–12 of the brief), not route generation (validated), not collision handling (zero penetrations), not channel selection or latching.

---

**CHANNEL_ROUTE_V1 USES GEOMETRIC FREE-SPACE ROUTE VALUES TO SELECT AND LATCH A DIRECT, CW/CCW, OR SHELF-CORRIDOR CHANNEL. IT SUPPLIES SHORT ROUTE LOOKAHEAD SUB-GOALS TO C3+, WHILE OBJECT–OBSTACLE SAFETY REMAINS ENTIRELY IN THE LCS COMPLEMENTARITY CONTACT. NO OBSTACLE PROXIMITY COST IS ADDED TO THE C3+ OBJECTIVE OR CANDIDATE RANKING.**
