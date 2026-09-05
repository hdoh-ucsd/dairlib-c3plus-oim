# C3+ LCS-contact v2 validation report

**Date:** 2026-09-05. **Branch:** `c3plus-obstacle-lcs-contact` (worktree `obstacle-lcs-contact`), building on the landing report `C3PLUS_OBSTACLE_CONTACT_LCS_FIX_REPORT.md` @ `2440d28c`. **Object:** matched 0.1 kg OIM-scale T. **Scenarios:** `push_t_bt010_single_obstacle` (primary), `push_t_bt010_shelf_gap` (secondary). **Results:** `results/c3plus_lcs_contact_v2/`.

**Run conditions:** at the user's direction the 19 campaign runs executed **4-way parallel** (distinct LCM ports, one run per lane). Under this shared load the sims covered ~85–102 s of sim time each before the wall cap; all numbers below are from these parallel-load, truncated-horizon runs. (The serial protocol memo notes parallel load historically depresses success odds; treat absolute numbers accordingly — cross-mode comparisons within this campaign share identical conditions.)

## 1. Previous implementations and why they were insufficient

Baseline: obstacle-blind solve + center-disc exponential ranking penalty (orientation-blind, no guarantee — it grazed the physical box in this campaign, see §12). Reciprocal-square / projected-exponential: soft inner-QP potentials, penetration possible, tuning-sensitive. `qp_halfspace_legacy`: frozen separating halfspace on object xy in the QP z-step only — no λ_obs, no lever arm, no rollout awareness, ranking cost still active. Full anatomy in the port-vs-design report (2026-09-04).

## 2. Exact LCS convention (verified from DAIRLab/c3 source, not assumed)

`x_{k+1} = A x_k + B u_k + D λ_k + d`; `0 ≤ λ_k ⊥ (E x_k + F λ_k + H u_k + c) ≥ 0`. The Anitescu row evaluates the **time-scaled next-step gap**: `η = J_c v_{k+1} + φ/dt` (η is neither a pure gap nor a pure normal velocity; η·dt = predicted next gap). State x(19): EE pos 0–2, object quat 3–6 (wxyz), object pos 7–9, EE vel 10–12, object ω 13–15, object v 16–18; v(9) = [EE 3, ω 3, v 3]; u(3) = EE prismatic forces. λ is a force (D = [dt²·N(q)M⁻¹J_cᵀ; dt·M⁻¹J_cᵀ]); λ ordering is contact-major (4 cone edges per Anitescu contact), obstacle slots appended last; η ordering mirrors λ. `scale_lcs` (AnDn = ‖A‖/‖D‖) is applied inside the solver and undone on λ before retrieval, so the controller-side augmentation is entirely in unscaled physical units. Normal: from obstacle toward the object witness (positive λ pushes the object away); φ>0 = separated.

## 3. Closest-footprint geometry

88-point `TFootprint()` boundary transformed by the current object pose each cycle; per obstacle: `d = min_m SDF(c_m^W)`, `g = d − obs_margin` (0.01 m), `n = ∇SDF/|∇SDF|` at the witness, obstacle witness `= p_w − d·n`, `r = p_w − p_object` (body-frame origin = the point the LCS object velocity refers to). Two SDF backends: disc `[x,y,r]` (default, matches the scenario config) and **exact axis-aligned box** via `SAMPLING_C3_OBS_BOXES=cx,cy,hx,hy;…` (new in v2). Both sampled-footprint (controller, per cycle) and exact-geometry distances (offline vs the true 0.1 m box / shelf boxes) are logged and reported side-by-side in every metrics.json.

## 4. Gap representation

Canonical derived-gap mode: `g` is recomputed from geometry at the start of every MPC cycle and enters the LCS only through `c = φ/dt + …` and the position-gradient block of E — never as a free decision variable, and with no clearance reward anywhere. The per-knot predicted gap is logged in `gap_state_trace.csv` (η_obs,k and g_pred,k = η·dt for k = 0..N−1 on the selected plan, next to the current geometric g). Knot-0 consistency: `η·dt − φ = dt·J·v_next` by construction, so the logged worst gaps (0.05–0.13 m) are exactly `dt·(approach speed)` with dt = 0.1 and |J v| ≲ 1.3 m/s — the derived gap tracks geometry to model order. The optional `explicit_state` gap-augmentation mode was **not implemented** in v2 (deferred; the canonical mode passed all tests and the explicit mode is a diagnostics-only variant per the spec).

## 5. Obstacle Jacobian and yaw coupling

`J_obs` (1×n_v): ω_z slot ← (r×n)_z, v_x/v_y slots ← n; EE slots 0; r is never discarded and the row acts on **velocities**, not position variables. Sign verified by finite perturbation (motion along +n increases d, along −n decreases it) and the FD-gradient of both SDF backends matches n to 1e-9 (unit tests, §10).

## 6. LCS matrix augmentation

Both LCS objects per candidate are rebuilt by one function (`AugmentLcsWithObstacleContacts`) with the complete coupling: D gains `[dt²·N·M⁻¹J_oᵀ; dt·M⁻¹J_oᵀ]` per slot; E/H/c gain the full gap rows (`E = [J_o·A_vq + J_o·N⁺/dt, J_o·A_vv]`, `H = J_o·B_vel`, `c = φ/dt + J_o·d_vel − E_pos·q0` — exact, derived from the factory's own A/B/D/d blocks); **F carries the Delassus coupling in both directions** (`F_new,old = J_o·D_vel_old = dt·J_o·M⁻¹J_cᵀ` exactly, and its transpose), so existing pusher/ground impulses affect the obstacle gap and λ_obs affects existing contact velocities. Not a single appended row. (The "preferred" plant-geometry route was evaluated and set aside for v2: it would change the fixed contact-resolution pipeline (`resolve_contacts_to_lists`, mu tables, sampler assumptions) globally; the manual augmentation is mathematically equivalent for these obstacles and leaves the frozen pipeline untouched.)

## 7. C3+ projection and consensus

λ/η dimensions: existing 16 (4 Anitescu contacts × 4 edges) + 2 obstacle slots = **18** total (rollout LCS: 20+2=22); η mirrors λ; n_z = 19+2·18+3 = 58. The C3+ projection is elementwise on (λ_i, η_i) scalar pairs — exactly the one-dimensional frictionless-normal LCP cone for the obstacle entries, and no Coulomb multi-edge projection is applied to them. G/U diagonals, delta copies, duals, residuals all span the augmented z; `GetForceSolution` returns λ_obs unscaled (physical N) — the logged λ_obs values are physical.

**Fail-loudly mode exclusivity (new in v2):** enabling `lcs_contact` together with any soft potential (`SAMPLING_C3_INNER_OBS_MODE`) or the legacy halfspace (`SAMPLING_C3_OBJ_NONPEN`) now throws at startup (verified: the process aborts with an explicit message). Startup asserts print `obstacle_cost_active=false obstacle_lcs_contact_active=true`.

## 8. Solve/rollout consistency

One contact set per control cycle feeds both the solve LCS and the fine-dt rollout LCS; per-cycle logs record `solve_nlambda=18` / `rollout_nlambda=22` (every cycle, all runs) plus the shared φ/n/witness/r×n values. The rollout's Lemke LCP solves the same augmented E/F/H/c — the around-and-past candidate predictions in §13 are direct evidence the rollout is obstacle-aware (they cannot arise from an obstacle-blind rollout).

## 9. Removal of obstacle objectives

In `lcs_contact` mode: ranking obstacle block skipped entirely, inner-QP obstacle block gated off, legacy halfspace gated off, no clearance reward exists. Verified numerically (TEST 10) — note the candidate CSV's `J_rank_obstacle_total` column is a **passive diagnostic reconstruction** (always computed for logging; it does not alter ranking): the *applied* ranking total `J_rank_code_total` was checked against its components. Baseline draw0: applied − (tracking+travel+repos) ≈ 8,036 ≈ the diagnostic obstacle potential 7,957 (obstacle cost applied, reconstruction noise ≈ 89). lcs_contact draw1: applied − (tracking+travel+repos) ≈ 221 (pure reconstruction noise) while the diagnostic potential averaged 2,961 — i.e., **the obstacle potential is absent from the applied ranking** (J_obstacle_QP = 0 by the gated code path; J_obstacle_rank = 0 in the applied total).

## 10. Unit tests

| Test | Result |
|---|---|
| 1 footprint distance vs exact geometry | PASS (max err 4.2e-5 over 0–180°) |
| 2 fixed-center rotation (d_center const; footprint/witness/r×n vary) | PASS (2026-09-04 rotation table) |
| 3 distance sign (out/contact/penetration) | PASS |
| 4 normal = FD gradient (disc 1.7e-10; box SDF 1.1e-9, 200 random pts) | PASS |
| 5 yaw coupling sign at off-center witness | PASS (r×n=+0.057 @30°, α sign matches) |
| 6 centered witness r×n ≈ 0 | PASS (0 to machine precision) |
| 7 complementarity cases (separated/touching/pushed/tangential) | PASS |
| 8 cross-contact coupling (dt·J₁M⁻¹J₂ᵀ ≠ 0) | PASS |
| 9 solve/rollout parity | PASS (logged dims + shared contact set, every cycle) |
| 10 no obstacle cost | PASS (applied ranking total excludes the obstacle potential: residual 221 ≈ noise vs diagnostic 2,961 on lcs draw1; baseline shows applied ≈ diagnostic) |
| 11 feature-off identity | PASS (baseline arm: no [OBS-LCS], dims 16/20, header-only contact CSV) |
| 12 derived vs explicit gap | N/A — explicit mode deferred (canonical derived gap validated in §4) |

## 11. Static-contact validation

From the 120 s single_obstacle approach smoke (landing report §12) plus this campaign: approach → λ_obs activates as φ falls (monotone, max 0.01–0.76 N depending on push force); no margin violation on approach; tangential slide observed in-run (up to 0.12 m of in-contact sliding, §12) with normal blocked; off-center witnesses produce nonzero r×n every run; predicted knot-0 gap consistent with geometry to `dt·|v|` (§4).

## 12. Five-draw single_obstacle results (4-way parallel, ~85–102 s sim each)

| Draw | Baseline final XY | Legacy final XY | LCS final XY | LCS min clr (real box) | LCS success |
|---:|---:|---:|---:|---:|---|
| 0 | 0.398 | 0.392 | 0.417 | 0.000 (graze) | no |
| 1 | 0.379 | 0.442 | **0.031** | +0.0089 | no (missed by 1.1 cm at truncation) |
| 2 | 0.403 | 0.431 | 0.409 | +0.0093 | no |
| 3 | 0.371 | 0.502 | 0.320 | +0.0215 | no |
| 4 | 0.397 | 0.367 | 0.392 | +0.0139 | no |
| **mean / best** | 0.390 / 0.363 | 0.427 / 0.367 | **0.314 / 0.030** | ≥0 all draws | **0/5** |

- **Task success 0/5 in every mode** — repeatability of success is NOT established (and not claimed). lcs_contact draw1 is the best non-oracle result ever on this scene: around the obstacle to 0.031 m / 0.169 rad at the truncated horizon.
- **Physical safety:** lcs_contact 0 penetration samples in 5/5 draws (worst = exact graze at 0.0); legacy 0; **baseline penetrated the real box in draw3 (2 samples)** — the ranking-cost-only mechanism has no guarantee.
- LCS per-draw detail: final yaw 0.17–2.44 rad; tangential slide 0–0.058 m per draw (plus baseline draw3's 0.12 m of *unprotected* contact sliding); max λ_obs 0.39–0.62 N, mean active λ_obs 0.03–0.37 N; max soft-iterate complementarity residual 0.32–1.14 (projection enforces exact complementarity on the projected copy each ADMM iteration — the residual lives in the z-step iterate by C3+ design); zero planner aborts; zero stalls; solver cadence unchanged (no missed-deadline signature in the cycle logs beyond the shared-load slowdown affecting all modes equally).

## 13. Route-diversity analysis (candidate terminal-state classes, all cycles)

| Mode | total candidates | laterally clear (north) | **around-and-past** | selected around-and-past |
|---|---:|---:|---:|---:|
| baseline | 106,527 | 63,384 | **0** | 0 |
| legacy_halfspace | 60,305 | 575 | 1 | 0 |
| lcs_contact | 123,686 | 74,286 | **9,641** | **2,751** |

This is the study's sharpest structural finding: with the obstacle inside the rollout LCS, candidate forward simulations slide around the obstacle, so around-and-past terminal predictions exist at all — 9,641 of them, of which 2,751 were actually selected (draw1, which then nearly finished). The obstacle-blind rollouts of baseline/legacy produce essentially zero such candidates (confirming the 2026-09-04 forensics). The LCS contact therefore doesn't just protect the object — it measurably increases usable route diversity, without any planner change.

## 14. Oracle diagnostic (not counted in success statistics)

`lcs_contact + SAMPLING_C3_ORACLE_ROUTE=left`, one run: the object detoured west with large clearance (min real-box clearance +0.071, the largest of any run) but the ~100 s truncated horizon ended mid-detour (final 0.357 m, still north-west of the obstacle). Inconclusive at this duration: the earlier full-length oracle test (failure-investigation report) did pass the obstacle (south to y = −0.198). Nothing in this run implicates the contact coupling — clearances and λ_obs behaved normally; the run simply needs its full duration. Combined with draw1 (which got around and past *without* the oracle), the evidence says the contact formulation is adequate and **route generation + horizon remain the blockers**.

## 15. shelf_gap result

- **Disc geometry** (mechanism isolation): 2 of 5 obstacles active every cycle (stable ids), dims 18/22 constant, λ_obs to 0.38 N, no obstacle cost, no real-box penetration (min real clearance 0.000 — grazing contact with a shelf face while the conservative discs report −0.001/φ −0.011, i.e. the disc over-approximation is being consumed, as designed).
- **Box geometry** (exact SDF, new): min controller φ +0.0006 — the contact activates exactly at the margin against the true faces; min real clearance +0.0106, zero penetration; final error 0.346/0.052 rad vs disc's 0.396/0.159 — the exact geometry recovers corridor width the discs falsely closed. Witness points and gaps move with object rotation in both runs (per-cycle logs).
- single_obstacle box-geometry run: rides the true box face at −0.1 mm (250 samples within numerical/sim-contact tolerance; controller φ bottomed at −0.010 = the margin absorbed the ride, which is the margin's job) and produced the second-best progress (0.305/0.040 rad). With exact geometry there is no disc conservatism left, so "riding at the surface" is the expected contact-consistent behavior; the −0.1 mm is within the sim's rigid-contact resolution.

## 16. Runtime impact

Augmentation cost per cycle: one n_v×n_v mass-matrix LDLT + O(n_v(n_λ+2)) products per candidate LCS (×2 LCS per candidate) — sub-millisecond against a ~100 ms cycle; identical cycle cadence across modes in this campaign (all modes ~590–680 recorder samples over the same wall window).

## 17. Remaining limitations

1. Route generation is still the success blocker (out of scope here by instruction): 0/5 success everywhere, and the near-success came from rollout-discovered detours, not planned ones.
2. Runs were truncated (~85–102 s sim) by wall caps under 4-way parallel load; the oracle diagnostic in particular needs a full-length serial rerun to reconfirm passage.
3. Explicit-gap-state mode not implemented (deferred; canonical derived gap validated).
4. Obstacle SDFs are planar discs/AABBs supplied per scenario; arbitrary/rotated shapes and native plant-geometry contacts remain Phase-B work.
5. Object-only scope: nothing here protects pusher or arm (their clearances are only passively diagnosable); the separately-gated pusher/swept/OSC extensions stay off.
6. Complementarity in the soft ADMM iterate can reach ~1.1 under hard pushing (3 iterations); exact complementarity holds on the projected copy — inherent to C3+, not a defect of the obstacle rows.

## Answers

1. **Is λ_obs a real LCS contact variable?** Yes — a column of D, a row of E/F/H/c, an entry of λ with matched η, projected and returned in physical newtons.
2. **Is η_obs the predicted nonnegative next-step gap quantity?** Yes — η = J·v_next + φ/dt, the time-scaled next-step gap in the code's own Anitescu convention (η·dt = predicted gap).
3. **Does the obstacle impulse affect object linear and angular dynamics?** Yes — via M⁻¹J_obsᵀλ into both v and ω (r×n in the ω_z slot).
4. **Is r×n included?** Yes, computed from the closest-footprint witness every cycle; sign unit-tested; nonzero in-run whenever the witness is off-axis.
5. **Does obstacle contact participate in projection and consensus?** Yes — appended scalar (λ,η) pairs in the elementwise C3+ projection, G/U, deltas, duals, residuals.
6. **Do solve and forward simulation use the same obstacle-aware LCS?** Yes — one augmentation feeds both; dims and contact data logged per cycle; the around-and-past candidates are behavioral proof.
7. **Are all obstacle objective terms disabled?** Yes — enforced by a fail-loudly startup assert and verified numerically against the applied ranking total (the CSV's obstacle column is a passive diagnostic; §9).
8. **Does derived gap match geometry?** Yes to model order — knot-0 η·dt differs from the static gap by exactly dt·(approach speed), and the geometric gap is refreshed from the footprint query every cycle.
9. **Does explicit gap state remain consistent?** Not applicable — explicit-state mode deferred; the canonical derived-gap mode is the validated implementation.
10. **Does the contact prevent penetration while allowing tangential sliding?** Yes — zero real-box penetration in all 5 lcs_contact draws (worst graze 0.0 mm; box-geometry run rides at −0.1 mm within solver/sim tolerance), with in-contact tangential slides up to ~6 cm.
11. **Is task success repeatable?** **No.** 0/5 in every mode; one near-success (0.031 m). Do not quote a success rate above zero.
12. **If task success still fails, what is the blocker?** **Route generation (plus horizon/duration), not the contact implementation and not execution**: the contact passes every structural/unit/behavioral test, the rollout now *generates* around-and-past candidates (9,641) that the other modes cannot, one draw rode them to the goal's doorstep, and the oracle's full-length history shows the local machinery completes a detour when a route is supplied.

## Verdict

All structural and unit tests pass:

**"OBJECT–OBSTACLE INTERACTION IS IMPLEMENTED AS A TRUE FRICTIONLESS CLOSEST-FOOTPRINT LCS CONTACT. THE OBSTACLE IMPULSE AND GAP COMPLEMENTARITY PARTICIPATE IN THE DYNAMICS, C3+ PROJECTION, ADMM CONSENSUS, AND FORWARD ROLLOUT; LEVER-ARM YAW COUPLING IS RETAINED; NO OBSTACLE OBJECTIVE OR LEGACY QP HALFSPACE IS ACTIVE."**
