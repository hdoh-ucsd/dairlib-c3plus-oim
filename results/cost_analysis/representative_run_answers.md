# Representative-run answers (§10 A–H) — reconstructed C3+ costs, 2026-09-08

Source: `cost_diagnostics_v2.py` reconstruction at the MEASURED state per control step
(J_trans/J_rot per cost_semantics_notes.md; J_obs = ranking-layer 5000·exp(−d/0.04),
CASE C — the local C3 objective is obstacle-blind, so J_obs is never summed into J_C3_task).

**GLOBAL CAVEAT (applies to every "prediction" answer below):** true per-candidate
PREDICTED rollout costs were never logged (CostLogger env `SAMPLING_C3_COST_LOG_DIR` unset).
What the outer loop actually compared — `J_rank(0)` of the current config vs
`min_i J_rank(i)` of sampled candidates under relative hysteresis (0.4/0.6 exit fractions,
0.9/0.5 re-entry, plus the 1%-drop-over-180-loops unproductivity gate) — is known only
QUALITATIVELY from source. All quantitative statements use the measured-state cost, a weak
proxy (single point, no horizon rollout, and prior forensics showed 74% of decisions were
issued mid-travel).

## open_task/pair02 — SUCCESS
- **J_trans decreased?** Yes, monotone staircase 8635 → 22 (near-zero) at success (79.1 s).
- **J_rot decreased?** Yes overall (48 → 4.9); a large transient spike to ~100-150 near step
  5200 (final yaw approach, e_yaw briefly 0.45 rad) is resolved in the last contact burst.
- **Obstacle cost/clearance:** no obstacles — J_obs ≡ 0 (flat, labeled).
- **Outer loop:** hysteresis over rollout costs (qualitative only; predicted costs unlogged).
  11 repositions, 18 no-progress exits — the staircase pattern (flat contactless plateaus,
  drops during shaded contact episodes) matches selection cycling until productive contact.
- **Low measured cost + no contact?** Yes in structure: plateaus between drops are contactless
  (contact fraction only 0.19 overall), but cost never rises there — parked, not regressing.
- Cost-progress Pearson r = 0.96.

## open_task/pair01 — TRANSIENT_SUCCESS (F5)
- **J_trans/J_rot decreased?** Yes: 8000 → 4.5 and 49 → then latched at 38.1 s. The failure is
  the terminal yaw overshoot: after success J_rot climbs back to ~8.6 (e_yaw 0.13 rad > 0.10)
  in the last ~0.3 s and the launcher stops before recovery — visible as the small post-green-
  line uptick of the orange curves in both panels.
- **F5 signature:** the executed push did not arrest the yaw the (unlogged) prediction promised
  — consistent with the proxy caveat; measured cost at decision time was already near-zero.
- J_obs ≡ 0 (no obstacles). r = 0.93.

## ycb_clutter/pair02 — F1 (contact acquisition), TRUE_STALL
- **J_trans/J_rot decreased?** Essentially NO. After a tiny initial drop (~step 400) J_trans is
  frozen at ~7350 and J_rot at ~48 for 22k steps. Physical contact fraction 0.013; 65/66 C3-like
  episodes are contactless.
- **Obstacle cost:** J_obs steps up to ~250 at the initial nudge and stays there — the object
  is parked at moderate clearance; obstacles are NOT what stops it. The stall is pure contact
  acquisition failure: the outer loop keeps selecting candidates (dense shading) whose execution
  never achieves contact.
- **Low measured cost + no contact?** Inverted here: measured cost is HIGH and flat; the loop
  churns because the (unlogged) predicted costs presumably promised improvement the contactless
  executions never realized — proxy data cannot confirm the promise, only the zero realization.
- r = 0.29 (cost barely moves, so correlation is uninformative).

## single_obstacle/pair01 — F2 (phantom contact churn), TRUE_STALL
- **J_trans decreased?** Only in the first ~700 steps (8000 → ~1975 during the single real push),
  then perfectly flat for ~23k steps. J_rot flat at ~44.
- **Obstacle cost:** J_obs jumps to ~1646 when the object is pushed next to the box obstacle and
  stays maximal to the end — the object parks in the obstacle's exponential skirt, but since the
  local C3 is obstacle-blind and the ranking cost is state-invariant while nothing moves, no
  gradient acts.
- **Validation-required signature confirmed:** long stretches with flat J and almost no contact
  ticks; 12/13 episodes contactless (contact fraction 0.013). The near-continuous episode
  shading with no contact ticks IS the phantom-churn picture: C3-mode episodes that never touch.
- r = 0.98 (driven entirely by the initial transient).

## icra_sign/pair05 — F6 (obstacle geometric block), TRUE_STALL
- **Did J_obs dominate before progress stopped?** YES. J_obs rises to 2000-2600 by ~step 300 and
  stays the LARGEST single component for nearly the whole run (mean 2316 vs mean J_trans 1619,
  mean J_rot 1868): the C glyph operates inside the obstacle skirt throughout. Task progress
  (J_task 8600 → ~800 by step 5600) happens DESPITE a dominant ranking-layer obstacle term;
  after the yaw blow-up at ~step 5000 (e_yaw 0.05 → 2.8 rad, J_rot → 2900) everything freezes
  with J_obs ≈ 2635 at terminal — the object wedged against glyph hulls, mid-rotation.
- **J_trans decreased?** Yes until ~step 5600 (final 591, e_pos 0.24 m); J_rot catastrophically
  INCREASED at the block. r = −0.38 — the only representative run where cost and error decouple,
  because the pre-latch J_trans regime and the rotation collapse move oppositely.
- Contact fraction 0.80 — this is a geometric block, not an acquisition failure.

## shelf_gap/pair05 — F12 (timeout but progressing)
- **Were task costs still decreasing at cap?** Largely converged rather than actively falling:
  J_task descends 7700 → ~45 by ~step 15000 (e_pos crosses 0.05 m near step 12000; through-gap
  transit visible as the J_obs hump peaking ~2050 around steps 3000-9000, then decaying to ~4).
  In the final ~8000 steps J_trans ≈ 19 (position DONE) while J_rot plateaus at ~26
  (e_yaw ≈ 0.22 rad, above the 0.10 rad line) with slow intermittent nibbling — "progressing"
  in the audit sense (monotone trend, no stall class), but the residual yaw was not being
  closed at a rate that would finish before the cap.
- **Obstacle behavior:** textbook — J_obs rises entering the shelf gap, falls to near-zero once
  through; clearance never binds at the end.
- r = 0.98; contact fraction 0.17, 35/75 episodes contactless.
