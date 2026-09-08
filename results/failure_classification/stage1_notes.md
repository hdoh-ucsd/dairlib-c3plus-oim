# Stage-1 failure classification — notes (2026-09-08)

Analyst script: `results/failure_classification/stage1_analyze.py` (read-only on run data;
16-way multiprocessing over the 30 runs). Outputs: `stage1_outcomes.csv`, `terminal_progress.csv`.

## 1. Metric validation (per §3/§5)

Sources: `tools/scene_smoke/postprocess_run.py` and
`results/xarm6_c3plus_scene_smoke/metrics/task_diagnostics_definition.md`.

- **e_pos** = planar L2 to goal: `hypot(x-gx, y-gy)` — confirmed (postprocess_run.py:436-438;
  definition md lines 10-13). Matches the CSV `position_error_m` to 1e-9 (script's own VALIDATE).
- **e_yaw** = `|wrap_to_pi(yaw - goal_yaw)|`, wrap = `(a+pi) mod 2pi - pi` into (-pi, pi] —
  confirmed (postprocess_run.py:79, 186; definition md lines 14-17).
- **Official tolerances**: pos_tol 0.05 m, ang_tol 0.10 rad, read from the recorder FINAL line
  with defaults 0.05/0.1 (postprocess_run.py:313-314); every recorder FINAL in this campaign
  carries `"pos_tol": 0.05, "ang_tol": 0.1`. Not changed here.
- **No dwell in the official criterion**: `success = bool(succ_rows)` — ANY single control step
  with both errors inside tolerance (postprocess_run.py:400-403); `first_success_t` is the
  recorder's trace-latched SUCCESS time. Confirmed no dwell/hold anywhere in the official path.
- **Goal ramp weights**: `ramp = min(1 + 0.005*k, 30.0)` per control step k, applied to both
  `goal_pos = ramp*200*d2` and `goal_theta = ramp*16*wrap(dyaw)^2`
  (definition md lines 71-76; postprocess_run.py Q_RAMP_PER_STEP=0.005, Q_RAMP_MAX=30.0,
  Q_POS=200, Q_THETA=16). So the goal terms grow 6000x in weight over the first 5800 steps.

## 2. Analysis-only labels (declared, NOT official)

- **sustained_success**: exists a continuous >=5.0 s window (state_trace time) with BOTH errors
  inside tolerance. The 5 s dwell is an ANALYSIS label only.
- **Dwell right-censoring**: the launcher stops each run ~2.5-3.2 s after the recorder SUCCESS
  latch, so NO official success can ever exhibit a 5 s dwell in this campaign. A run whose trace
  ends inside tolerance and <5 s after first_success is labeled `dwell_censored=True` and
  counted SUCCESS; only a run that *exited* tolerance before trace end is TRANSIENT_SUCCESS.
- **Timing base**: state_trace `t` (0.1 s-ish cadence, wall-clock-like) is used for all timing;
  the metrics-CSV `sim_time` is used only for contact-episode seconds. Cross-check: trace end
  times (40.7-366.3 s) exceed CSV sim_time_end where the planner lags — consistent with the
  known sim_time-lags-wall caveat. "600 s" is the wall cap; trace durations are ~295-366 s for
  cap-reaching runs.

## 3. Noise floor and sensitivity

- Per-run floor = peak-to-trough of e_pos / e_yaw over the flattest 30 s window (min e_pos std).
  Pooled (median across 30 runs): **pos 0.0004 m, yaw ~0.0000 rad** (stalled runs are dead-flat);
  p90: pos 0.0167 m, yaw 0.311 rad (p90 windows include real motion, not noise).
- Post-success holds (6 official successes, latch->trace-end): pos ptp median 0.0005 m,
  max 0.0249 m; yaw ptp median 0.0061 rad, max 0.0641 rad.
- The prescribed TIMEOUT_PROGRESSING thresholds (terminal-120s best-error improvement
  > 1 cm pos OR > 0.05 rad yaw) are ~25x / >100x the pooled median floor — comfortably > 2x
  floor. Justified.
- **Sensitivity (floor x1 / x2 / x4 as the 2x-floor threshold)**: because the median floor is
  near zero, x1/x2 thresholds are sub-millimeter and would flip up to 15 TRUE_STALLs to
  TIMEOUT_PROGRESSING on micrometer creep — degenerate, rejected. At x4 (0.003 m / 0.0002 rad)
  only **icra_sign_pair05** flips (its 120 s window shows a few-mm improvement). No run flips
  in the other direction at any multiplier. Conclusion: labels are robust under the fixed
  1 cm / 0.05 rad thresholds; icra_sign_pair05 is the single boundary case.

## 4. Productive-contact definition (stated per task)

Contact step k (metrics CSV, `physical_contact_active`) is **productive** iff, over the
+/-3-step window [k-3, k+3] (clamped at ends), the finite-difference rate satisfies
`d(e_pos)/dt < -1 mm/s` OR `d(e_yaw)/dt < -0.01 rad/s` using CSV sim_time for dt.
`productive_contact_fraction` = productive contact steps / all contact steps.
Contact episodes = maximal runs of consecutive contact-active steps; episode seconds via the
sim_time span of the episode; `contact_acquisition_rate` = episodes per minute of sim_time.

## 5. Outcome rules applied

SUCCESS = official (recorder SUCCESS / result.json first_success_t) AND (5 s dwell OR dwell
censored as in §2). TRANSIENT_SUCCESS = tolerance crossing that exits before trace end without
a 5 s dwell. RUNTIME_TERMINATION = the two known single_obstacle planner crashes (pairs 04/05).
TIMEOUT_PROGRESSING = cap-reaching run whose terminal-120 s best-error improvement exceeds
1 cm (pos) or 0.05 rad (yaw). TRUE_STALL = remaining cap-reaching runs.

**Counts: SUCCESS 5, TRANSIENT_SUCCESS 1, TIMEOUT_PROGRESSING 6, TRUE_STALL 16,
RUNTIME_TERMINATION 2.**

## 6. Per-run one-liners (surprises / context)

- open_task_pair01 — the only genuine TRANSIENT: latched at 38.1 s, exited after 0.3 s via yaw
  overshoot (final e_yaw 0.130 rad, final e_pos 0.021 m); best-ever errors 0.021 m / 0.0009 rad.
- open_task_pair02-05 — SUCCESS at 79.1/125.8/60.2/114.0 s, inside tolerance until launcher
  stop (dwell censored at 2.6-2.9 s).
- shelf_gap_pair01 — the only non-open-task success (186.2 s); dwell censored at 3.2 s.
- shelf_gap_pair05 — best failure of the campaign: best e_pos 0.074 m / e_yaw 0.060 rad, still
  improving in the terminal window (TIMEOUT_PROGRESSING); came within 2.4 cm of the pos gate.
- icra_sign (all 5 fail) — 4/5 TIMEOUT_PROGRESSING; pairs 02/04 reached e_yaw 0.056/0.134 rad
  with pos still 0.24-0.28 m out.
- icra_sign_pair05 — SURPRISE: 80.3% contact fraction (highest in campaign) but only 14.5%
  productive — sustained pushing that mostly does not reduce error (grinding against the C-glyph
  / obstacle); TRUE_STALL and the lone threshold-sensitivity boundary case.
- slalom_pair01 — 12.2% contact but only 7.6% productive (campaign-lowest productivity).
- single_obstacle_pair04/05 — planner crash at t~15/21 s (known); the sim + recorder kept
  logging to ~365 s with the object untouched thereafter (best e_pos ~0.60 m frozen); the
  ~0.4 contact fraction in their truncated CSVs is pre-crash-era data only. Labeled
  RUNTIME_TERMINATION; their trace durations reflect post-crash coasting, not control.
- TRUE_STALL cluster signature: contact fraction 0.8-6% (object essentially never engaged after
  an early failed approach) with e_yaw parked near 2.4-3.0 rad — the far-yaw goal half of each
  pair looks systematically unreachable for C3+ in obstacle scenes.
- Contact acquisition: SUCCESS runs show 10-18 episodes/min with ~0.8-1.1 s mean episodes;
  TRUE_STALL runs mostly <2/min (12/16 runs; exceptions icra_sign_pair05 9.4, ycb 03 5.2,
  slalom 01 6.1, slalom 02 4.7 — contact without productivity). See stage1_outcomes.csv.
