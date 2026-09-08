# Failure classification — xArm6 C3+ scene campaign (600 s tier), 24 failures

Data: `runs/<scene>/pair0N/*_metrics.csv`; roster: `metrics/pair_campaign_summary.csv`.
Per-trial features + class: `metrics/failure_classification_600s.csv`. Grid figure: `figures/failure_classes.png`.
Decision rules applied top-down, first match wins; final window W = last 25% of control steps.
Note: metrics CSVs end at sim_time ~300-390 s (wall-cap tier), except two planner crashes at ~15/21 s.

## Class counts

| Class | n | Trials |
|---|---|---|
| OBSTACLE_BLOCKED | 18 | icra 02-05; shelf 02-04; single 01-03; slalom 01,03-05; ycb 01,03-05 |
| EARLY_ROTATION_GRIND | 4 | single 04\*,05\* (\*planner crash), slalom 02, ycb 02 |
| SLOW_MONOTONE_PROGRESS | 1 | icra 01 |
| OTHER_PLATEAU | 1 | shelf 05 |

No trial matched NEVER_ACQUIRED, ROTATION_STALL, LAST_DECIMETER_STALL, or OSCILLATION_CHURN.

## Per-scene narratives

**single_obstacle (0/5, 3 blocked + 2 crashes).** Pairs 01-03 push briefly in the first ~10 s (last >5 mm progress at t = 9.8-10.3 s), then park with median W clearance <= 1 mm and contact fraction 1-13% — the object is literally wedged against the obstacle for ~95% of the run, with yaw error still ~2.9-3.0 rad for 01/02 (block happened before the first rotation) while 03 reached best_ang 0.12 rad first. Pairs 04/05 are not behavioral failures at all: the planner aborted (core dump) at t ~15/21 s on the `CheckForWorkspaceLimitViolations` inner-radius assertion (`sampling_based_c3_controller.cc:3397`, EE inside `robot_radius_limits[0]`) during a reposition burst.

**shelf_gap (1/5).** Pair 01 latched at 186.2 s. Pairs 02-04 stall against the shelf almost immediately (last progress t = 12-18 s, W clearance 17-30 mm, contact fraction 2-9%, yaw stuck near pi) — same wedge-and-park signature as single_obstacle, before any rotation work. Pair 05 is the outlier OTHER_PLATEAU: it nearly succeeded (best 0.043 m / 0.049 rad, final 0.044 m / 0.224 rad, last progress 177 s, W clearance 0.23 m — clear of the shelf) and then sat just outside the 0.05 m/0.1 rad latch with the yaw drifting back out; it is a near-miss endgame plateau, not a geometric block.

**ycb_clutter (0/5, 4 blocked + 1 grind).** All five show the same shape: a single early push (last progress at t = 3.6-15.3 s), then flat pos error ~0.52-0.77 m and yaw pinned near pi (2.0-3.1 rad final; whole-run ang progress fraction -0.13 to +0.29) for the rest of the run, with contact fraction 1-6%. Four have median W clearance 0-26 mm (blocked in clutter); pair 02 sits at 39 mm so it classifies as EARLY_ROTATION_GRIND, but mechanistically it is the same never-got-started stall. Blocked-vs-grind here is a threshold artifact, not two mechanisms.

**icra_sign (0/5).** Failures here are late-stage, not early: contact fractions up to 80% and best errors deep into the task (pair 05 best_ang 0.007 rad; pair 04 best_pos 0.21 m). Pairs 02-05 end parked against the C-glyph hull (median W clearance 0-12 mm, pos progress in W < 3 mm) at final pos 0.21-0.74 m; pairs 04/05 additionally lose rotation late (ang progress in W negative, pair 05 final ang 2.38 rad after having been at 0.007 — the object was knocked/rotated away after near-completion). Pair 01 was still monotonically improving at cutoff (+0.072 m pos, +1.06 rad ang in W, last progress at t = 306.6 s = end of log): it ran out of cap, not out of mechanism.

**slalom (0/5).** Identical to ycb: one early push (last progress t = 5-26 s), then a full-run park with yaw near pi (final ang 2.69-3.10 rad, whole-run ang progress fraction -0.12 to +0.06) and contact fraction 1-12%. Four of five have median W clearance 13-29 mm (against a slalom pillar); pair 02 at 32 mm falls to EARLY_ROTATION_GRIND by threshold only. The pi-flip rotation demand is never even started in this scene.

**open_task (5/5).** All success_trace True (38-126 s); two latched-then-overshot rows excluded per protocol.

## What would raise SR (diagnosis only, from the metrics)

- **The dominant failure (17-19 of 24) is a single signature:** one push in the first ~10-25 s, then a 280+ s park with near-zero contact fraction, near-zero position progress, and (in cluttered scenes) yaw still at ~pi. Whatever ends that first push never re-engages: `last_progress_t` medians of ~10 s against 300+ s caps mean ~97% of the budget is spent parked. Re-acquisition after the first obstacle-adjacent stall is the single highest-leverage phase in the data.
- **Blocked geometry, not blocked dynamics:** in the parked window the object sits 0-30 mm from an obstacle while `evaluation_total` stays high and flat (eval_W within ~±10% of run mean in most trials) — the cost sees no gradient it can act on from the wedged pose. Trials where clearance stayed large (shelf 05, icra 01) got to near-tolerance, confirming clearance-in-W is the discriminating variable.
- **icra_sign is a different, later failure:** high contact fraction and near-tolerance bests, with regression after best (pair 05: 0.007 rad best -> 2.38 rad final). Protecting near-goal states from being pushed back into the glyph hull (the 0-clearance park) is what separates its 0/5 from success; pair 01 needed only more time.
- **Two crashes cost two trials outright:** the single_obstacle 04/05 inner-radius workspace assertion is a hard abort at ~15-20 s; those trials never got a fair 600 s.
- Half the pi-flip trials (ycb/slalom/single with init ang ~2.7-3.1) show negative whole-run angular progress — the early push makes yaw worse before the park. The initial pi-flip demand plus clutter is the compound that no trial in these three scenes survived.
