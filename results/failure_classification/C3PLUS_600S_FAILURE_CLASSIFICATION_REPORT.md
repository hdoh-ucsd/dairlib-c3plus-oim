# xArm6 C3+ 600 s failure-classification report (30 runs, 6 scenes × 5 start/goal pairs)

Inputs: `stage1_outcomes.csv`, `terminal_progress.csv`, `outer_loop_summary.csv`,
`c3_segment_audit.csv`, per-run data under `results/xarm6_c3plus_scene_smoke/runs/`.
Full per-run assignments: `failure_audit.csv`; per-scene rollup: `failure_summary_by_scene.csv`;
outcome × cause table: `causal_confusion_matrix.csv` / `.png`. Solver mode is C3+ (RUN-META
solver mapping; `mode=c3` is only the dispatcher string).

## The 16 questions

**1. How many official successes?** 6 of 30 latched the official (any-single-step) criterion:
open_task pairs 01-05 and shelf_gap pair01. Under the outcome taxonomy 5 are SUCCESS and 1
(open_task pair01) is TRANSIENT_SUCCESS. Official task SR = 6/30 = 20% latched, 5/30 = 16.7%
excluding the transient.

**2. How many sustained (5 s dwell) successes?** Zero — but this is **measurement censoring,
not controller behavior**: the launcher stops every run ~2.5-3.2 s after the SUCCESS latch, so
a 5 s dwell is unmeasurable for any success in this campaign. The 5 dwell-censored successes
held tolerance until trace end (post-latch pos ptp median 0.0005 m); they are counted SUCCESS.

**3. Transient successes?** 1 — open_task pair01, which latched at 38.1 s and exited 0.3 s later
via yaw overshoot to 0.130 rad (F5; see failure_card_F5).

**4. Timeout-but-progressing runs?** 6: icra_sign pairs 01-04, shelf_gap pair05,
single_obstacle pair03. All improve beyond 1 cm / 0.05 rad in the terminal-120 s window;
shelf_gap pair05 came within 2.4 cm of the position gate.

**5. True stalls?** 16: single_obstacle 01/02, shelf_gap 02/03/04, ycb_clutter 01-05,
slalom 01-05, icra_sign 05.

**6. Outer-loop phantom-churn dominated (F2 primary)?** 13 of the 16 true stalls:
single_obstacle 01/02, shelf_gap 02/03, slalom 01-05, ycb_clutter 01/03/04/05. Phantom-churn
runtime fractions run 0.58-0.94 in the churn exemplars; the "park" variants (single_obstacle 02,
slalom 05) show the same 126-141-entry log churn with a physically motionless tip.

**7. Contact-acquisition dominated (F1 primary)?** 2: ycb_clutter pair02 (98.5% of C3 episodes
zero-contact) and shelf_gap pair04 (69% zero-contact, weaker churn). F3 (failure-memory
reselection) is secondary on 12 runs — equivalent-sector reselection 0.72-0.97 sustains both
loops — but is never causally first.

**8. Genuine local-C3 fixed points (F4)?** Zero as primary. The only candidate, icra_sign
pair05 (80% contact, 14.5% productive), has median obstacle clearance ≈ -0.0001 m with 98% of
contact steps within 5 mm of the C-glyph hull: the object is geometrically blocked (F6 primary),
with F4 only secondary (C3 keeps pushing into an unmodeled wall).

**9. Execution/prediction mismatches (F5)?** 1 primary: open_task pair01's terminal yaw
overshoot. No other run got close enough to the goal for terminal mismatch to matter.

**10. Workspace / stability / runtime failures?** 2 workspace crashes (F7):
single_obstacle pairs 04/05, the `CheckForWorkspaceLimitViolations` assert at
`sampling_based_c3_controller.cc:3397` during reposition at 15.3 s / 21.2 s (F10 secondary).
No topples (F8): the transition-vocabulary census found zero topple/exhaustion lines. No other
runtime or numerical failures.

**11. Which failures are scientifically valid evidence against C3+ (the local contact
solver)?** At most one, partially: icra_sign pair05, where C3+ held rich contact for ~176 s
without progress — and even there the primary cause is obstacle geometry (clearance ~0), so
the defensible criticism is "C3+ does not recognize/route around a geometric block", not "C3+
cannot push". No run in the campaign shows C3+ failing while in unobstructed productive contact.

**12. Which failures must NOT be read as local-C3 incapability?** The 15 F1/F2 true stalls
(contact fraction 0.8-9%: C3+ was never given a contact), the 2 F7 crashes (planner dead after
15-21 s), the 6 F12 timeouts (progressing when capped), and the F5 transient (goal reached,
then overshot). That is 24 of the 25 non-success runs.

**13. Scenes with highest outer-loop failure fraction?** ycb_clutter (5/5 outer-loop primary;
median phantom churn 0.83, C3 time 96-99% contactless) and slalom (5/5; median churn 0.58,
97% contactless), then shelf_gap (3/5). single_obstacle is 2/5 outer-loop plus the 2 crashes.

**14. Scenes failing after confirmed physical engagement?** Only icra_sign: pair05 stalled
after driving yaw to 0.0066 rad (block against the C-glyph), and pairs 01-04 were still
progressing through real contact when capped. Every other failing scene fails *before*
meaningful engagement (contact fractions ≤ 9%, mostly ≤ 3.5%).

**15. Runs deserving targeted replay?**
- **shelf_gap pair05** with a longer cap (was 2.4 cm from the gate, still improving) — cheapest
  possible additional success datapoint.
- **icra_sign pairs 02/04** likewise (best e_yaw 0.056/0.134 rad, pos still closing).
- **single_obstacle pair04 or 05** under a debugger/clamped sampler to root-cause the
  cc:3397 workspace assert (reproduces in ≤ 21 s).
- **icra_sign pair05** with the goal or C-glyph shifted, to separate F6 (geometry) from F4
  (solver fixed point) — the one open capability question.
- **open_task pair01** with the launcher's post-latch stop lengthened, to see whether the F5
  overshoot self-recovers (also fixes the dwell-censoring defect for all future successes).

**16. Does the 600 s cap expose capability, throughput, or outer-loop defects?** Primarily
outer-loop defects: 15/25 non-successes are F1/F2 loops that would not resolve at any cap.
Secondarily throughput: 6/25 are F12 runs the cap censors mid-progress (plus successes that
took 60-186 s, i.e. throughput is marginal even when it works). Genuine capability evidence
against the local C3+ solver is essentially absent (≤ 1 run, and there geometry-confounded).

## FINAL VERDICT
- **Task SR:** 6/30 latched official (20%); 5/30 excluding the transient; 0/30 sustained-5s —
  unmeasurable by construction (launcher dwell censoring), not a controller finding.
- **True stalls:** 16/30.
- **Outer-loop-attributable (F1+F2 primary, F3 secondary):** 15/30 (13 F2 + 2 F1) — 94% of all
  campaign repositions are "not making progress in C3" exits; the designed cost-threshold C3
  exit fired only 15 times in 30 runs.
- **Local-C3-attributable (F4 primary):** 0/30 (1 geometry-confounded candidate, icra_sign
  pair05, classified F6 with F4 secondary).
- **Timeout-progressing:** 6/30.
- **Workspace/runtime:** 2/30. **Execution mismatch:** 1/30. **Unknown (F13):** 0/30.
