# Outer-loop episode audit — xArm6 C3+ 30-run scene-smoke campaign

Analyst script: `analyze_outer_loop.py` (this directory). Read-only over
`results/xarm6_c3plus_scene_smoke/runs/<scene>/pair0N/`. Python: `/root/miniconda3/bin/python3`
(the named env `push_anything_ADMM` lacks pandas; base conda env used instead).

## 1. Transition vocabulary census (all 30 planner.log files, exact substring counts)

| line (digits masked) | total |
|---|---|
| `Switching to C3 because reached repositioning target` | 1796 |
| `Repositioning after not making progress in C3` | 1693 |
| `Repositioning because found good sample` | 161 |
| `Crossed cost switching threshold.` | 15 |
| `abort: ... CheckForWorkspaceLimitViolations(): condition 'std::pow(lcs_x_curr...` | 2 |

No topple/exhaustion/other exit lines exist. The 2 workspace aborts are exactly
single_obstacle pair04 (15.3 s) and pair05 (21.2 s) — the known early crashes; both were
processed normally on their truncated CSVs.

Headline ratio: **94% of all repositions campaign-wide are "not making progress in C3"**
(1693 vs 161 good-sample). Only 15 of 1796 C3 entries ever crossed the cost switching
threshold; icra_sign and open_task/shelf_gap pair01-05 each have exactly 0 or 1.

## 2. Behavioral segmentation rule (v2, dwell-vs-transit)

A pure gap threshold failed validation: several runs (single_obstacle p01/p02, slalom p03-05,
ycb p01/p04) hover **stationary at a constant 1.7–7.5 cm gap** for essentially the whole run,
so "C3 = gap < 1 cm" found almost no episodes despite 100+ logged C3 entries.

Final per-step rule (hysteretic state machine over the metrics CSV):
- tip speed = 11-step centered rolling median of planar tip velocity;
- **enter C3-LIKE** when `pusher_object_gap < 0.010 m` OR tip speed `< 0.015 m/s` (dwell);
- **exit to REPOSITION-LIKE** only when tip speed `> 0.03 m/s` and not in near-contact;
- episodes separated by transits `<= 15` steps are merged; episodes `< 60` steps dropped.

Validated by parameter sweep (`sweep.py`) on 6 representative runs against log
`c3_entry_count`; chosen point gives reconciliation ratio ~0.7–1.9 in 4 of 6 scenes.

### Reconciliation quality
- Within/near ±30%: all icra_sign, ycb p01/p02/p04, single_obstacle p03, shelf p04/p05, slalom p01.
- Systematic +60–95% over-count in open_task (behavioral rule splits one logged C3 stint at
  brief micro-transits; open_task runs terminated early on success/timeouts at 40–130 s).
- **Irreducible under-count (ratio 0.03–0.24) in single_obstacle p01/p02, slalom p03-p05,
  ycb p03/p05**: the log records 100–142 C3 entries but the tip physically parks at a fixed
  standoff. The outer loop flips mode flags ~140 times with *no corresponding tip travel* —
  no behavioral rule can recover those; the divergence is itself part of the pathology
  (mode churn without physical repositioning).

## 3. Per-scene medians (final numbers, from `outer_loop_summary.csv`)

| scene | phantom_churn_runtime | contactless_C3 | acquisition_failure | equiv_failed_reselection | reconciliation |
|---|---|---|---|---|---|
| icra_sign | 0.099 | 0.795 | 0.429 | 0.800 | 1.16 |
| open_task | 0.228 | 0.678 | 0.462 | 0.667 | 1.86 |
| shelf_gap | 0.369 | 0.804 | 0.635 | 0.904 | 1.43 |
| single_obstacle | 0.262 | 0.793 | 0.750 | 0.200 | 1.01 |
| slalom | 0.581 | 0.972 | 0.818 | 0.778 | 0.24 |
| ycb_clutter | 0.826 | 0.965 | 0.833 | 0.750 | 0.93 |

Definitions: phantom_churn = fraction of run wall-time inside C3-like episodes that are both
>70%-contactless AND no-progress; acquisition_failure = fraction of C3 episodes with zero
contact steps; equiv_failed_reselection = fraction of episodes whose object-frame approach
sector (8 bins, tip-minus-object rotated by -yaw at episode start) matches a prior
no-progress episode's sector.

## 4. Strong F2 signature runs (contactless C3 + stationary object + repeated repositions + sector reselection)

Ranked by phantom_churn_runtime_fraction:
1. **single_obstacle/pair01 — 0.937** (contactless 0.988, acq-fail 0.923; 142 log entries, tip parked at 7.5 cm gap essentially the whole 327 s)
2. **slalom/pair04 — 0.932** (contactless 0.982, parked at ~1.7 cm)
3. **ycb_clutter/pair01 — 0.858**, **ycb/pair05 — 0.851**, **ycb/pair03 — 0.826**
4. **slalom/pair03 — 0.793**, **ycb/pair04 — 0.691** (acq-fail 0.977, reselection 0.966)
5. **shelf_gap/pair03 — 0.624** (acq-fail 0.990, reselection 0.962 — the purest churn-loop: 105 episodes, 104 no-progress)
6. **slalom/pair02 — 0.581**, **ycb/pair02** (acq-fail 0.985, reselection 0.924)

Textbook churn-cycle exemplars (many discrete episodes, near-total acq failure, >90% sector
reselection): shelf_gap/pair03, ycb_clutter/pair02 & pair04, slalom/pair02.
Park exemplars (single endless contactless hover, log still churning): single_obstacle
pair01/02, slalom pair04/05, ycb pair05.

Low-F2 runs: icra_sign/pair05 (87% contact inside C3 — the pusher genuinely engages;
its failure is elsewhere), open_task pair01/03/05 (small errors, near-success),
shelf_gap/pair01 & pair05 and single_obstacle/pair03 (tight final e_pos 0.04, mixed).

## 5. Surprises

- **ycb_clutter and slalom C3 time is 96–99% contactless** — the mode named "C3" almost never
  touches the object in these scenes; the controller optimizes against a believed contact
  that does not exist (matches the prior forensics' optimistic-contact-belief mechanism).
- **The log/behavior split is bimodal**: half the campaign churns (discrete
  reposition→hover→reposition episodes), the other half *parks* — the outer loop keeps
  logging `Switching to C3` / `Repositioning after not making progress` at high rate while
  the tip is physically motionless. In park runs the "reposition" never moves the tip, so
  reselection equivalence is trivially satisfied at the same standoff.
- Sector-level reselection equivalence is high nearly everywhere churn occurs (0.72–0.97):
  after a no-progress exit the next approach is drawn from an already-failed object-frame
  sector in ~3 of 4 cases, confirming the equivalent-standoff-reselection leg of F2.
- `Repositioning because found good sample` is concentrated in the healthier runs
  (icra p02/p05, single_obstacle p03 with 83) and absent from every slalom/ycb run.
- Cost-threshold crossings (the designed C3-exit) are essentially unused: 15 in 30 runs.

## Outputs
- `c3_segment_audit.csv` — 1211 C3-like episodes.
- `outer_loop_summary.csv` — 30 rows (single_obstacle p04/p05 included from truncated data).
- `vocab_census.json` — full masked-line census.
