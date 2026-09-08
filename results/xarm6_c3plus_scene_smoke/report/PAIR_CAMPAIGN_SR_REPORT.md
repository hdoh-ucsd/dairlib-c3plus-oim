# xArm6 C3+ pair campaign — SR report (2-lane validated tier)
2026-09-08 · branch `feature/oim-scene-sync-metrics` · 30 trials
(6 scenes × paired starts/goals 1–5, upstream OIM d6d80a6 poses).

**Supersedes the erased 5-lane tier**, whose SRs were invalid: 5 concurrent
trios starved the planner to ~17 solves/s while the sim ran realtime
(root cause in `OPEN_TASK_FAILURE_INVESTIGATION.md`). This tier reran ALL 30
cases with **2 lanes**, and every trial logs a `solves_per_s` validity metric
in `metrics/pair_campaign_summary.csv`.

## Protocol
200 s cap; success = pos < 0.05 m AND |yaw err| < 0.1 rad (upstream xarm6
tolerance); matched physics; exact planner geometry (base disc + boxes/hulls);
frozen C3+ controller; 2 isolated lanes (unique udpm ports 7941+).

## Success rates

| scene | **SR** | mean solves/s | notes |
|---|---|---|---|
| open_task | **2/5** | 57 | p1 S @ 29.3 s; p2 S @ 70.0 s; p4 near-miss 0.0151 m / **0.1011 rad** (0.001 rad out at cap); p5 0.034 m/0.33 rad |
| single_obstacle | 0/5 | 34 | p4 starved (5.3 solves/s — low validity); others 0.41–0.51 m |
| shelf_gap | 0/5 | 33 | p1 starved (5.6 solves/s); p5 near-miss 0.245 m/**0.075 rad** |
| ycb_clutter | 0/5 | 38 | uniform 0.53–0.77 m, ang ≈ π: acquisition+first rotation eats the cap |
| icra_sign | 0/5 | 24 | p1 best-of-campaign 0.139 m/0.070 rad; rotation solves, slot translation stalls |
| slalom | 0/5 | 38 | 0.53–0.75 m, ang ≈ 3: never past gate 1 within cap |
| **overall** | **2/30 (6.7%)** | 37 | |

## Per-trial finals (pos m / ang rad; S = success)

| scene | p1 | p2 | p3 | p4 | p5 |
|---|---|---|---|---|---|
| open_task | **S 29.3 s** | **S 70.0 s** | 0.175/0.387 | 0.015/0.101 | 0.034/0.328 |
| single_obstacle | 0.414/2.749 | 0.434/2.917 | 0.498/0.731 | 0.581/3.120* | 0.507/0.608 |
| shelf_gap | 0.576/2.907* | 0.462/2.570 | 0.595/3.131 | 0.592/2.837 | 0.245/0.075 |
| ycb_clutter | 0.559/3.034 | 0.770/3.094 | 0.556/3.102 | 0.554/3.133 | 0.526/2.015 |
| icra_sign | 0.139/0.070 | 0.392/1.388 | 0.619/0.723 | 0.233/1.125 | 0.187/1.387 |
| slalom | 0.752/3.096 | 0.752/2.998 | 0.531/3.002 | 0.526/2.897 | 0.726/2.660 |

\* low-validity trials (solves/s ≈ 5 — transiently starved; rerun before quoting).

## Reading
1. **Load fix worked where load was the problem**: open_task went 1/5 → 2/5
   with two more hair-width misses; pair 2 now succeeds at 70.0 s (benchmark:
   52.9/83.4 s). Trial-to-trial draw variance (Ipopt FP floor — known
   non-determinism) explains p4/p5 flipping between success (solo probe:
   112.7 s / 75.7 s) and near-miss here.
2. **Obstacle scenes are genuinely capped by solve dynamics, not load**: at a
   healthy 33–38 solves/s, ycb/slalom still end ≈π from the goal orientation —
   the 200 s cap ends during acquisition/first rotation. These SRs are
   floor estimates; a ≥600 s cap tier is the necessary next step.
3. **icra_sign** consistently solves the C's rotation (p1 within 0.07 rad and
   0.139 m) — the remaining work is the last-decimeter translation into the
   slot between glyphs.
4. Two trials (shelf p1, single p4) hit ~5 solves/s — transient contention
   (likely overlap with a straggling neighbor trio) — and should be rerun
   before their cells are quoted.

## Provenance
- Per-trial packages: `runs/<scene>/pair0N/` (video, per-control-step metrics
  CSV, two-panel eval graph, result JSON, manifest).
- Table: `metrics/pair_campaign_summary.csv` (incl. `solves_per_s`).
- Historical open_task successes for cap context: 34.7–112.7 s across
  benchmark + this session; max 112.7 s (solo pair 4).
