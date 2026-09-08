# xArm6 C3+ pair campaign — full report with success rates
2026-09-07 · branch `feature/oim-scene-sync-metrics` @ ced7396df · 30 trials
(6 scenes × paired starts/goals 1–5, upstream OIM d6d80a6 poses).

## Protocol
- 200 s sim cap per trial (smoke tier); success = position error < 0.05 m AND
  orientation error < 0.1 rad simultaneously (upstream xarm6.yaml tolerance).
- Matched physics (`--matched_mu`), exact planner geometry (base disc + exact
  boxes/hulls via env), frozen C3+ controller, 5-joint xArm6, OIM stick EE.
- 5 parallel isolated lanes (unique udpm ports, per-run dirs/tmp/pgroups).
- Per-trial artifacts (video, per-control-step metrics CSV, eval graph,
  result JSON, manifest) under `runs/<scene>/` (pair 1) and
  `runs/<scene>/pair0N/` (pairs 2–5). Table: `metrics/pair_campaign_summary.csv`.

## Success rates (200 s cap)

| scene | **SR** | final pos <0.05 m | best pos <0.05 m | best ang <0.1 rad | mean final pos (m) | mean final ang (rad) |
|---|---|---|---|---|---|---|
| open_task | **1/5** | 1/5 | 1/5 | 2/5 | 0.251 | 0.838 |
| single_obstacle | **0/5** | 0/5 | 0/5 | 0/5 | 0.445 | 2.186 |
| shelf_gap | **0/5** | 1/5 | 0/5 | 0/5 | 0.432 | 1.637 |
| ycb_clutter | **0/5** | 0/5 | 0/5 | 0/5 | 0.566 | 3.072 |
| icra_sign | **0/5** | 0/5 | 0/5 | 2/5 | 0.492 | 0.392 |
| slalom | **0/5** | 0/5 | 0/5 | 0/5 | 0.688 | 2.996 |
| **overall** | **1/30 (3.3%)** | 2/30 | 1/30 | 4/30 | 0.479 | 1.854 |

## Per-trial results (final errors at cap; S = success with time)

| scene | p1 | p2 | p3 | p4 | p5 |
|---|---|---|---|---|---|
| open_task | **S @ 49.0 s** (0.017/0.017) | 0.340/0.129 | 0.326/2.186 | 0.224/0.412 | 0.348/1.448 |
| single_obstacle | 0.308/1.523 | 0.345/2.721 | 0.496/1.544 | 0.582/3.125 | 0.494/2.018 |
| shelf_gap | 0.590/3.140 | **0.049/0.268** | 0.599/3.090 | 0.427/0.135 | 0.497/1.554 |
| ycb_clutter | 0.556/3.040 | 0.554/3.029 | 0.549/3.122 | 0.554/3.118 | 0.617/3.052 |
| icra_sign | 0.365/**0.021** | 0.623/0.451 | 0.631/0.645 | 0.248/0.688 | 0.596/0.156 |
| slalom | 0.640/2.907 | 0.755/3.020 | 0.751/3.074 | 0.502/3.120 | 0.793/2.861 |

(entries are final position error m / final orientation error rad)

## Reading
- **open_task**: the only outright success (pair 1, zero start-yaw jitter).
  Pairs 2–5 add start/goal yaw jitter; all four spend the cap mid-rotation
  (best-angle 2/5 under 0.1 rad shows rotation does complete sometimes, but
  never simultaneously with position).
- **shelf_gap pair 2** is the campaign's near-miss: position converged
  (0.0489 m < 0.05) with orientation at 0.268 rad — one more rotation burst
  short of success.
- **icra_sign** solves orientation well (mean final ang 0.392 rad, two trials
  under 0.1) but stalls 0.25–0.63 m from the slot — translation across the
  glyph row is the bottleneck, not rotation of the C.
- **ycb_clutter** and **slalom** are uniformly stuck near the start's
  orientation (≈π remaining) at 0.5–0.8 m: acquisition plus the first
  rotation consumes the entire 200 s in dense obstacle fields.
- Pattern matches the matched benchmark (bb55b1ec7: C3+ 6/10 open_table at a
  100 s cap only for the no-jitter pose family, 1/10 single_obstacle, 0/10
  shelf_gap): C3+ per-pair solve time is the dominant limit, not scene
  infidelity — every scene now runs the upstream-exact geometry.

## Caveats
- 200 s is a smoke-tier cap. All non-open scenes show monotone progress
  signatures at cap (see eval graphs); SRs here are lower bounds and should
  not be quoted as the method's converged SR. The intended follow-up campaign
  should raise the cap (≥600 s) before drawing comparisons.
- One trial per (scene, pair): no seed repetition; single-draw results.
- icra pair 4 uses a per-pair re-IK'd start arm pose (shared family pose
  clipped the rotated C at spawn); all other pairs share their scene's
  family start pose, mirroring the benchmark convention.
