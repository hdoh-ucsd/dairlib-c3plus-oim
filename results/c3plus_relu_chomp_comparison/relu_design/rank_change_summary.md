# Checkpoint B — Offline candidate-ranking audit (footprint-ReLU obstacle cost)

Date: 2026-09-08. Script: `offline_ranking_audit.py` (this directory). Raw per-step rows:
`offline_candidate_ranking.csv` (5,000 rows = 5 scenes x 5 pairs x 200 subsampled steps).

## Setup

- Data: 600 s tier `runs/<scene>/pair0N/*_metrics.csv` recorded object poses + goals; open_task skipped.
- Candidates are SYNTHETIC: recorded pose + 8 perturbations (+-2 cm x, +-2 cm y, +-0.2 rad yaw,
  and the two coupled diagonals). No real per-candidate logs exist (CostLogger env unset).
- Costs (per single knot; a rollout would multiply each obstacle term by ~11x uniformly, which
  rescales but does not reorder within a fixed comparator):
  - old_obs = Sum_obstacles 5000 * exp(-d_center / 0.04), d_center = hypot to fallback-disc center - r
    (disc lists from each scene's scenario_params.yaml, robot base disc included).
  - new_obs = 200 * max(0, (0.01 - d_fp)/0.01)^2, d_fp = min over footprint boundary points
    (<=2 mm spacing, pose-transformed) of exact obstacle SDF (polys/boxes/discs incl. robot base).
  - task proxy = 10000*((x-gx)^2+(y-gy)^2) + 510*wrap(yaw-gyaw)^2.
- Comparators: A = task+old (historical non-lcs term), C = task only (true frozen lcs_contact
  baseline). Variant B = task+new. Metrics over the 9-candidate set per step.

## Per-scene ranking comparison (mean over 1000 steps/scene)

| scene | Spearman B vs old | top-1 change vs old | top-3 overlap vs old | Spearman B vs baseline | top-1 change vs baseline | top-3 overlap vs baseline |
|---|---|---|---|---|---|---|
| single_obstacle | 0.779 | 72.6% | 0.693 | 0.552 | 58.0% | 0.759 |
| shelf_gap       | 0.762 | 63.7% | 0.798 | 0.886 |  0.8% | 0.887 |
| ycb_clutter     | 0.630 | 97.5% | 0.613 | 0.958 |  1.5% | 0.879 |
| slalom          | 0.474 | 76.7% | 0.592 | 0.955 |  0.0% | 0.938 |
| icra_sign       | 0.733 | 58.8% | 0.695 | 0.495 | 30.9% | 0.661 |

Reading: vs the TRUE baseline (no obstacle term) the ReLU cost is surgical — it reorders
candidates essentially never in shelf_gap/ycb/slalom (0-1.5% top-1 change; term is exactly 0
beyond 10 mm clearance) and intervenes heavily only where recorded trajectories actually hug
geometry (single_obstacle 58%, icra_sign 31%). vs the OLD center-exp term the rankings differ
strongly everywhere (59-98% top-1 change) because the exp term is never zero and is driven by
inflated disc centers, i.e. the old term was reordering candidates even in free space.

## Motivating defect: center-safe but footprint-unsafe

Fraction of recorded steps with min center clearance > 25 mm while footprint clearance < 10 mm:

| scene | mismatch fraction |
|---|---|
| single_obstacle | 3.0% |
| shelf_gap | 0.8% |
| ycb_clutter | 19.3% |
| slalom | 0.1% |
| icra_sign | 61.4% |

(icra_sign is dominated by the C glyph operating flush against concave glyph hulls that the
circumscribed discs grossly over/under-cover; ycb hulls similarly. Slalom's mismatch is tiny
because the circumscribed fin discs are conservative, i.e. its defect is the opposite sign —
phantom cost in truly free corridors, visible in the 76.7% top-1 change vs old.)

Flagship (single_obstacle pair01, t = 316.7 s stall): center clearance 24.1 mm ("safe" by the
25 mm rubric and only mildly penalized by exp) while true footprint clearance is 0.67 mm —
essentially touching. old_obs = 2739 (spread thinly over all candidates), new_obs = 174 at the
recorded pose; top-1 candidate changes under the new term vs BOTH comparators at this step.

## Sanity

- new cost verified numerically 0 in all 45,000 candidate evaluations with d_fp >= 10 mm
  (0 violations).
- 0 NaN across task/old/new/d_fp arrays.

## Limitations

- Synthetic candidate sets (fixed +-2 cm / +-0.2 rad star), not the planner's real sampled
  candidates; real sets cluster near contact and may inflate or deflate change rates.
- Single-knot obstacle cost proxy; the real ranking integrates ~11 knots of a rollout, and
  rollout states differ across candidates (uniform-multiplier assumption is approximate).
- Task-cost proxy weights (10000/510) approximate, not the exact ranking objective.
- No real logged candidate sets existed (CostLogger unset), so this is an offline
  reconstruction, not a replay.
