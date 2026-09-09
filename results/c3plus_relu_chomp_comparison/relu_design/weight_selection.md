# ReLU obstacle ranking cost — declared parameters (CHECKPOINT A)
Declared 2026-09-08 BEFORE any live campaign. No post-hoc weight sweeps.

## Formula (ranking layer only; never in the C3 QP)
d_k = min signed distance of the orientation-aware object footprint to every
obstacle at rollout knot k (`RouteFootprintSdf`: footprint boundary points ×
exact obstacle SDF — boxes/polys/discs; positive outside, 0 at contact,
negative inside).

J_obs_relu = w_obs · Σ_k max(0, (ε − d_k)/ε)²   over the N+1 = 11 rollout knots.

## ε = 0.01 m
The benchmark defines no source-backed margin for ranking; 0.01 m is the
task-brief default and matches the controller's existing `c3_min_clearance:
0.01` sampling margin — the one in-repo precedent for "close enough to care".

## w_obs = 200 (declared)
Scale audit against the terms J_rank actually compares (cost_type-5 rollout,
object task terms; observed candidate-cost differences O(10²–10³); Task-A
reconstruction: J_C3_task ≈ 10²–10⁴ over a run):

| d | r(d)² per knot | Σ over 11 knots × w=200 |
|---|---|---|
| +50 mm | 0 | 0 |
| +20 mm | 0 | 0 |
| +10 mm | 0 | 0 |
| +5 mm | 0.25 | 550 |
| 0 mm | 1.0 | 2 200 |
| −5 mm | 2.25 | 4 950 |

Rationale: zero influence outside ε (no path-length inflation, no change to
obstacle-free candidate ordering — open_task ranking is bit-identical);
inside ε the penalty reaches the same order as typical inter-candidate task-
cost differences at contact (2.2k) and dominates under penetration (≈5k at
−5 mm), while staying far below the old exp term's ~18k at 44 mm (which
over-penalized whole regions). For comparison the old center-based exp term
is INACTIVE in the lcs_contact baseline, so the experimental contrast is
"no ranking obstacle term" vs "footprint ReLU term".

## Activation (env-gated; default path compile-identical to frozen baseline)
SAMPLING_C3_RANK_OBS_MODE=relu_footprint
SAMPLING_C3_OBS_RELU_EPS=0.01
SAMPLING_C3_OBS_RELU_W=200
