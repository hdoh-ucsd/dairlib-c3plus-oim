# Long-sign-row compatibility audit (§11)

The sign spans y ∈ [−0.60, +0.66] (glyph 6's hull reaches y = 0.6 + 0.06),
~1.26 m along the table. Audit of every size/band assumption in the current
stack:

| assumption | where | verdict |
|---|---|---|
| obstacle array sizes | `scenario_params.obstacles` (std::vector), `ObsCfg().obs_boxes/obs_polys` | fully dynamic — 8 obstacles load; no fixed count anywhere |
| LCS size vs obstacle count | `n_obs_slots` N_closest (default 2) | by design independent of scene obstacle count; slots select the 2 smallest-φ obstacles per tick — identity transitions logged in `obstacle_lcs_contacts.csv` |
| \|y\| ≤ 0.3 band | nowhere in the C++ stack (that band is an OIM layout *convention*, and icra_sign is upstream's own exception) | no code assumption found |
| workspace limits | `workspace_limits` y ∈ [−0.6, 0.6] (EE, planner QP) | constrains the PUSHER, not obstacles. The C's corridor (+0.4 → −0.4) and the goal slot (−0.40) are well inside. EE cannot reach beyond glyph 2b/6 (y ≥ 0.45), which the task never requires. NOT changed. |
| `robot_radius_limits` [0.25, 0.75] | hard abort ring | goal at r = 0.64; glyph I at r ≈ 0.74 — inside; task feasible. NOT changed. |
| sampling grids `grid_x/y_limits` ±0.11 | object-relative projection grid | object-relative, scene-size independent |
| swept-veto obstacle top `obs_top_z` default 0.12 | P5 PWL veto | glyph tops are at z = 0.046 world; the 0.12 default treats the sign as 7 cm taller than real, so lateral repositioning legs OVER the sign get vetoed. Conservative, never unsafe. The run recipe sets `SAMPLING_C3_OBS_TOP_Z=0.046` (existing documented env knob, scene configuration). |
| visualization | vis_params ranges | cosmetic only |

**No silent omission/clipping path exists**: obstacle SDF/witness loops run
over the full obstacle vector; the only reduction is the intended N_closest
LCS slot selection. No controller-semantics changes were needed for the long
row; the one scene-level adjustment is the `SAMPLING_C3_OBS_TOP_Z` value.
