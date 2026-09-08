# Data inventory — offline C3+ cost reconstruction (2026-09-08)

Scope: `results/xarm6_c3plus_scene_smoke/runs/<scene>/pair01..05` (6 scenes x 5 pairs = 30 runs),
plus `results/failure_classification/` and `tools/scene_smoke/scene_configs/`.

## 1. Availability matrix

All 30 run dirs contain ALL of: `*_metrics.csv`, `*_result.json`, `*_manifest.yaml`,
`steps_raw.jsonl`, `state_trace.jsonl`, `planner.log`, `sim.log`, `osc.log`, `recorder.log`,
`launcher.log`, mp4, eval png. Zero missing files; no per-run exceptions.

Row counts (metrics.csv, data rows):

| scene | p01 | p02 | p03 | p04 | p05 |
|---|---|---|---|---|---|
| icra_sign | 13324 | 14505 | 14021 | 14400 | 15990 |
| open_task | 2931 | 5925 | 9402 | 4955 | 8537 |
| shelf_gap | 13440 | 25846 | 22351 | 22515 | 23525 |
| single_obstacle | 24126 | 24085 | 25721 | **1030** | **1435** |
| slalom | 22530 | 22963 | 23609 | 23385 | 23351 |
| ycb_clutter | 23045 | 22583 | 23357 | 23116 | 22838 |

Truncations: **single_obstacle/pair04 ends at sim_time 15.42 s, pair05 at 21.25 s** (the two
crash runs). Their result.json files exist and are internally consistent
(`n_control_steps`/`sim_time_end` match the CSV tail), so they truncate cleanly — no partial rows.
open_task runs are short because they succeeded early, not truncated.

## 2. metrics.csv — exact columns (identical across all 30)

```
run_id, scenario, control_step, sim_time,
object_x, object_y, object_yaw, goal_x, goal_y, goal_yaw,
position_error_m, orientation_error_rad,
tip_x, tip_y, tip_z, tip_roll, tip_pitch, tip_yaw,
physical_contact_active, pusher_object_gap, min_obstacle_clearance,
goal_pos, goal_theta, obstacle, support, approach, align, tilt,
tip_z_cost, contact_z, pusher_obstacle, robot_contact, effort,
admm_penalty, evaluation_total
```

- Per-step object pose, errors, tip pose, `physical_contact_active`, `pusher_object_gap`,
  `min_obstacle_clearance`: **present in all 30 runs**.
- The 14 cost columns are **already an offline measured-state cost decomposition** produced by
  `tools/scene_smoke/postprocess_run.py`: instantaneous per-step block costs at the measured
  state (goal_pos = ramp*Q_POS*pos_err², goal_theta = ramp*Q_THETA*wrap(yaw_err)²,
  obstacle = W_OBSTACLE*sum(exp(-sdf/decay)) over footprint boundary samples).
  Note the **ramp factor** `min(1 + Q_RAMP_PER_STEP*k, Q_RAMP_MAX)` multiplies goal terms — so
  goal_pos grows with step index even at constant error; divide out the ramp to compare across time.
  `pusher_obstacle`, `robot_contact`, `admm_penalty` are NaN (NOT_COMPARABLE);
  `evaluation_total` is a subset sum over non-NaN blocks.

## 3. steps_raw.jsonl

Schema (all 6 scenes verified): `control_step, sim_time, robot_q[5], robot_v[5], robot_u[5],
objects: {OBJECT_G_shape_video_STATE_SIMULATION: [qw,qx,qy,qz,x,y,z]}`.
**robot_u is populated** (real torques, not zeros). Full object quaternion available here
(metrics.csv only carries planar yaw).

## 4. state_trace.jsonl / planner.log / sim.log — predicted-cost verdict

- `state_trace.jsonl`: 0.1 s cadence, `t, q[5], obj[7], pos_err, ang_err`. Redundant with
  metrics.csv but useful as a cross-check.
- **planner.log contains NO numeric cost lines.** Grep across all 30 runs:
  `SAMPLE_COSTS` 0, `J_` 0, `rank` 0, `best` 0; `cost` 40 hits = only two non-numeric strings
  ("Crossed cost switching threshold." and the `[OBS-LCS] ... obstacle_cost_active=false` banner);
  `curr` 632 hits = all path-string warnings.
- The controller HAS passive instrumentation (`systems/controllers/sampling_based_c3_controller.cc`,
  CostLogger) that writes `controller_cycle_costs.csv`, `candidate_ranking_costs.csv`,
  `selected_candidate_qp_costs.csv`, `selected_candidate_qp_variables.jsonl` — but only when env
  `SAMPLING_C3_COST_LOG_DIR` is set. **It was not set for this campaign**: no reference in any
  `tools/scene_smoke/` launch script and no cost-named file exists in any of the 30 run dirs
  (recursive search, incl. tmp/ — all tmp/ dirs are empty).
- **VERDICT: per-candidate predicted costs (and the chosen candidate's predicted cost) were
  logged NOWHERE. Not reconstructible offline.**

## 5. failure_classification

- `results/failure_classification/c3_segment_audit.csv` — 1211 rows; columns:
  `run_id, episode_idx, start_step, end_step, start_t, end_t, duration_steps, contact_fraction,
  obj_disp_m, yaw_change_rad, epos_reduction, eyaw_reduction, exit_reason, object_frame_sector,
  equivalent_to_prior_failed`. Provides per-run C3-like episodes with start/end steps, contact
  fractions and exits — sufficient for mode shading and transaction analysis.
- `outer_loop_summary.csv` — 30 rows (one per run): reposition/cost-cross/entry counts,
  behavioral episode counts, contact fractions, churn fraction, final errors. Complete.

## 6. Scene geometry

All 6 configs in `tools/scene_smoke/scene_configs/*.yaml` carry `footprint` polygon,
`obstacles.polygons` and/or `obstacles.discs`, goal, pusher_radius, block_half_height,
control_dt, boundary_sample_spacing — **complete for obstacle-metric reconstruction**.
Caveat: schema is 2D only; slalom notes fin height in a comment (treated as full-height walls).

## 7. What CAN vs CANNOT be reconstructed offline

CAN:
- Per-step measured-state cost decomposition (translation, orientation, obstacle) — already in
  metrics.csv, and independently recomputable from pose columns + scene yaml + postprocess_run.py.
- Mode shading and episode segmentation (c3_segment_audit start/end steps join metrics.csv on
  control_step / sim_time).
- Episode-level realized progress (epos/eyaw reduction, obj displacement) and contact behavior
  (physical_contact_active, pusher_object_gap, contact_fraction).
- Effort/energy proxies from steps_raw robot_u/robot_v.

CANNOT:
- **Per-candidate predicted cost J_pred at decision time**, the ranking spread, and the chosen
  candidate's predicted-vs-realized gap. The candidate set, their C3 rollouts, and the ranking
  costs lived only inside the controller and were never emitted.
- Proxy: the **measured-state ranking-style cost evaluated at each episode's start step** can
  stand in for the chosen candidate's cost level. Limitations: (a) it is a single point, not the
  ranked alternative set, so no selection-margin analysis; (b) predicted cost is a rollout sum
  over the horizon under the LCS model, while the proxy is instantaneous at the measured state —
  model-mismatch and horizon effects are invisible; (c) prior forensics
  (prediction-fidelity arc, 2026-09-05) showed 74% of promises are issued while the pusher is
  traveling, so the start-of-episode measured state systematically understates the intended
  contact configuration.

## 8. Quirks that will bite a reconstruction script

1. **Stream cadences differ**: metrics.csv ~70 Hz (rows/sim_time ≈ 0.014 s), steps_raw ~0.026 s,
   state_trace 0.1 s. Join on `sim_time` with nearest-neighbor, never on row index.
2. metrics.csv `sim_time` starts at ~0.1 s, not 0.
3. The ramp factor in goal_pos/goal_theta (Section 2) — divide it out or recompute unramped.
4. NaN columns (pusher_obstacle, robot_contact, admm_penalty) — evaluation_total is a subset sum.
5. Two truncated single_obstacle runs (15.4 s / 21.3 s) — clip episode joins to the run's last step.
6. Object orientation in metrics.csv is yaw-only; use steps_raw quaternion if out-of-plane motion
   matters (icra_sign topple class).
7. grep on these long-line logs is unreliable per standing rule — count with Python.

## 9. Recommendation on rerun

For true predicted costs, **a targeted rerun is required and cheap**: the passive CostLogger is
already in the controller; set `SAMPLING_C3_COST_LOG_DIR=<run_dir>` in the launch env for a small
targeted subset (e.g. one pair per scene, or the single_obstacle pairs). No code change needed; the
hook is documented as no-op on control flow, so behavior should match the frozen baseline.
Not run here per task constraints.
