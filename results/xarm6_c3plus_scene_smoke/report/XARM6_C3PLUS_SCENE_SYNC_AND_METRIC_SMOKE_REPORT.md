# xArm6 C3+ scene synchronization + evaluation-metric smoke validation
2026-09-07 · branch `feature/oim-scene-sync-metrics` (worktree `oim-scene-sync-metrics`, base `dca28c1f4`)
Output root: `results/xarm6_c3plus_scene_smoke/` (in-worktree, mirrored to the shared results tree).

## Answers to the 20 questions

1. **OIM commit defining the tasks:** `d6d80a65ba6a45079e1384ba879e54d50d25bc39`
   (upstream `main`, fetched this session; includes the new slalom task 80a6fde).
   Read-only worktree: `external/oim_upstream_main`. See `provenance/oim_reference.yaml`.
   Note: `eval_metric.jpg` was not found anywhere on this machine; metric
   definitions were source-traced instead, per the brief's own rule.
2. **All 5 starts + 5 goals extracted for all six scenes:** YES —
   `scene_fidelity/*_upstream.yaml` + merged `scene_fidelity/pose_equivalence.csv`
   (60 rows). C++ task dirs exist for pairs 1–5 on open_task/single_obstacle/
   shelf_gap; pair 1 only for ycb_clutter, icra_sign, slalom (pairs 2–5 are
   extracted and ready to instantiate for the 5×5 campaign).
3. **open_task synchronized:** YES. All 10 poses exact (≤1e-6), geometry/mass/
   friction exact under `--matched_mu`; no placeholder obstacle can enter the
   planner (all obstacle branches guarded by `!obstacles.empty()`). Caveats:
   per-link mass split (uniform-density vs upstream 0.05/0.05, CoM Δ1.4 mm) and
   the family-wide planner `mu_per_pair_type` divergence (frozen, not touched).
4. **single_obstacle synchronized:** YES. Cube 0.1³ @ (0.35,0) exact in sim;
   planner now uses the exact upstream box via `SAMPLING_C3_OBS_BOXES` and the
   upstream robot-base disc (0,0,r=0.09) was added to `scenario_params.yaml`.
5. **shelf_gap synchronized:** YES. Both shelves verbatim vs `shelf_gap.xml`;
   corridor 0.200 m; feasibility margin +0.087 m (sim) / +0.070 m (planner
   circles) vs the T's 0.113 m worst-yaw footprint; exact shelf AABBs supplied
   per-index via env; base disc added.
6. **YCB inventory complete:** YES — `scene_fidelity/ycb_object_roles.csv`.
   Upstream currently has block_T (manipulated) + obs_box + domino_sugar +
   spam_can + mustard_bottle (both PRESENT at d6d80a6) + planner-only base
   disc. All physical obstacles exact in the C++ sim; base disc added to the
   planner; exact hulls/boxes via env overrides.
7. **ICRA uses C rather than T:** YES — manipulated object is `push_c_glyph.sdf`
   (3-box block C, mass 0.1 kg) with the `c_glyph` planner footprint, in the new
   `anything_icra_c_matched_xarm6_t1` task (built this session from the
   in-tree faithful `anything_icra_c_v5` scenario grafted onto the xArm6
   matched template; the pre-existing `open_table_c_ira_obstacles_xarm6` task
   was audited NOT faithful — wrong glyph set/poses).
8. **ICRA visibly shows I _ R A 2 0 2 6:** PARTIALLY — the rendered scene
   (`figures/scenes/icra_sign_initial.png` + run video) shows the xArm6, the C
   object, and the seven fixed glyphs in the upstream row with the empty C slot
   at the goal. The glyphs render as their convex-hull prisms (the same
   geometry upstream's planner uses, at exact upstream poses/scales), not as
   textured letterforms — geometrically faithful, typographically simplified.
9. **slalom faithfully reconstructed:** YES — new scene built directly from
   upstream `scenes.py:588-599` + `slalom.xml` at d6d80a6: six fins (exact
   half-extents/centers), three 0.240 m gates at y=+0.22/0/−0.22, 0.07 m fin
   height, base disc, exact fin AABBs via env. Trap documented: slalom.py's
   docstring (±0.21/0.200 m) is stale vs the test-enforced scenes.py values.
   `scene_fidelity/slalom_upstream.yaml`, `figures/scenes/slalom_topdown.png`.
10. **Position error formula:** `pos_err[k] = ||object_xy[k] − goal_xy||₂`
    evaluated on the post-step state (upstream `oim/utils/metrics.py:28-46`,
    the `[1:]` shift), in meters.
11. **Orientation error formula:** `|wrap(object_yaw[k] − goal_yaw)|` with
    wrap to (−π, π] via `(a+π) mod 2π − π`. X-axis: control-step index (not
    wall time).
12. **Upstream Robot block costs:** `goal_pos, goal_theta, obstacle, support,
    approach, align, tilt, tip_z, contact_z, pusher_obstacle, robot_contact,
    effort, admm_penalty` (`oim/utils/costs.py`, TERM_ORDER lines 36-51), with
    time-ramped goal terms, shaping fade within 0.25 m of goal, exp barriers
    capped at exp-arg 10, xarm6.yaml weights. `total` is the per-step sum of
    active terms (no stored series). Full inventory:
    `metrics/oim_robot_block_cost_inventory.csv`.
13. **Live controller costs:** every plotted series is a **passive post-hoc
    recomputation**; each mirrors a live OIM optimizer term except
    `robot_contact`, which is plotted at execution fidelity (measured contact
    force) unlike its planning-fidelity live counterpart.
14. **Evaluation-only:** all of them, as plotted (see 13).
15. **Mapping to C3+:** `metrics/c3plus_robot_block_cost_mapping.csv` —
    EXACT_SHARED_METRIC: pos/theta errors, goal_pos, goal_theta, approach,
    align, tip_z, contact_z; MEANINGFUL_EQUIVALENT: obstacle, support, tilt,
    effort, total (labeled subset sum); NOT_COMPARABLE → NaN (never zero):
    robot_contact, admm_penalty, pusher_obstacle, rate. Any obstacle series
    plotted for C3+ is an EVALUATION DIAGNOSTIC ONLY.
16. **Was the C3+ live objective changed? NO.** No edits to C3/C3+, Q/R, ADMM,
    ranking, buffers, progress, hysteresis, route logic, executor, or contact
    semantics. Scene-side yaml/SDF/env-geometry changes only (base-disc
    obstacle entries, new task dirs, and per-task object/friction config for
    the two newly created scenes, mirroring the already-validated v5 recipe).
17. **open_task metric-pipeline validation:** PASSED. s01→g01 SUCCESS at
    t=49.0 s under the upstream 0.05 m/0.1 rad gate (final 0.0172 m/0.0172 rad),
    3705 control steps, strictly monotonic; position/orientation errors
    reconstruct independently to <1e-9; result JSON equals the CSV final row;
    graph renders (left "Task diagnostics", right "Robot block costs", symlog,
    distinguishable total) with no hidden-NaN handling.
18. **Concurrent scene runs:** 4 of the 5 remaining scenes ran fully
    concurrently (unique udpm ports 7811–7815, isolated dirs/tmp/process
    groups). icra_sign ran serially afterwards because it required a fresh
    worktree binary build plus three per-task config fixes (controller object
    model, base/body names, C sampling params, non-colliding IK start).
19. **All READY scenes produced valid artifact packages:** YES — six complete
    packages (mp4 + metrics CSV + eval graph + result JSON + manifest) under
    `runs/<scene>/`, validation PASS lines in each packaging log.
20. **Scenes ready for the 5×5 campaign:** all six, with two to-dos first:
    instantiate pair-2..5 task dirs for ycb_clutter/icra_sign/slalom (poses
    already extracted), and expect obstacle scenes to need a cap well above
    200 s (no obstacle scene reached the goal within the smoke cap; open_task
    succeeded at 49 s; icra reached 0.365 m/0.021 rad).

## Smoke results (`metrics/smoke_summary.csv`)

| scene | success @200 s | final pos err (m) | final ang err (rad) |
|---|---|---|---|
| open_task | **YES (49.0 s)** | 0.0172 | 0.0172 |
| single_obstacle | no | 0.3083 | 1.5232 |
| shelf_gap | no | 0.5897 | 3.1403 |
| ycb_clutter | no | 0.5564 | 3.0397 |
| icra_sign | no | 0.3649 | 0.0214 |
| slalom | no | 0.6400 | 2.9068 |

Failures at the 200 s smoke cap are consistent with the matched benchmark's
known short-cap C3+ behavior (bb55b1ec7: 1/10 single_obstacle, 0/10 shelf at
100 s) — the smoke gate here is artifact/metric validity, which all six pass.

## Notable incidents (for reproducibility)
- The prebuilt main-checkout binaries were at c08e80611 (no xArm6 flags); a
  rebuild attempt invalidated them; the frozen-source binaries from the
  `audit-xarm6-plant` worktree (diff-verified identical sources) ran the five
  T-scenes; icra needed this worktree's own build (17 min, warm repo cache).
  Binary sha256s: `provenance/binary_sha256.txt`.
- A `pkill -f record_metrics` in relaunch one-liners matched its own wrapper
  and killed the launch chain; three apparent icra "failures" were one stale
  log. Diagnosed via file timestamps; fixed with self-safe patterns.
- icra topple-guard trip at t≈1.2 s was a real physical clip: the T-era IK
  start tip (x=0.246) sits inside the C's 48 mm half-footprint + 19.5 mm
  pusher radius; moved to x=0.217.

## Post-review visual/FK fidelity fix (2026-09-07, second pass)

User review flagged the run videos: wrong desk direction/color and a gapped,
oversized end-effector. The SIM was already faithful (it welds the OIM stick
`end_effector_xarm6_stick.urdf` flush at link6 and uses the white
`ground_oim_xarm6.urdf` 0.80 x 1.523 m y-long table); the defects were in the
smoke tooling only, inherited from an older render script. Fixed in
`tools/scene_smoke/`:
- renderer + postprocessor FK now mount the flush OIM stick (no 0.107 m
  flange offset, no `end_effector_full.urdf`) — tip poses in the metrics CSVs
  are now the true stick tip (0.17385 m from the flange);
- renderer draws the upstream lab table (white rgba 0.95, x∈[−0.05,0.75],
  y∈[−0.7615,0.7615], top z=0) instead of the legacy brown x-long platform,
  with camera-frame lighting so it reads white;
- `pusher_radius` in all scene configs corrected 0.0195 → 0.00555 (the stick
  tip sphere), so `pusher_object_gap`/`physical_contact_active` use the real
  contact geometry.
All six runs were re-postprocessed and re-rendered from the original traces
(no re-simulation needed; trajectories unchanged).

## 30-trial pair campaign (2026-09-07, third pass — user-directed)

All remaining start→goal pairs (s02–s05 for every scene, 24 trials) were run
at the same protocol as the smoke tier (200 s cap, upstream 0.05 m/0.1 rad
success, `--matched_mu`, exact planner geometry env), through **5 parallel
lanes** (unique udpm ports 7841+/7871+, isolated dirs/process groups; machine
load ≈17/16 during waves). Twelve new task dirs were generated for
ycb_clutter/icra_sign/slalom pairs 2–5 (`tools/scene_smoke/gen_pair_dirs.py`,
poses verbatim from upstream `examples/poses` at d6d80a6; the upstream
base-disc was also added to the pre-existing single_obstacle/shelf_gap t2–t5
planner lists for parity with t1).

Full per-trial table: `metrics/pair_campaign_summary.csv` (30 rows = 6 scenes
× 5 pairs). Every trial produced the complete artifact package under
`runs/<scene>/pair0N/`.

**Successes at the 200 s cap: 1/30** — open_task pair 1 (49.0 s). Closest
misses: shelf_gap pair 2 (final 0.0489 m/0.268 rad — just outside both
gates), icra pair 4 (0.248 m), open_task pair 4 (0.224 m). ycb and slalom
show uniform ~0.55–0.79 m finals with near-π orientation error — acquisition
+ first-rotation phases consume the whole cap, consistent with the matched
benchmark's short-cap C3+ behavior (bb55b1ec7). A future statistically
meaningful campaign should raise the cap substantially.

Incidents: the dir generator initially truncated
`sampling_c3_controller_params.yaml` (write-before-read bug — 12 trials ran
empty and were regenerated + rerun); icra pair 4's shared start arm pose
clipped the rotated C at spawn (topple guard at t=1.7 s) and was re-IK'd to
tip (0.291, 0.448) — the pair-4 dir carries its own `q_init_franka`.

## Verdict

SIX_SCENE_METRIC_SMOKE_VALIDATED
