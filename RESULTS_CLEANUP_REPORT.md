# Results cleanup report

**Date:** 2026-09-07. **Branch:** `maintenance/results-cleanup` (from `13bb143dc`). No source code,
models, configs, scene definitions, or frozen baselines were modified; no git history rewritten.

## Size accounting
- Shared results tree before: **38 GB** (in-scope C3+/OIM arc: 17.17 GB; out-of-scope lineages
  fig8/jack/push_t/reference/c3T/campaigns: 23.4 GB — DO_NOT_TOUCH, untouched).
- After: **24 GB** total; in-scope residue ≈ **1.8 GB** (icra_sign_faithful_port 1.33 GB protected
  as ACTIVE ICRA work; c3plus_p5_fix_validation canonical draws 0.25 GB; validation dirs; final
  benchmark 55 MB).
- **Removed ≈ 14 GB** (~82% of the in-scope tree). Deletion log:
  worktree `results/cleanup_staging/deletion_log-equivalent` = the explicit rm batches recorded in
  the session; manifest decisions in `RESULTS_CLEANUP_MANIFEST.csv`.

## Major directories removed (bulk raw; final reports/manifests retained in place)
xarm6_oim_arc (gates-arc, user-invalidated — `INVALIDATED_xarm6_oim_arc.txt` left);
obstacle_pushing_comparison shelf_gap+single_obstacle raw draws (pre-fidelity frames);
c3plus_lcs_contact_v2 raw draws; c3plus_variant_cost_probe banks/probes/short_online_validation;
c3plus_inner_qp_obstacle_t010; franka_oim_arc; full_sampling_geometric_release_progress_*;
oim_sampling_c3plus; c3plus_obstacle_lcs_fix; c3ab_* raw trials + video collection;
c3plus_{cost_decomposition,collision_awareness,baseline}_t010 (+t01);
panda_to_xarm6_port_runs..r5 and port_final_r1..r5 (evidence extracted first);
panda_to_xarm6_final_probe1..3 (superseded by probes 4–7);
p5_fix_validation draw2 + single_obstacle_fix;
parallel_c3plus_audit/agent_d_ycb_icra/runs (pre-fidelity ycb/icra validation raws).

## Canonical retained
- `results/final_oim_c3plus_comparison{,_runs}` — PRIMARY dataset; **validated post-cleanup:
  180/180 metric recomputations match** (`cleanup_staging/final_benchmark_validation.csv`).
- Frozen baseline + gate campaign (`panda_to_xarm6_port_final_r6`) + root-cause probes 4–7.
- `results/canonical_failure_evidence/` — 10 classes, 3.3 MB, one representative each with README.
- All final reports/handoffs of every root-cause chapter (worktree root + per-dir).
- Active ICRA (`icra_sign_faithful_port` untouched) and YCB fidelity artifacts.
- MOVE executed: `c3plus_baseline_t010/.../config_snapshot` →
  `/root/push_anything_ADMM/config/snapshots/c3plus_baseline_t010_single_obstacle_seed0`
  (SDF referenced by micro_push_test.py; references repointed).

## Invalid benchmarks documented
Old C++ icra (T-object) raws: removed with Agent D's report retained; pre-fidelity obstacle scenes
(shifted frames, non-rigid poses, unmatched μ): raws removed, invalidation recorded in the final
benchmark report + `obstacle_pushing_comparison_2026-09-06/` report retained.

## Reference safety
4 conflicts found (`cleanup_staging/referenced_artifacts.csv`); resolved by §13B: three historical
reports annotated with cleanup notes pointing at canonical equivalents; the trap handoff's probe
glob pinned to probes 4–7. No canonical report has a broken load-bearing path.

## Duplicates
`cleanup_staging/duplicate_files.csv`: 52 groups, ~0.10 GB — dominated by intentional
worktree↔shared mirroring of small CSVs (retained; the D tree is a mirror by definition).

## Sanity checklist
[x] final matched benchmark complete + metrics reproduce (180/180)
[x] frozen baseline + protocol present
[x] figures/videos present  [x] port fidelity evidence present (probes 4–7, r6, reports)
[x] Agent A/B/C/D conclusions defendable (reports + canonical evidence)
[x] YCB + faithful ICRA work intact  [x] no broken canonical references
[x] no controller/source changes  [ ] D mirror — synced in the follow-up step below

CLEANUP_COMPLETE_CANONICAL_RESULTS_PRESERVED


## Second pass (2026-09-07, user-directed: prune videos and logs)
- ICRA draws v1-v5/v1_fresh/v4_zfix removed (kept v5_zfix tight-success + summary CSVs + note):
  icra_sign_faithful_port 1.33 GB -> 122 MB.
- Diagnostic logs (planner/osc/sim.log) stripped from final benchmark runs, r6 gate,
  env-validation trials, p5 draws (rollout.mp4 + qp_variables.jsonl also dropped);
  metric-bearing traces (state_trace.jsonl, success.log, CSVs) retained everywhere.
- Videos pruned to representatives: 3x 3D success + gate success/fail pair + matched
  success/fail pair + P6_r2_t3 TOPPLE + P5_r2_t4 NEARMISS (+ manifest); ~60 redundant MP4s removed.
  The curated D XARM6_C3PLUS_WORKING_VIDEOS folder retained as the user-facing gallery.
- Also removed: 6-DOF port rounds r6/r7, the early oim_xarm_full_20260829 receipt, oim-arc
  sidepanel MP4s, native-validation per-trial logs (summaries restored from committed copies),
  and the geometric-release debug arc from D.
- In-scope retained set now ~360 files / ~0.6 GB. D mirror updated with --delete on retained dirs.
- REMAINING BULK IS OUT OF SCOPE: ~23 GB / ~4400 files are the fig8/jack/push_t/reference
  lineages, untouched per the original scope ruling - a third pass can cover them on direction.


## Third pass (2026-09-07, user-directed: fig8/jack/push_t/reference lineages)
- Campaign run subdirectories removed from 21 lineage dirs (fig8 campaigns/archives, reference,
  campaigns, c3T trials) keeping every top-level summary CSV/JSON/MD + RUNS_REMOVED_NOTE per dir;
  the curated fig8_success_gallery retained; repo-root FIG8_*.csv untouched.
- Oversized raw stdout logs deleted (jacktoy 600s captures, push_t_krandom_28_s0.txt [v2 kept],
  ttg vm-killed log, gallon failure txt); memory-referenced receipts and all small evidentiary
  videos (tgate_sixleg, push_t_single_video, jack_win/costlcs/rerun sidepanels) kept.
- Freed locally: ~22.4 GB. Local results tree final: 948 MB / 1,131 files (from 38 GB / ~5,200).
- D mirror: equivalent pass freed ~22.9 GB + D-only raw jack ttg/win stdout logs and the
  L1_progress_cluster; D results final 3.4 GB (extra vs local = source-asset fig8_objects meshes
  [protected as assets], oim_mjx_run_jsons, and small D-only historical receipts).
