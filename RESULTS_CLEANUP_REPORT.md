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
