# Failure card — F7 Reposition workspace failure
**Representative run:** single_obstacle/pair04 (s04→g04), RUNTIME_TERMINATION, confidence HIGH

## Key numbers
- Planner abort at **15.3 s** (pair05 twin at 21.2 s): `CheckForWorkspaceLimitViolations()` assert at `sampling_based_c3_controller.cc:3397`, during a reposition
- 1 030 control steps before abort; sim + recorder coasted to ~365 s with the object frozen (final e_pos 0.603 m)
- Pre-crash contact fraction 0.40 is truncated-era data only — no post-crash control existed

## Where to look
- Eval graph: `results/xarm6_c3plus_scene_smoke/runs/single_obstacle/pair04/xarm6_c3plus_single_obstacle_s04_to_g04_eval_metrics.png`
- Video `.../single_obstacle/pair04/xarm6_c3plus_single_obstacle_s04_to_g04.mp4`: **0:10** (initial approach), **0:15** (reposition that trips the workspace assert; arm freezes), **1:30** (coasting — nothing will happen again)

## Narrative
The reposition-target generator commands a tip state outside the declared workspace box; the hard assert kills the planner instead of clamping or rejecting the sample. Everything after is a dead sim.

## Causal explanation
Workspace failure inside reposition sampling (F7), a runtime robustness defect (F10 secondary). Says nothing about pushing capability; both crashes are in single_obstacle and reproduce early (15-21 s).
