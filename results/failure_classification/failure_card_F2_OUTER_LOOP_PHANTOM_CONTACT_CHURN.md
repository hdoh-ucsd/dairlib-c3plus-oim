# Failure card — F2 Outer-loop phantom-contact churn
**Representative run:** single_obstacle/pair01 (s01→g01), TRUE_STALL, confidence HIGH

## Key numbers
- Duration 326.7 s, 24 126 control steps; final e_pos 0.397 m / e_yaw 2.97 rad (never inside tolerance)
- phantom_churn_runtime_fraction **0.937** (campaign max), contactless_c3_fraction **0.988**
- physical_contact_fraction 1.3% (3 episodes total); acquisition_failure_rate 0.923
- 142 logged C3 entries / 141 repositions vs only 13 behavioral episodes (reconciliation 0.092): the outer loop flips mode flags while the tip is physically parked at a ~7.5 cm standoff

## Where to look
- Eval graph: `results/xarm6_c3plus_scene_smoke/runs/single_obstacle/pair01/xarm6_c3plus_single_obstacle_s01_to_g01_eval_metrics.png`
- Video `.../single_obstacle/pair01/xarm6_c3plus_single_obstacle_s01_to_g01.mp4`: **0:40** (loop already established — tip hovering at fixed standoff), **2:45** (mid-run, identical pose, log churning), **5:20** (end — object untouched)

## Narrative
After an early failed approach (~first 30 s) the tip settles at a constant gap. C3 mode runs with an optimistic contact belief against a contact that never exists, makes no progress for the progress window, exits via "Repositioning after not making progress in C3", and the reposition target is (near-)identical, so the tip barely moves before "Switching to C3 because reached repositioning target" fires again — 142 cycles with zero net object motion.

## Causal explanation
Pure outer-loop defect: the run says nothing about local C3's ability to push once in contact — the planner spends 94% of the run optimizing a phantom contact. Fix locus: reposition-target diversity + contact-belief validation, not the C3 solver.
