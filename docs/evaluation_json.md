# Evaluation JSON reference

Inspected `C:\Users\augus\Downloads\xarm6_icra_sign_mppi_20260901_172438.json`
through Windows PowerShell on 2026-09-12. The reference is a MuJoCo/Warp MPPI
run (seed 0, start 5, goal 2), not a C3+ result. Our runs remain seed 42.

The file has five top-level objects: `schema`, `run`, `hyperparameters`,
`static`, and `dynamic`. It does not contain a separate aggregate metrics block.

| Block | Required meaning |
| --- | --- |
| `schema` | State/input indexing, coordinate frames, optional prediction alignment, velocity definitions |
| `run` | World, task, robot, algorithm, optimizer names, seed, start/goal indices, backend, interactive flag |
| `hyperparameters` | Steps, samples, horizon, optimizer settings, control interval, success tolerances, cost weights; inapplicable settings are null |
| `static` | Goal, body-frame footprint, limit surface/wrench limits, obstacles, robot definition, simulation timestep and state layout |
| `dynamic` | State trajectories, applied controls and per-step computation/contact diagnostics |

For N applied inputs, `time`, `object_pose`, `object_velocity`, `robot_pos`,
`robot_vel`, `qpos`, and `qvel` have N+1 entries, including the initial state.
`robot_control`, `compute_time`, `tip_z`, `tip_tilt`,
`contact_normal_force_z`, and `robot_contact_force` have N entries.
The inspected file has N=1000. Optional wrench and predicted-plan arrays are
described by `schema` but absent from this particular run.

Object pose is world-frame `[x,y,yaw]`; footprint vertices are body-frame.
Robot position is the contact/tip point, not joint position. The example has
five-dimensional controls and nine-dimensional simulator qpos/qvel; those
layouts are backend-specific and must not be relabeled as Drake state arrays.
Optional predictions must describe the same object's trajectory in both blocks,
not robot joint plans.

## Current compatibility gap

`record_metrics.py` currently records asynchronous latest-state snapshots on
planner debug ticks, plus a separate approximately 10 Hz pose trace. Its
`robot_u` comes from reported robot-state effort, not a synchronized applied-input
stream. It does not record optimizer compute time, object velocities, or contact
forces. The existing `*_result.json` is an aggregate summary and is **not** this
schema. Exact N+1/N transition alignment cannot be recovered reliably from old
logs by simply renaming fields or adding an artificial terminal sample.

After reproduction/mesh review, implement a synchronized recorder and exporter
with explicit Drake state layout, real timing/contact channels, and truthful
nulls for unavailable or inapplicable algorithm settings. Keep that change
separate from the current dependency/documentation cleanup; do not fabricate
historical measurements or claim full cross-algorithm cost equivalence.
