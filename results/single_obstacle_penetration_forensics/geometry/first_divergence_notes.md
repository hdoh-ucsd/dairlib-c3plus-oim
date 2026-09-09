# First-divergence notes — single_obstacle pair01 penetration forensics

Verdict up front: **no divergence of substance exists anywhere in the run.**
The suspected penetration is a rendering illusion, not a geometry or planner defect.

## Physical vs planner
- On the state-trace grid (1943 poses, t = 0..326.7 s), |phi_planner_footprint − d_phys| < 1 mm at **every** sample; median 0.0002 mm. The planner's 2D footprint SDF (ObsSdfPoint box branch, qx=|x−0.35|−0.05, qy=|y|−0.05, 2 mm edge sampling of the 8-vertex footprint) is an essentially exact model of the Drake 3D signed distance because the T stays flat (z ≈ 0.0008 m, |roll|,|pitch| < 2e-6 throughout the contact phase) and both collision reps are boxes.
- The planner_vs_physical CSV shows an apparent max difference of 13.3 mm at sim_time 6.86 s. This is **purely an interpolation artifact**: d_phys there is linearly interpolated from the 0.2 s state trace onto the ~0.014 s metrics grid during the only fast-transit window (t = 5.4–9.8 s, 189 affected rows, all pre-contact with d_phys > 40 mm). Zero affected rows during the 316 s contact phase.
- phi_planner_center (ranking J_obs input) reads +44.5 mm at closest approach — the T center never gets near the box; center-based ranking cost was therefore nearly inert during the stall, but that is a cost-shaping observation, not a geometric disagreement.

## Planner vs recorded metrics column
- metrics `min_obstacle_clearance` equals the reimplemented phi_planner_footprint to 4.2e-17 m (machine epsilon) on all 24126 rows: the postprocessor used the identical footprint-polygon formula. **No disagreement.**

## Visual vs physical — the only real "divergence"
- Earliest material disagreement: **t ≈ 9.8 s trace (video ≈ 3.0 s)**, and most strongly at the final frame (video 97.1 s, t = 326.7 s). The MP4 shows the T apparently merged with / tucked into the obstacle cube. Causes: (1) T and obstacle are rendered in near-identical orange, (2) oblique 3D camera projects the 59.6 mm-tall T against the 100 mm cube face, (3) actual clearance is 0.65–1.5 mm — visually indistinguishable from overlap at this resolution.
- Ground truth at that configuration: d_phys = +0.65 mm (min over the whole run, at t = 326.7 s), crossbar_collision vs obstacle_collision, witness on the box east face x = 0.40. d_phys was **never ≤ 0**: 0 of 1943 samples in any negative band.

## Timeline
- t = 0–5 s: object static at (0.381, 0.400).
- t = 5–9.8 s: pushed rapidly around/along, reaching the box east face.
- t = 9.8 s: first near-touch (1.47 mm). t = 10 s: motion effectively stops.
- t = 10–326.7 s: pressed against the east face, clearance creeping 1.47 → 0.65 mm (~2.5 µm/s). Run ends failed (pos_err 0.397 m, goal on the far side at (0.381, −0.4)) — the interesting defect here is a 316 s stall against the obstacle, not penetration.

## Missing telemetry
- lambda_obs (obstacle LCS contact force) is not logged anywhere in metrics/steps_raw/planner.log: MISSING_TELEMETRY.
- Metrics `sim_time` starts at 0.1 and lags trace t by up to one control step (~14 ms); irrelevant at these time scales.
- State trace dt is irregular (0.1–0.4 s, median 0.2 s), not the nominal 0.1 s; the MP4 has 972 frames = stride-2 over 1943 trace rows (max-frames 1200 → step 2), so the video covers the FULL 326.7 s at ~3.4x real time — the 120 s cap concern does not apply.
