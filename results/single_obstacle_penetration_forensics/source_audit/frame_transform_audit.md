# Frame / Transform Audit — single_obstacle pair01 (xArm6 C3+, lcs_contact)

Scope: read-only source audit; run = results/xarm6_c3plus_scene_smoke/runs/single_obstacle/pair01
(demo matched_single_obstacle_xarm6_t1, sim `--matched_mu`, planner env
`SAMPLING_C3_OBSTACLE_MODE=lcs_contact` + `SAMPLING_C3_OBS_BOXES="0.35,0.0,0.05,0.05"`,
set by tools/scene_smoke/run_scene_smoke.sh:18-19 + tools/scene_smoke/env_single_obstacle.sh:3).
All paths relative to worktree root.

## 1. World frame / ground datum — CONSISTENT
- xArm6 base welded at world identity (MJCF jointless base auto-weld,
  examples/sampling_c3/sampling_c3_utils.cc:70-74).
- Ground welded to base with offset (0,0,-0.029) (sampling_c3_utils.h:44,
  sampling_c3_utils.cc:121-131). Ground URDF collision box center at local
  z=-0.05, thickness 0.1 (urdf/ground_oim_xarm6.urdf:24-26) → **table top z = -0.029 + (-0.05) + 0.05 = -0.029**.
  Matches the OIM convention stated in the obstacle SDF comment ("top z = -0.029").
- Legacy platform (0.3×0.6×0.029 box at x offset -0.07, sampling_c3_utils.h:45,
  urdf/platform.urdf:13-15) sits under the robot base region only; irrelevant at x≈0.35.

## 2. Sim obstacle pose — CONSISTENT
- SDF: urdf/single_obstacle_box_oimframe.sdf:12 `<pose>0.35 0.0 0.021</pose>`, box
  full extents 0.1×0.1×0.1 (:18,:23). Bottom = 0.021 − 0.05 = **-0.029** = table top (rests exactly).
  Top = 0.071.
- Loaded as a static (self-welding) model in franka_sim.cc:130-137 from
  scenario_params.obstacle_model (matched_single_obstacle_xarm6_t1/parameters/scenario_params.yaml:7).
  No extra offset applied at parse — pose is world-frame verbatim. No legacy 0.50 x-offset (that
  lives only in the retired single_obstacle_box.sdf, per the comment in the SDF header).

## 3. Sim object spawn — CONSISTENT
- sim_params.yaml:17 `q_init_object: [1,0,0,0, 0.381, 0.4, 0.0008]` (wxyz quat + xyz).
- T SDF (urdf/push_t_oimscale_m01.sdf): vertical_link root at model origin; crossbar
  collision box 0.089×0.0198×0.0596 at local (0, 0.0099, 0) (:16-17); horizontal_link
  (stem) at link pose (0,-0.0397,0) (:21) with box 0.0198×0.0794×0.0596 (:31).
  Half-height 0.0298 → bottom = 0.0008 − 0.0298 = **-0.029** = table top. Spawn at
  (0.381, 0.4), ~0.40 m from the obstacle center — clear at t=0.

## 4. Planner obstacle representation — CONSISTENT (numbers match exactly)
- Discs: scenario_params.yaml:2-4 → obstacle 0 disc [0.35, 0.0, 0.0707]
  (0.0707 = 0.05·√2 = circumscribed radius of the box — used only when no box/poly override),
  obstacle 1 = robot base disc [0,0,0.09] (planner-only; deliberately has NO sim geometry).
- Box override: `SAMPLING_C3_OBS_BOXES` parsed at
  systems/controllers/sampling_based_c3_controller.cc:920-929 as `cx,cy,hx,hy` →
  obs_boxes[0] = (0.35, 0.0, 0.05, 0.05). Semantics of b[2],b[3] are HALF-extents:
  the box SDF branch computes `qx = |wx-b[0]| - b[2]` (ObsSdfPoint, :429; identical formula
  in the LCS witness, :713-714). Env supplies 0.05 = 0.1/2 of the SDF full extent →
  **half-extent convention matches, no half/full mix-up, no x/y swap** (b[0]=x-center 0.35,
  b[1]=y-center 0.0 match SDF pose x,y).
- Box applies to obstacle index 0 only (`oi < obs_boxes.size()`, :427/:697); the base disc
  (index 1) stays a true disc — matches the env file's stated intent (env_single_obstacle.sh:1-2).

## 5. Planner object footprint + yaw — CONSISTENT
- Footprint selector (:334-343): env `SAMPLING_C3_OBJECT_FOOTPRINT` unset in this campaign
  → TFootprint() (:244-261): rect(0, 0.0099, 0.089, 0.0198) + rect(0, -0.0397, 0.0198, 0.0794),
  i.e. boundary samples of exactly the two SDF collision boxes **in the vertical_link (model
  root) frame** — same frame as the object pose the sim publishes (vertical_link is the
  floating base; ObjectStateSender publishes model state, franka_sim.cc:207-224).
  Crossbar center offset +0.0099 and stem center -0.0397 both reproduced. CONSISTENT.
- World transform of footprint points: `wx = ox + cos(yaw)*bx - sin(yaw)*by`,
  `wy = oy + sin(yaw)*bx + cos(yaw)*by` (witness :700-702, route :443-444) — standard CCW
  R(yaw)·p_local with yaw = YawWXYZ(q) from x_lcs quat slots 3-6 (:690). **Yaw sign correct**
  (world = R(yaw)·local, matching Drake's planar rotation of the floating body).
- Object position slots x_lcs(7),(8) = object x,y (:692) — consistent with the 3-pos layout
  used everywhere else in the controller (e.g. ranking cost :2529).

## 6. Witness / SDF formula parity — CONSISTENT
- ObsSdfPoint (:419-434) and the lcs_contact witness (ComputeObstacleLcsContacts :685-763)
  use byte-identical AABB math (comment :416-418 asserts this; verified: same q-form).
  Outside: d = hypot(max(qx,0), max(qy,0)) with outward normal; inside: nearest-face
  signed distance (negative). phi fed to the LCS = d_raw − margin, margin default
  0.01 m (nonpen_margin, :201; `SAMPLING_C3_OBJ_MARGIN` not set by the campaign scripts).

## 7. Known intentional 2D simplification (not a mismatch, but note)
- The planner obstacle model is purely planar (x,y footprint). Obstacle height (top z=0.071)
  is ignored by the LCS witness; z-awareness exists only in the reposition PWL veto
  (obs_top_z default 0.12, :233-237). For the T on the table (top of T = 0.0008+0.0298
  = 0.0306 < 0.071) the planar model is valid — the T can never pass over or under the box.

## Verdict table
| Check | Verdict | Numbers |
|---|---|---|
| Table-top datum sim vs planner | CONSISTENT | z = -0.029 both |
| Obstacle pose sim vs OBS_BOXES | CONSISTENT | (0.35, 0.0), 0.05 half-extents both |
| Half vs full extents | CONSISTENT | env 0.05 = SDF 0.1/2; code uses half-extent q-form |
| x/y swap | NONE | b[0]→x, b[1]→y verified in both branches |
| Yaw sign | CONSISTENT | CCW R(yaw)·local in witness and route |
| T footprint origin convention | CONSISTENT | vertical_link frame, offsets 0.0099 / -0.0397 match SDF |
| Legacy obstacle offset | ABSENT | oimframe SDF at 0.35; legacy 0.50 file not referenced |
| Disc radius vs box | CONSISTENT | 0.0707 disc dormant (box overrides index 0) |
| Base disc (index 1) | PLANNER-ONLY by design | no sim geometry; scenario_params.yaml:4 comment |

**No frame/transform MISMATCH found.** Any real object–obstacle penetration is not explained
by a scene-registration error; see obstacle_contact_pipeline_trace.md for the mechanism-level
gaps (frozen witness, single-point contact, margin linearization).
