# Next-stage obstacle-comparison manifest (§14) — post-baseline-freeze

Prepared offline by the audit-ycb-icra session (owner of the faithful icra_sign
port and the ycb/icra scene validation). NO tuning here — this is the matched-
conditions inventory for the four obstacle scenes, to be run against the frozen
xArm6 baseline without modification between scenes.

Common to all scenes (from the OIM cross-check + scene audits):
- object–table μ = 0.3 (explicit MJCF pairs); **object–obstacle and EE–object
  μ = 0.5** (OIM default geom, common.xml:50) — the matched-μ set for obstacle
  scenes needs the 0.5 obstacle pairs added, not just the open_table values.
- OIM obstacle representation = 2D convex hull of the COMPILED MJX collision
  mesh (planner) == what MJX collides (sim). C++/Drake functional equivalent =
  `lcs_contact` N_closest frictionless LCS contact + P5 swept veto, with exact
  geometry via `SAMPLING_C3_OBS_BOXES` (AABBs) or `SAMPLING_C3_OBS_POLYS`
  (exact hulls, landed on branch icra-sign-faithful-port).
- OIM planner also carries the ROBOT BASE as an obstacle (Circle r=0.09 at the
  origin, scenes.py:333) in EVERY tabletop scene — the C++ scenario files for
  single_obstacle/shelf_gap/ycb_clutter do NOT; add the base disc for parity.

| scene | task object | physics (μ obj-table / obj-obs) | start → goal | obstacles (OIM authoritative) | OIM representation | current C++/Drake representation | functional-equivalence requirement |
|---|---|---|---|---|---|---|---|
| single_obstacle | T (0.089×0.099, 0.1 kg) | 0.3 / 0.5 | (0.381, +0.4, 0) → (0.381, −0.4, 0) [OIM]; C++ uses (0.5,0.3)→(0.5,−0.3) registered frame | 0.1 m cube at (0.35, 0, 0.05) | exact box footprint | disc r=0.0707 at (0.45/0.5, 0) — CONSERVATIVE; box available via OBS_BOXES | nonpenetration + tangential slide + reposition recovery around one blocker |
| shelf_gap | T | 0.3 / 0.5 | corridor +y → −y | two shelf slabs forming a gap | exact hulls | 2 boxes (sim) + discs/boxes (planner) | corridor transit with yaw alignment; P5 veto active |
| ycb_clutter | T | 0.3 / 0.5 | corridor +y → −y | cube (0.35,0) + spam_can mesh (0.60,−0.16) + domino_sugar box (0.22,0.20) + mustard_bottle mesh (0.15,−0.18) — ALL FOUR | 8-vert compiled-mesh hulls (spam/mustard), exact boxes (cube/sugar) | **port incomplete: only 2 of 4 obstacles exist** (cube+sugar, shifted positions; agent-D audit 2026-09-05); spam/mustard hulls are ready to add via OBS_POLYS using scenes.py:248-266 vertices | multi-obstacle N_closest identity switching; passable-lane preservation (OIM lane widths 0.099–0.442 m vs 0.089 m crossbar) |
| icra_sign (faithful) | **C glyph** (0.0966×0.103×0.025, 0.1 kg, μ 0.3, own footprint) | 0.3 / 0.5 | (0.3, +0.4, 0) → (0.5, −0.4, +π/2) + 4 jitter variants | 7 glyph hulls "ICRA 2026" + base circle | 10-vert compiled hulls + Circle(0,0,0.09) | **COMPLETE on branch icra-sign-faithful-port** (`anything_icra_c*` tasks, SAMPLING_C3_OBS_POLYS, CFootprint, zero-drift consistency test; Franka baseline: 1 tight success/7 runs) | concave-object footprint + slot insertion + long-row (obs y∈[−0.6,0.66]) + base obstacle; task workspace z floor must be −0.015 (C works at EE z≈−0.0095) |

xArm6-specific carryovers for the obstacle stage:
1. The C task's low working height (EE z ≈ −0.0095) must clear the xArm6
   adapter's z-task and any workspace z checks (the Franka task needed the z
   floor lowered to −0.015; verify the xArm6 executor's Cartesian z range).
2. `SAMPLING_C3_OBS_TOP_Z` per scene: glyphs 0.046, shelf/cube 0.10–0.12,
   ycb mustard 0.19 (tallest) — the P5 z-aware veto needs the per-scene value.
3. ycb_clutter completion (2 missing obstacles + positions) is a SCENE port
   task, not controller work — do it before, and independently of, the
   comparison campaign, exactly as icra_sign was done.
