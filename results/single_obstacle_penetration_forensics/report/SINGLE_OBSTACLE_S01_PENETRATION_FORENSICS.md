# single_obstacle s01→g01 — object–obstacle penetration forensics
2026-09-08 · run `xarm6_c3plus_single_obstacle_s01_to_g01` (600 s tier) · FREEZE
honored: zero source/config/scene changes; offline pydrake reconstruction only;
no reruns needed. Compute: 16 cores, two parallel forensic tracks, ~8 min wall.

## Answers to the 18 questions

1. **Is the apparent penetration physically real? NO.** Offline Drake
   `SignedDistancePairs` over every recorded pose (all object collision geoms ×
   all obstacle geoms, 1943 trace poses): the signed distance never goes
   negative.
2. **Minimum physical signed distance: +0.65 mm** (crossbar_collision vs
   obstacle east face x=0.40), at t=326.7 s (final sample).
3. **Duration of physical penetration: 0 s.** 0/1943 samples < 0; all four
   magnitude bands (0…−1, −1…−3, −3…−5, <−5 mm) have zero entries.
4. **Compliant-depth question: moot** — there is no overlap at all. (For
   reference the sim runs Drake-default point contact at dt 1e-4; normal
   compliant depth would be µm–mm scale; ≥5 mm would be abnormal.)
5. **Does the planner/SDF report penetration? NO.** `phi_planner_footprint`
   (exact ObsSdfPoint box SDF + footprint polygon) matches d_phys to <1 mm at
   every sample (median 0.2 µm — the T stays flat to 2e-6 rad, so the 2D SDF
   is the true geometry). The metrics CSV's own `min_obstacle_clearance`
   matches the reimplementation to 4e-17 m on all 24126 rows.
6. **Do physical and planner boundaries coincide? YES** — exact agreement.
7. **Obstacle pose/dimension/frame correct? YES.** SDF (0.35, 0, 0.021),
   0.1³ cube resting on the table (top −0.029); env box "0.35,0,0.05,0.05"
   is half-extents exactly as the code consumes; no half/full mix-up, no x/y
   swap, no legacy +0.50 offset. (`frame_transform_audit.md`)
8. **T physical geometry correct? YES** — SDF crossbar/stem verbatim vs the
   OIM reference; spawn z puts the bottom exactly on the table.
9. **T planner footprint correct? YES** — `TFootprint()` reproduces both
   collision boxes with correct offsets and CCW yaw sign.
10. **Collision filtering correct? YES** — no exclusions anywhere; obstacle
    registered with full proximity role; `--matched_mu` pair lines visible in
    sim.log confirm the pair is live.
11. **Does Drake generate contact force?** Force telemetry was not logged:
    MISSING_TELEMETRY_REQUIRES_FUTURE_INSTRUMENTATION. (Given d_phys ≥ +0.65
    mm, sustained contact force is not expected; the object is pressed to
    within compliant range of the face by the pusher.)
12. **Does lambda_obs activate when expected?** MISSING_TELEMETRY — obstacle
    λ/η/φ were not logged in this campaign. The frozen binary CAN log them
    with `SAMPLING_C3_COST_LOG_DIR` set (obstacle_lcs_contacts.csv +
    per-knot gaptrace, controller.cc:64-117/2735-2800) — a future
    frozen-binary replay needs only that env var, no source change.
13. **Correct obstacle active in the planner? YES** structurally: lcs_contact
    slots for the box are built from the exact env-box witness; the base disc
    is a separate scenario obstacle.
14. **Obstacle contact through all planner stages** (pipeline trace):
    geometry query PRESENT; C3 subproblem slots PRESENT; ADMM λ PRESENT;
    projection PRESENT; ranking rollout PRESENT (same augmented LCS); ranking
    exp obstacle cost ABSENT by design in lcs_contact mode (guard :2522);
    executed OSC trajectory ABSENT (EE knots only — obstacles never reach the
    executor, by design).
15. **Earliest causal divergence: t≈9.8 s, visual-vs-physical only.** The T
    reaches 1.47 mm from the face at t=9.8 s and the render shows apparent
    merging from then on; no physical-vs-planner or planner-vs-metrics
    divergence exists anywhere in the run.
16. **Best-supported hypothesis: P1_VISUAL_FALSE_POSITIVE** (HIGH
    confidence). Both bodies render in near-identical orange under an oblique
    camera; 0.65–1.5 mm true clearance is sub-pixel. All of P2–P9 are
    numerically excluded for this run (exact geometry agreement, correct
    frames, live collision pair); P6/P7 unevaluable for force/λ but moot with
    d_phys > 0.
17. **What would need changing later (smallest future fixes):**
    (a) rendering only — give the obstacle a distinct color in
    `render_run_3d.py`/scene SDF material so tangency isn't misread (cosmetic,
    outside the frozen controller);
    (b) unrelated-to-penetration but surfaced here: the ranking obstacle
    cost's object-CENTER clearance read +44.5 mm during the entire 316 s
    press-stall while the true footprint gap was 0.65 mm — a footprint-based
    clearance for J_obs_rank is the candidate improvement (Task-B territory,
    NOT changed here);
    (c) enable `SAMPLING_C3_COST_LOG_DIR` in future campaigns for λ/φ
    telemetry.
18. **Regression test after any future fix:** replay this exact run config
    frozen; assert (i) offline d_phys ≥ 0 at every trace pose, (ii)
    |phi_planner_footprint − d_phys| < 2 mm throughout, (iii) with cost
    logging on, obstacle φ in the gaptrace agrees in sign with d_phys, and
    (iv) a render frame at the closest approach visibly separates the bodies.

## Context (recorded, out of scope)
The run's actual failure is the documented F2 press-stall: pushed to the box
in 10 s, then parked 316 s at 0.65–1.5 mm clearance (creep 2.5 µm/s), goal on
the far side. Outer-loop behavior recorded only, per the freeze. Also noted:
the LCS obstacle witness is frozen per planning cycle (shared across
candidates/horizon) — a real mechanism by which fast object motion could
outrun the constraint in other runs, but NOT implicated here (no penetration
occurred).

## VERDICT

ROOT_CAUSE_IDENTIFIED_NO_CODE_CHANGED
(root cause of the *appearance*: rendering false positive at sub-pixel
clearance; obstacle nonpenetration behaved correctly in this run.)
