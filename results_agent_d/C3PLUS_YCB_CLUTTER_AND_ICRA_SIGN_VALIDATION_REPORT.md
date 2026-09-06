# C3+ ycb_clutter and icra_sign Scene Validation (Agent D)

Base `f8c3bd800` (post-P5: 5fbec41ef + 43d5efb96), branch `c3plus-channel-route-v1`;
audit branch `audit/ycb-icra-scene-validation` (worktree `.claude/worktrees/audit-ycb-icra`).
Config: plain arm — `SAMPLING_C3_OBSTACLE_MODE=lcs_contact`, no route mode, P5 timeout/veto
defaults ON, `bt010` task family, serial draws under `flock /tmp/c3plus_sim.lock`, 600 s wall cap.
Evidence: `results/parallel_c3plus_audit/agent_d_ycb_icra/`. No behavioral changes were made.

NOTE on naming: the brief said "xArm6 stack"; the post-P5 C3+ obstacle stack simulates the
**Franka + 19.5 mm sphere pusher** (the xArm6 lives only in the OIM/MJX Python side). The two
scenes here are the dairlib-side ports `push_t_bt010_{ycb_clutter,icra_sign}`.

## 0. Headline

Both scenes **load correctly and run end-to-end** on the current stack; **neither introduces a
new architectural failure class**. But both ports are *reduced* versions of the OIM scenes:

- `ycb_clutter` keeps **2 of OIM's 4 obstacles** (cube + sugar box, as static boxes in sim and
  as circumscribed **discs** in the planner/LCS); spam can and mustard bottle are absent, and
  obstacle positions differ from the OIM MJCF.
- `icra_sign` has **no sign at all** — no obstacle in sim or planner. It is an open-table push
  of the T to the OIM C-slot pose (0.5, −0.4, +90°). The "ICRA 2026" glyph row (7 static mesh
  obstacles, 0.197 m slot passage, 47 mm side clearance for the 0.103 m C) exists only in the
  OIM MJX scene. Every sign-specific question in the brief is therefore **vacuous on this stack**.

Run outcomes (3 serial draws each):

| scene | draw | outcome | best XY | notes |
|---|---|---|---|---|
| ycb_clutter | 0 | FAIL (topple @ t=37.9 s at (0.60, 0.075), beside cube corner) | 0.383 | 320 s dead post-topple |
| ycb_clutter | 1 | FAIL (local fixed point north of cube, 93 repos, 6 P5 timeouts fired+recovered) | 0.400 | acq 12% |
| ycb_clutter | 2 | FAIL (position ~solved: 41 mm; yaw plateau at ~pi) | **0.041** | slid past both obstacles, min obj-obstacle clearance +0.6 mm, 0 penetration |
| icra_sign | 0 | FAIL (topple @ t=31.0 s at (0.48, −0.19), mid-rotation yaw 0.75) | 0.146 | |
| icra_sign | 1 | FAIL (topple @ t=27.6 s at (0.48, −0.18), mid-rotation yaw 1.09) | 0.163 | same state as draw0 |
| icra_sign | 2 | **TIGHT SUCCESS 0.0197 m / 0.094 rad @ t=53.9 s** | 0.020 | survived a 0.43 rad tilt transient |

## 1. Scene loading (§1–3)

All five scenes (open_table, single_obstacle, shelf_gap, ycb_clutter, icra_sign) build the
franka_sim plant with **zero initial penetrations** (`scene_inventory/*_initial_geometry_checks.csv`).
T starts (0.5, 0.3) with 32.6 mm clearance to the sugar box — legitimately "in clutter".
Goals are inside the workspace (radius 0.583 / 0.640 < the 0.75 abort ring). Sim and planner
agree on obstacle set and placement for ycb (2 boxes ↔ 2 discs); the planner discs are
conservative — the sugar-box disc (r=0.0996) overstates its y-half-extent by **52 mm**, which
closes the inter-obstacle west lane in the planner's model (0.068 m between discs vs ~0.10 m
between boxes; both smaller than the T's 0.089 m crossbar, so the east detour is the only route
in either representation — draw2 took it successfully). `TFootprint()` in the controller matches
the m01 T exactly. Object roles: `scene_inventory/ycb_object_roles.csv`.

## 2. Physical sanity (§4)

Standalone rig (`tools/c3plus_diagnostics/agent_d/forced_contact_sanity.py`): a commanded exact
contact moves the T ~1:1 with post-contact pusher travel (45 mm object motion for 45 mm advance)
in both scenes, no obstacle interference at the chosen sector. Environment and plant are healthy;
failures below are controller-level.

## 3. Failure classification (§6)

- ycb draw0: **F10_TOPPLE_OR_STABILITY** (earliest cause; obstacle-adjacent push tipped the T at
  the cube's east corner). No workspace abort, no deadlock.
- ycb draw1: **F4_LOCAL_C3_FIXED_POINT** (earliest), sustained by **F1_CONTACT_ACQUISITION**
  (12% acquisition, 27 mm standoff misses — Agent A's exact anatomy). P5 backstops worked:
  6 reposition timeouts fired and recovered; no absorbing state.
- ycb draw2: **F7_PUSH_PRODUCTION_PLATEAU** in yaw (position solved to 41 mm; 180° yaw residual
  never converted; same class as the open-table orientation ceiling).
- icra draw0/1: **F10_TOPPLE_OR_STABILITY — systematic, not lottery**: both topple at the same
  state (xy≈(0.48,−0.19), t≈28–31 s, yaw 0.75–1.09 into the +90° rotation). The rotated goal
  makes the controller produce yaw through aggressive edge pushes on the tall m01 T.
- icra draw2: SUCCESS.

## 4. Contact acquisition (§9)

Per-transaction segmentation (`contacts/*_contact_acquisition.csv`):
- ycb: acquisition 12% / 33% / 55% per draw; productive_given_contact 0.36–1.0. Acquisition is
  the dominant per-attempt loss where the object sits near obstacles (the 27 mm standoff class).
- icra: acquisition 83–86% — **acquisition is NOT the icra problem**; open-table approach lanes
  make the standoff conversion reliable.

## 5. Candidate pool (§10)

At ycb draw1's plateau the live pool covers **all 8 sectors** (`candidate_pool/*.csv`), and a
dense 8-sector forced-contact bank at the exact stalled state shows every sector contactable,
lateral cube-slides productive (44 mm), but **no straight push yields direct goal progress** —
a genuine local-progress fixed point, escapable only by composed lateral moves.
Classification: **POOL_SUFFICIENT** (both scenes); the deficit is selection/greedy progress,
not sampling coverage.

## 6. Prediction honesty (§11)

`prediction/predicted_vs_actual.csv`, 2 s windows per fresh commitment, conditioned on contact:
- ycb: ρ_xy given contact 0.10–1.6 (draw1: predicted median 65 mm vs ~10% realized) — the known
  dishonest over-promising predictor; ρ≈0 without contact.
- icra: ρ_xy given contact 2.3–2.9 (under-promise; small predictions + real slides).
Prediction is mis-calibrated in both directions but is not the earliest cause in any draw here.

## 7. Object–obstacle LCS (§12)

`contacts/object_obstacle_contact_trace.csv`: both slots always resident (n_obs_slots=2 = the
scene's obstacle count → N_closest switching is trivially correct; zero identity switches, no
staleness possible). φ went negative only within disc conservatism (d_raw ≥ −0.0198 against the
disc while physical box clearance stayed ≥ −0.0 graze; **no meaningful physical penetration in
any draw**). Draw2 slid the T tangentially along the cube with +0.6 mm minimum clearance —
basic-avoidance criterion 2 demonstrated. λ_obs is soft-ADMM-valued (|λ| ~0.01 far, ~0.05–0.12
near; 616 slightly-negative values; comp residual ≤ 0.05) — bounded but not crisp
complementarity, consistent with prior lcs_contact validation.

## 8. Reposition / P5 (§13–14)

No run entered an indefinite blocked reposition: draw1's 6 timeouts all recovered; swept-path
veto logs show no vetoes were needed (paths clear). **P5 still prevents the original deadlock
class in both new scenes.** Basic-avoidance definition: ycb_clutter =
**PARTIAL_BASIC_AVOIDANCE** (criteria 1,2,3,4,5 met; criterion 6 met — draw2 executed feasible
local pushes through the clutter; downgraded from PASS only because 2/3 draws ended in
topple/fixed-point without recovery). icra_sign = **PASS_BASIC_AVOIDANCE — vacuously** (no
obstacles to avoid).

## 9. Required answers (§19)

ycb_clutter: (1) loads correctly, 0 penetrations; (2) planner knows both ported clutter objects
(but only 2 of OIM's 4 exist); (3) LCS valid with both obstacles resident, 0 penetration;
(4) xArm6→Franka acquires contacts at 12–55%/draw (acquisition remains the weakest per-attempt
layer near obstacles); (5) useful contacts are sampled (8/8 sectors); (6) predicted motion is
NOT reliably realized (ρ 0.10–1.6); (7) no success in 3 draws (best 41 mm + π yaw);
(8) earliest causal failures, ranked per draw: topple / local fixed point / yaw production.

icra_sign: (1) the sign does NOT load — it was never ported; (2–4) geometric feasibility,
pusher reach, and arm reach are trivially satisfied (open table); (5) P5 recovery works;
(6) contact acquisition works (83–86%); (7) local C3 produced a full tight success in draw2;
(8) 1/3 success; (9) earliest causal failure in the two failures = systematic topple during
+90° yaw production at a repeatable state.

## 10. Verdicts (§20)

- **YCB: Y7_MULTIPLE_CAUSES_RANKED** — (1) F10 topple beside obstacle, (2) F4 local fixed point
  + F1 acquisition, (3) F7 yaw production; environment and LCS themselves are valid
  (Y1's environment clause holds; capability blocked by the known trio). Secondary:
  **Y5-lite** — the port under-represents OIM (2/4 obstacles, disc conservatism closing a lane).
- **ICRA: I5_GEOMETRIC_OR_OBSTACLE_MODEL_PROBLEM (port-completeness)** for the scene itself —
  the sign geometry is absent, so the benchmark's intent is untested; for the task as ported:
  **I1_ENVIRONMENT_AND_LCS_VALID_TASK_CAPABLE** (tight success draw2), limited by a systematic
  F10 topple during rotation (would be I7 if scored on failures alone).

## Final answer

**These two scenes reveal NO genuinely new architectural limitation.** Every observed failure
is one of the already-measured classes: contact acquisition (F1), local fixed point (F4),
push/yaw production ceiling (F7), and topple (F10) — with P5 reposition recovery confirmed
working in both. What the scenes DO reveal is a **scene-porting gap**: icra_sign's defining
obstacle (the sign) and half of ycb_clutter's obstacles were never mapped into the reference
stack, so the current runs under-test the OIM benchmarks. The one new empirical fact worth
acting on is that the icra topple is **repeatable at a specific state** (same xy±5 mm, twice),
making it a clean target for root-causing the yaw-production topple mechanism — a stability
issue, not an obstacle issue. Per the minimal-fix policy, the indicated fixes are: complete the
scene port (obstacle representation only), and address acquisition/stability in their existing
lanes; no new cost, route heuristic, or planner is justified by this data.
