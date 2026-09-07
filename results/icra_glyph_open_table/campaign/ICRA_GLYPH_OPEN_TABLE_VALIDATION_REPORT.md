# ICRA glyph open-table validation — final report (2026-09-07)

**Question:** Can the faithful frozen C3+/dairlib xArm6 controller manipulate I, C, R, A
independently on open_table before any obstacle/sign geometry is introduced?

**Answer: YES for A, R, I (with per-glyph reliability caveats); NO for C under this scene —
not because C is unmanipulable (it latched success once and reached ≤3.5 cm with yaw solved in
5 more trials — once 1.8 cm) but because the protocol goal sits 5.5 cm from the ground-slab edge and C is the
one glyph the pusher can geometrically hook and drag over that edge.**

## Protocol (as pre-declared in provenance/run_manifest.yaml)
Frozen xArm6 baseline (mainline lineage, five-joint velocity executor, prelift release,
matched_mu) — zero controller changes. Objects = the push-anything letter files
(I/C/R_shape_texture, A_shape_video; user-directed switch from derived prisms), anything-lineage
wiring: kMeshNormal sampler (strategy 7), VHACD sim SDFs 0.05 kg, universal 1.0 kg controller
model (intentional lineage convention). Task: (0.30, +0.40, yaw 0) → (0.50, −0.40, +π/2).
Tolerance 0.05 m ∧ 0.1 rad. 3 smoke + 10 full trials per glyph, 2400 s wall cap, no tuning
between glyphs. One code accommodation (not policy): demo-name whitelist extended to
`open_table_glyph*`; EE start rotated off the spawn point (base −0.7 rad) after the initial
descent plowed the object.

## Pre-campaign gates (all passed before any campaign trial)
model inventory + sim-vs-controller audit; static scene renders; footprint/sampling coverage;
exact-contact microtests (all letters translate 0.081–0.094 m and rotate 0.66–0.80 rad, zero
topple, max roll/pitch 0.164 rad); prediction sanity. Flag carried in: anything-R yaws even on
centroid-aimed pushes (mass asymmetry of counter+serif).

## Results (latched = first time both tolerances met; final = state at run end)

| Glyph | Latched | Final-pass | t_success range | Failure signature |
|---|---|---|---|---|
| A | 10/13 (77%) | 9/13 | 76–100 s | 3 stalls/near-miss; 1 post-latch drift |
| R | 7/13 (54%) | 7/13 | 172–487 s | stalls at 0.3–0.5 m with yaw already solved |
| I | 7/13 (54%) | 5/13 | 285–1107 s | slowest; chronic 5–7 cm park-outside near-miss; post-latch drift |
| C | 1/13 (8%) | 0/13 | 160 s (once) | 11 table-edge falls; 2 on-table stalls |

(Counts = smoke + full combined, 13 trials/glyph, all complete; per-trial rows in
smoke/smoke_scores.csv and full/full_scores.csv.)

## Failure taxonomy (observed classes)
- **G-EDGE (C only, 11×):** table-edge fall. ground.urdf is 0.91 m in y → surface ends at
  y = ±0.455; goal y = −0.40 leaves 5.5 cm margin. Object z sags over the lip 1–2 s before
  attitude changes, so the attitude-based topple guard fires inherently late. Two planner
  death modes downstream: SAMPLING_C3_TOPPLE_GUARD throw, EE workspace assert.
- **G-HOOK (C only, driver of ≥2 edge falls):** EE peg (3 cm) captured in C's ~60° open mouth;
  arm posture freezes and the base joint carts the letter on a ~0.68 m arc (once ~200°, 1.3 m,
  over the opposite edge). Unmodeled contact mode — the planner believes it is repositioning.
- **G-STALL (A 2×, R 4×, I 1×, C 2×):** no-progress repositioning churn + unsuccessful-sample
  buffer overflow, object parked 0.3–0.5 m out, frequently with yaw fully solved.
- **G-NEARMISS (I 4×, A 1×):** parks 5–7 cm from goal (position) or just over the yaw bar;
  I's dominant mode — min errors show both axes individually solved but never simultaneously.
- **G-DRIFT (A 1×, I 2×):** latched success then post-latch disturbance pushes yaw back out
  (0.13–0.26 rad) with no re-convergence by cap.
- Zero intrinsic tip-overs, zero sampler exhaustion, zero OSC/sim crashes. Both planner aborts
  were C post-edge-fall consequences.

## Geometry-specific analysis
- **A**: near-symmetric wide footprint, closed outline — no capture, fastest convergence.
- **R**: pressure-center offset (open counter + serif) rotates even centroid pushes; closed-loop
  compensates (yaw min ≤0.001 in most trials) but position stalls dominate failures.
- **I**: narrow slat — small yaw lever arm makes the last cm of simultaneous pos+yaw hard;
  chronic near-miss parking.
- **C**: statically as stable as the others (matches microtests); uniquely vulnerable
  closed-loop: mouth capture (only glyph where the 3 cm pusher fits), convex curved rim
  (pushes convert to spin+coast), 0.05 kg at μ=0.3 ≈ 0.15 N ground friction (coasts through
  the goal toward the edge). Full forensics: analysis/C_TOPPLE_MECHANISM.md.

## Verdict
- **A: PASS** (validated for sign-stage use).
- **R: PASS-with-caveat** (capable + repeatable; ~40% stall tail is the generic frozen-baseline
  churn also seen on A, not R-specific).
- **I: MARGINAL** (capable — 50% latched — but chronic near-miss parking; expect long times).
- **C: FAIL under this scene margin** — the blocker the user pre-registered ("do not proceed to
  slot insertion until C at minimum passes") STANDS. C is manipulable in the interior; the
  failure surface is goal-to-edge margin + unmodeled mouth-hook drag. No changes made (frozen
  baseline). Candidate remedies for authorization: object-position workspace bound (only the EE
  is checked today), goal placement margin, or hook-aware contact handling.
- **Full ICRA sign benchmark: NOT started**, per instruction.

Receipts: results/icra_glyph_open_table_runs/{smoke,full,analysis}/ + results/icra_glyph_open_table/
(gates). Scorer: score_glyph_runs.py (tolerance 0.05/0.1, topple flag at 0.3 rad).
