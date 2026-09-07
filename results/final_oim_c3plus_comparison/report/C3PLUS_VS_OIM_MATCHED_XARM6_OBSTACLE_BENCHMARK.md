# C3+/dairlib vs OIM — matched xArm6 obstacle benchmark

**Date:** 2026-09-06/07. **C3+:** frozen baseline `9557bbdc4` (+ scene-definition-only additions,
verified inert on the frozen behavior by a 3/3 open_table health check). **OIM:** upstream
`a954f000`, native MJX, first-ever local execution (RTX 5070 Ti; setup receipt in provenance).
**Protocol (declared pre-campaign):** shared cap 100 sim s (= upstream 2000×0.05 s), upstream
success tolerance 0.05 m ∧ 0.1 rad, RIGID identity pose registration from the upstream pose yamls,
paired pose keys 1–5, matched physics (T-table 0.3, EE-T 0.5, object-obstacle 0.5).
Parallel trial execution was user-authorized mid-campaign (C3+ 3-way, OIM 2-way GPU); sim-time
scoring is load-invariant.

## Answers (§25)

1–2. **Same xArm6?** Same embodiment semantics (5 velocity-servo joints, joint 6 welded, identical
   limits/kv/armature/damping/spring), realized in each method's native simulator (C3+/Drake matched
   to OIM/MJX in the port phase; `robot_equivalence.csv`). One declared difference: the C3+ tool is
   the Franka sphere-tip (r=0.0195) vs OIM's stick (r=0.00555) — the frozen baseline's tool.
3. **Object physics matched?** Yes (0.1 kg T, matched inertias, μ pairs 0.3/0.5/0.5 both sides;
   physics manifests per scene).
4. **Start/goal matched?** Yes — upstream pose pairs verbatim, rigid identity (the historic
   dairlib y×0.5 registration was found and rejected; obstacle SDFs were discovered to sit in a
   +0.15/+0.11 m shifted legacy frame and were re-issued in the OIM-native frame before any
   comparison run).
5. **Same tasks?** For the three included scenes, yes (`scene_equivalence_matrix.csv`
   valid_for_comparison=TRUE with the tool difference noted).
6–7. **icra_sign / ycb_clutter?** EXCLUDED from performance claims: icra awaits the
   xArm6-embodiment visual gate (Agent D's faithful port is merged and env-gated, glyph/object
   fidelity CSVs in scene_fidelity/); ycb's C++ reconstruction still lacks 2 of 4 obstacles.
8–9. **Frozen baseline, no post-freeze tuning?** Yes — only §2-permissible scene definitions were
   added; the health check reproduced open_table success post-merge.
10. **Same time budget?** Yes, 100 sim s both methods.

## Results (30 C3+ trials, 15 OIM trials)

| Scene | Method | Success | Median T_goal | Median final XY | Median yaw | Primary failure |
|---|---|---:|---:|---:|---:|---|
| open_table | C3+ | 6/10 | 70.8 s | 0.033 | 0.073 | F10 timeout-progressing |
| open_table | OIM | **5/5** | **16.4 s** | 0.048 | 0.056 | — |
| single_obstacle | C3+ | 1/10 | 42.9 s | 0.525 | 2.637 | F4 fixed point / F1 acquisition |
| single_obstacle | OIM | **5/5** | **15.2 s** | 0.029 | 0.075 | — |
| shelf_gap | C3+ | 0/10 | — | 0.445 | 2.715 | F4/F1 TRUE_STALL |
| shelf_gap | OIM | **5/5** | **17.6 s** | 0.036 | 0.083 | — |

11–15. OIM succeeded in **15/15** trials at 15–18 s median; C3+ in 7/30. When both succeed, OIM is
   ~4× faster; C3+ finishes tighter in position (0.018–0.041 m vs OIM's 0.03–0.05 near its own
   gate). C3+ failures: the acquisition/duty-cycle and local-fixed-point classes documented
   throughout this project — under the upstream cap its reposition-heavy cadence rarely completes
   obstacle scenes (its own historical shelf successes need 140+ s). OIM failure analysis: none
   needed (no failures); its internals were not instrumented here.
16. **Attribution:** the gap is dominated by the algorithmic contrast (sampling-MPC with dense
   parallel rollouts vs C3+'s contact-transaction outer loop) *interacting with the benchmark's
   short cap*; a documented secondary factor is the tool difference (thin stick vs sphere). It is
   NOT attributable to robot, physics, poses, or budget asymmetries — those were matched and
   verified.
17. **Reproducible advantage:** yes on the included scenes — OIM 5/5 vs C3+ 1/10 and 0/10 on the
   obstacle scenes is decisive (Wilson 95% CIs disjoint); open_table is suggestive (5/5 vs 6/10).

## Caveats (integrity)

- The 100 s cap is the upstream benchmark's and was applied identically, but it is *the* binding
  constraint for C3+; a longer-cap ablation would quantify how much of the gap is speed vs ability.
- OIM n=5/scene (single seed per the standing no-multi-seed rule; OIM is seed-deterministic).
- Two of five scenes excluded pending fidelity; no claims about them.
- C3+ trials 4b/5b etc. ran under 3-way CPU parallelism (user-authorized); sim-time scoring holds.

## Verdict

**B. OIM_OUTPERFORMS_C3PLUS_UNDER_MATCHED_XARM6_BENCHMARK** (on the three fidelity-verified
scenes, under the upstream 100 s cap and tolerances; ycb_clutter and icra_sign excluded pending
task fidelity).
