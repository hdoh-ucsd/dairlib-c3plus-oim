# Panda C3+ → xArm6 5-joint faithful port — final report

**Date:** 2026-09-06. **Branch:** `audit/xarm6-plant-fidelity`, final commit this closure (base
chain 000eef388 → 4c99df4a4 → 01def72fe → f3ed26860). Joint work with the Agent D session
(icra-sign-faithful-port branch: independent architecture/friction cross-check @323e3d8b7, r2 trial
forensics, prediction-honesty analyzer).

## Answers to §19

1. **Really 5-joint velocity controlled?** YES — source-proven and independently cross-checked:
   the MJX benchmark arm has exactly five `<velocity>` actuators (ctrl = target q̇, ±0.5 rad/s,
   kv [300,300,200,200,200]); no IK, no torque; gravcomp handles gravity.
2. **Which five joints?** xarm6_joint1–5. Joint 6 (wrist roll) is welded out of the model
   (axisymmetric stick); joint 4 is actuated with an added passive spring 175 N·m/rad.
3. **Command mapping?** `XARM6_5JOINT_VELOCITY_ADAPTER.md`: Cartesian reference → 5-D task
   velocity (translation feedback Kc=4; tilt-to-vertical bias α=2; stick yaw dropped) → weighted
   damped square 5×5 Jacobian solve (W tilt 0.2, λ=0.05) → direction-preserving q̇ saturation at
   ±0.5 → per-joint velocity servo torque + gravity comp (MuJoCo ordering).
4. **Was the previous 6-DOF OSC faithful?** NO — wrong plant (6 joints), wrong actuation (torque
   OSC), wrong task structure. Replaced below the interface.
5. **Outer-loop semantics changed?** NONE. Only sanctioned robot-specific items: declared reach
   0.70, safe lift height 0.10, plus two identical-for-both-robots termination guards (topple
   >0.5 rad; 10k sampler-iteration cap).
6. **Same candidate/contact/reposition decisions?** Structurally yes (identical binary+params;
   `policy_equivalence_after_adapter.csv`); no observed failure classifies as X10.
7. **Why did the previous port topple?** The 6-DOF OSC's lateral lag left the tip wedged at the
   policy's reposition call and the PWL lift scooped the T at μ_eff=1.0 — reproduced 20/20.
8. **Does pre-lift disengagement remove it?** YES, jointly with matched friction: the release
   phase retreats to ≥0.095 m before any lift (logged, dozens of firings), and **zero topples
   occurred in the final 10 trials** (r4+r5, ~800 s each). One graze in final-r2 (tilt 0.37) was
   SURVIVED at matched μ — friction amplified (A) and release was required (B); both quantified
   (`prelift_release_ab.csv`, `friction_ab_results.csv`).
9. **Role of μ=1.0?** The omitted-SDF Drake default made every wedged lift catastrophic and every
   graze fatal; it was a physics mismatch, not the benchmark value.
10. **Final comparison friction?** T-table 0.3, EE-T 0.5 — the OIM MJX task's explicit values
    (`friction_provenance.csv`; independently confirmed for all tabletop scenes; object-obstacle
    0.5 recorded for the obstacle stage). `--matched_mu` applies them identically to both robots.
11. **Exact-contact xArm6 motion vs Panda?** Qualitatively yes where measurable: final-r2 trial4
    drove the full 0.6 m translation task to 0.009 m through dozens of acquired contacts; the
    frozen-state replay protocol remains unbuildable (no sim state save/restore).
12. **Contact-conditioned prediction agreement?** Panda-side measured (Agent D analyzer; in-contact
    ρ positive, no-contact ≈0.09, with an ill-conditioning caveat); xArm6 per-push extraction
    awaits a successful run. Predictions are the identical computation for both robots.
13. **≥3/5 open_table?** **NO — 0/5 in all five acceptance rounds** (`xarm6_panda_equivalent_open_table.csv`).
    Best: position solved to 0.009 m (r2 t4, wall-clock truncated mid-yaw); the final rounds are
    topple-free with two trials still progressing at the 1200 s budget.
14. **Safe to freeze?** NO — `xarm6_panda_equivalent_baseline.yaml` deliberately NOT created.

## Remaining ranked failure classes (all execution-layer; none policy)

1. **X6/X2 — velocity-tracking/kinematic-trap stalls**: the damped 5×5 solve still parks at
   reachable targets in some postures (3/5 r5 trials); the weighted-rows fix reduced but did not
   eliminate it. Next lever: posture-aware trap escape or a reachability-filtered target stream in
   the executor.
2. **X4 — acquisition duty cycle** (shared with the Panda stack; amplified by the slower arm).
3. **X9 — throughput vs budget**: the ±0.5 rad/s servo arm moves the task ~10× slower than the
   Panda; two r5 trials were still converging at truncation. A budget definition for the fair
   comparison (sim-time per robot class) needs a user decision.
4. **X7 — one residual sim NaN blowup** under sustained saturated pressing; a planner-ingestion
   finiteness guard would classify it cleanly.

## Verdict

**B. POLICY_EQUIVALENT_BUT_5JOINT_EXECUTION_NOT_YET_RELIABLE**

The §16 claim is supportable for every clause except the last: policy preserved (evidence: layer
map, zero robot references, byte-copied params), robot execution replaced with a faithful
5-joint velocity implementation (source-traced, independently cross-checked), physics matched to
the benchmark (provenance + A/B), object stability solved (0 topples / 10 trials) — but
repeatable task success is not yet demonstrated, so no comparison baseline is frozen and obstacle
scenes remain out of scope (§17 untouched, per §20's gate).
