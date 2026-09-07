# xArm6 5-joint kinematic-trap & throughput report — CLOSED WITH GATE PASS

**Date:** 2026-09-06. Frozen r5 config: `r5_frozen_configuration.yaml`. All diagnostics offline or
probe-only; policy untouched throughout.

## Answers to §17

1. **Trapped targets kinematically feasible?** YES — exact IK (2 mm + 0.1 rad tilt) succeeds from
   all 8 seeds on every one of the 5 detected trap events (`trap_reachability_classification.csv`).
2. **Different posture needed?** NO — feasible from the current posture too.
3. **Jacobian near singular?** NO — σ_min(Jw) 0.066–0.083, far above the 0.03 bar
   (`trap_svd_decomposition.csv`, `figures/sigma_min_vs_stall.png`).
4. **Unachieved motion in a near-null direction?** NO — only 16–24% projects onto σ4/σ5.
5. **λ=0.05 suppressing it?** NO — damping filter factors ≥0.994; the λ sweep changes predicted
   speed <10% and flips no stall (`damping_counterfactual.csv`).
6. **wx/wy rows blocking translation?** NO — translation-only counterfactual speed identical to the
   5-row solve on every event (`task_row_conflict_analysis.csv`).
7. **Direction-preserving saturation faithful?** YES — cos 0.97–0.99999, magnitude ratio 0.98–1.04
   (`saturation_taskspace_effect.csv`).
8. **The two 1200 s runs truly stalled or slow?** TRUE_STALL — the "progressing" premise was
   falsified (last object motion at t=85–496 s; `throughput_time_to_goal.csv`).
9. **Justified validation budget?** 2400 s wall with a 300 s no-progress early stop
   (`time_budget_recommendation.md`) — used for the gate; final benchmark shares one cap between
   C3+ and OIM on the same robot.
10. **Residual NaN cause?** N4 planner-internal overflow (predicted-x0 extrapolation the prime
    suspect), with every recorded external signal finite (`nan_first_bad_signal.json`); root
    dynamics driver was the chatter (below), now removed.
11. **The minimal executor corrections justified by evidence** (two distinct measured defects, each
    proven then fixed then probe-verified):
    a. **Armature restoration** — MJCF `armature=1` was silently lost (the parser imports zero
       actuators, so the code-added actuators had rotor inertia 0; measured I_eff at chatter
       0.027 kg·m² vs 1.0). Result: a 500 Hz period-2 bang-bang servo chatter (±0.6 rad/s, ±32 N·m,
       q static) consuming all torque authority. Fixed via `set_default_rotor_inertia(1.0)`;
       chatter eliminated in the live probe.
    b. **Terminal-hold v_des** — beyond the trajectory's time range Drake extrapolates the last
       segment; captured live: `v_des == −Kc·(p_des − p_tip)` to 4 significant digits, exactly
       cancelling the feedback and nulling v_task — the true identity of the "kinematic trap"
       stalls (K3). Fixed by zeroing v_des outside the range, matching the Panda OSC's
       FirstOrderHoldWithTerminalHold semantics.
12. **Policy preserved?** Exactly — both fixes are plant-substrate/executor-internal; no planner
    message, candidate, target, or decision changed.

## Acceptance gate (§14) — PASSED

Round r6 (5 serial trials, 2400 s budget, matched physics, release on, guards on):
**3/5 full successes — 84.2 s / 161.2 s / 96.5 s** (final errors 0.011–0.020 m, 0.075–0.099 rad),
i.e. Panda-class completion (Panda reference 49.9–131.2 s). Failures: trial 3 (0.049 m/0.868 rad)
and trial 5 (0.107 m/0.510 rad) at 1560–1670 s — endgame-convergence shortfalls of the same
character the Panda stack shows on unlucky draws, not a new xArm6 execution class.
`xarm6_panda_equivalent_baseline.yaml` is FROZEN (§15). No further open_table tuning.

## Verdict

**A. EXECUTOR_MAPPING_DEFECT_IDENTIFIED** (two of them: plant-substrate inertia fidelity and
trajectory-boundary velocity semantics) — and with both repaired, the faithful 5-joint port passes
the acceptance gate at reference-class speed.
