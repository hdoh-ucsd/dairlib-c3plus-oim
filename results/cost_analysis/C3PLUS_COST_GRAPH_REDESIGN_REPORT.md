# C3+ cost reconstruction and evaluation-graph redesign (Task A)
2026-09-08 · offline analysis of the existing 30-run 600 s campaign · no reruns.
Inputs: run metrics/traces/logs, `failure_audit.csv`, episode audit; source at the
frozen baseline. Artifacts in this directory + `<run>_cost_diagnostics_v2.png`
beside every run (old graphs untouched).

## Answers

1. **True translation cost.** Local C3 object-position cost at the measured
   state: two-phase. Pre-goal-latch (until e_pos < 0.25 m, latched):
   J_trans = 12500·(Δx²+Δy²+Δz²); post-latch: 10000·Δx² + 10000·Δy² + 6000·Δz²
   (w_Q=50 × q_pos [200,200,120]). No time-varying/ramped weight exists in the
   controller (γ=1.0, no final-QP boost in this stack); the min(1+0.005k,30)
   ramp is evaluation-only instrumentation. Raw e_pos is retained alongside.
2. **True orientation cost.** Quaternion-error Q-block with weight 510 (5
   pre-latch); planar-yaw equivalent J_rot = 510·wrap_to_pi(θ−θ_goal)², exact
   to second order — the actual quaternion form is documented in
   `cost_semantics_notes.md` and the yaw derivation is labeled as derived.
3. **Does C3+ have an obstacle objective?** Not in the local C3 solve —
   **CASE C**. The solve is obstacle-blind (`Solve(...) // PASS 1 (nominal,
   obstacle-blind)`, sampling_based_c3_controller.cc:2337; obstacle-aware
   PASS-2 exists but is env-gated OFF in the frozen baseline).
4. **If yes, where?** The obstacle objective lives in **candidate ranking**:
   J_obs_rank = Σ_k Σ_obs 5000·exp(−(‖p_center−p_obs‖−r)/0.04) over the N+1
   rollout knots (cc:2515-2536). It is plotted as "J_obs (ranking layer)" and
   never summed into J_C3_task. Note: it measures object-CENTER clearance,
   unlike the footprint-witness geometry used by nonpen/lcs_contact.
5. **How are obstacles otherwise handled?** (i) `lcs_contact` mode (this
   campaign): two frictionless complementarity slots in the LCS — constraints,
   no objective; the ranking exp term is disabled in this mode for the LCS
   pass but the scenario-level exp cost applies in ranking per the audited
   guard (see notes for the exact mode matrix); (ii) legacy `qp_halfspace`:
   one frozen linear halfspace per obstacle from the measured pose;
   (iii) hard candidate filters: repositioning PWL path veto (cost:=1e12).
6. **New right-hand graph contents.** J_trans, J_rot, J_C3_task = J_trans+J_rot
   (thick black), and J_obs (ranking layer, dashed) on a symlog axis;
   tip_z/tilt/align/approach/effort removed (they remain in the old
   `*_eval_metrics.png`, kept as the debug figure). Left panel: e_pos/e_yaw
   with 0.05 m/0.10 rad thresholds and success/timeout/crash markers. Both
   panels carry C3-episode shading and contact on/off ticks.
7. **Is the total meaningful?** J_C3_task = J_trans+J_rot is a genuine sum
   inside one decision objective. A three-way total including J_obs_rank is
   NOT mathematically meaningful (different layer) and is not drawn.
8. **Success (open_task p2).** J_C3_task staircases 8635 → 27; every drop
   aligns with a shaded contact-rich episode; cost–progress Pearson r ≈ 0.88
   (campaign median across runs: 0.877).
9. **F1 (ycb p2).** Cost never moves: J_C3_task flat at ≈7398 for the whole
   run; contact never acquired. The measured cost is HIGH and flat — the
   stall is not explained by a deceptively low measured cost.
10. **F2 (single p1).** One productive push (~700 steps), then ~23k steps of
    C3-shaded, contactless, perfectly flat J_C3_task ≈ 2019 with J_obs pinned
    at 1646 (parked in the obstacle skirt). 12/13 episodes contactless.
11. **F6 (icra p5).** J_obs is the dominant component for essentially the
    whole run (mean 2316 vs J_trans 1619 / J_rot 1868), peaks ≈2635 and pins
    there when progress stops (~step 5600); the run's cost–progress r is the
    campaign's only strongly negative value (−0.38) because the rotation
    collapse sends J_rot 0.05 → 2900. Obstacle geometry dominates before the
    stall — consistent with the F6 classification.
12. **F12 (shelf p5).** At the cap, position is solved (J_trans 19,
    e_pos<0.05) while J_rot plateaus at ≈26 (e_yaw≈0.22) for the final ~8000
    steps; J_obs decayed 2046 → 4 after the gap transit. Progressing, not
    finishing; obstacles not binding at timeout.
13. **Does cost explain the failures?** Mostly NO in the causal sense the
    outer loop needs: 66% of C3-like episodes are contactless; contactless
    episodes realize >0.01 progress only 0.5% of the time (vs 32.1%
    contact-rich; mean realized progress 0.0006 vs 0.084). Measured-state
    costs correlate well with progress when contact exists (median r 0.877).
    The failures arise because the assumptions under which the decision cost
    was evaluated (imminent contact) were not physically realized — matching
    the F1/F2 forensics. This is a property of the outer loop's optimistic
    contact belief, not of the cost formulas.
14. **Can all 30 plots be reconstructed offline?** YES — done; all
    `<run>_cost_diagnostics_v2.png` rendered from recorded state + scene
    configs, no simulation.
15. **Targeted rerun needed?** NO for everything delivered. One genuinely
    unreconstructable signal exists: the per-candidate PREDICTED costs the
    outer loop compared at decision time (the controller's passive CostLogger
    was never enabled — `SAMPLING_C3_COST_LOG_DIR` unset in all campaign
    scripts). All prediction-flavored analyses here use the episode-start
    measured-state cost as a clearly-labeled weak proxy. IF a future task
    needs true predicted costs, the minimal path is 1-2 reruns (e.g.
    single_obstacle p01, ycb p02) with only that env var set — no code
    change. Not launched; not required for this task's conclusions.

RERUN_REQUIRED = NO

## FINAL VERDICT

A. COST_GRAPH_REDESIGNED_OFFLINE_NO_RERUN
