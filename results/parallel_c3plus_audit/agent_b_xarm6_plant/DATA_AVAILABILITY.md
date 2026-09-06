# Data availability — Agent B deliverables

Deliverables produced from measured data: plant_model_comparison.{md,csv}, command_pipeline.md,
contact_acquisition.csv, prediction_vs_execution.csv, phase_table.csv, timeline_10s_bins.csv,
shelf_plateau_divergence.md, normal_lateral_microtest.csv, candidate_replay_results.csv,
workspace_abort_analysis.json, topple_analysis.json, scripts/analyze.py.

## Deliverables NOT generated as new run logs (and why)

- **osc_execution.jsonl / command_timing.csv** — no new instrumented run was executed this audit.
  The passive instruments ALREADY EXIST and are the smallest path to this data:
  - xArm6 stack: `XarmVelocityServoBridge` CSV logger (`--control_log_period`, default 0.02 s)
    records q, v, tau_osc, tau_servo, qdot_cmd, tau_gravity, tau_cmd per cycle
    (`examples/sampling_c3/oim_t/xarm6_osc_controller.cc:735-846`).
  - Franka stack: `publish_debug_info: true` in `shared_parameters/osc_params.yaml` already
    publishes the OSC debug LCM message; campaigns simply did not record that channel.
  Justification for not re-running: every osc.log and sim.log across all 9 audited draws is free
  of tracking/limit/saturation warnings, and the in-contact fidelity bound (rho ~ 0.61) caps what
  OSC-level detail could explain; the dominant loss (acquisition, rho = 0) is upstream of OSC.
- **physical_contact_pairs.jsonl / contact_mode_trace.csv** — no pusher-object signed-distance
  channel exists in the archived campaign logs (verified: gap CSVs are object-vs-obstacle only).
  A calibrated EE-to-object-center proxy was used instead (see shelf_plateau_divergence.md).
  The latent instrument for true pairs is `XarmObjectContactMonitor`
  (`xarm6_sim.cc`, publishes contact point/force/depth on OIM_T_STATE_SIMULATION_CONTACT);
  the Franka sim would need an equivalent ContactResults tap (plant already exposes the port).
- **Single-candidate state-restore replay (section 11)** — the sim stack has no state
  save/restore; population-matched windows + the scripted-pusher micro-tests substitute
  (candidate_replay_results.csv documents the mapping).
