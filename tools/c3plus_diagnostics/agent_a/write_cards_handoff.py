#!/usr/bin/env python3
"""Write Agent A failure cards + handoff JSON (data assembled from the audit)."""
import json
OUT = '/root/push_anything_ADMM/results/parallel_c3plus_audit/agent_a_stepwise'
cards = [
 {"run_id": "c3plus_p5_fix_validation/single_obstacle_fix/plain/draw0", "commit": "43d5efb96",
  "first_irreversible_failure_time": 25.0,
  "symptom": "object parks north of obstacle at 0.40 m; never improves for 185 s; 167 mode flips",
  "primary_failure_class": "S2_LOCAL_FIXED_POINT",
  "secondary_failure_classes": ["S6_PREDICTION_EXECUTION_MISMATCH"],
  "decisive_measurements": {"best_xy": 0.399, "plateau_duration_s": 185, "mode_flips": 167,
    "probe_S1_finding": "at plateau cycle all 3 alternates hard-filtered 1e12; only productive bank contact (stem_tip circumnavigation +8mm) never drawn"},
  "earliest_causal_event": "t~25s: object reaches obstacle north side; direct sub-goal points through obstacle; feasible pool empties",
  "confidence": 0.8, "missing_signals": ["per-candidate veto-reason logging"]},
 {"run_id": "c3plus_p5_fix_validation/single_obstacle_fix/plain/draw1", "commit": "43d5efb96",
  "first_irreversible_failure_time": 49.5,
  "symptom": "planner hard abort: EE crossed outer workspace radius chasing over-pushed object",
  "primary_failure_class": "S4_WORKSPACE_ABORT",
  "secondary_failure_classes": ["S6_PREDICTION_EXECUTION_MISMATCH"],
  "decisive_measurements": {"last_ee_radius": 0.748, "obj_x_at_abort": 0.617, "obj_tilt_transient_deg": 32.7},
  "earliest_causal_event": "t~14s: 33-deg tilt transient sends object +x; chase crosses r=0.75",
  "confidence": 0.85, "missing_signals": ["commanded-vs-measured EE at abort cycle"]},
 {"run_id": "c3plus_p5_fix_validation/single_obstacle_fix/plain/draw2", "commit": "43d5efb96",
  "first_irreversible_failure_time": 79.3,
  "symptom": "T topples to 123 deg during a push while converging (0.60->0.21 m)",
  "primary_failure_class": "S5_TOPPLE", "secondary_failure_classes": [],
  "decisive_measurements": {"max_tilt_deg": 123, "best_xy_before": 0.21,
    "replay_evidence": "crossbar-bottom bank contacts topple T at 90 deg in deterministic replay; no topple filter exists"},
  "earliest_causal_event": "final push engages crossbar-underside-class contact; no tilt guard",
  "confidence": 0.85, "missing_signals": ["contact witness at topple onset"]},
 {"run_id": "c3plus_variant_cost_probe/short_online_validation/so/V0_plain/draw0", "commit": "0abc0eb7c",
  "first_irreversible_failure_time": 180.3,
  "symptom": "near miss 0.056 m / 0.50 rad when the wall cap ended the run",
  "primary_failure_class": "S7_PUSH_PRODUCTION_PLATEAU",
  "secondary_failure_classes": ["S9_RUNTIME_OR_COMMUNICATION_ABORT"],
  "decisive_measurements": {"final_xy": 0.056, "yaw_err": 0.496, "c3_frac": 0.52},
  "earliest_causal_event": "slow monotone progress; ended by harness wall-cap, not controller",
  "confidence": 0.7, "missing_signals": ["longer-horizon run"]}]
json.dump(cards, open(OUT + '/single_obstacle_failure_cards.json', 'w'), indent=1)
h = {"base_commit": "0abc0eb7c68f18070d03893e870c02159805a0cc",
 "repeated_attempt_count": 4,
 "sequence_similarity_verdict": ("BEHAVIORALLY_SIMILAR - same contact sector (-63..-65 deg), same "
   "selected id, near-identical 241 mm sweep; NOT numerically identical (dU_inf 0.19-0.77, dx0_inf "
   "~0.19): fresh solves of a re-created problem, not cached solutions"),
 "first_layer_of_repetition": ("reposition/contact acquisition (arrival with 27 mm pusher-object gap; "
   "push sweep misses by ~10 mm); persistence layer: outer-loop memory (no unsuccessful-buffer add on "
   "kToReposUnproductive, sampling_based_c3_controller.cc:2647)"),
 "snapshot_paths": ["results/parallel_c3plus_audit/agent_a_stepwise/c3_candidate_solutions_shelf_draw0.json",
   "results/c3plus_variant_cost_probe/probe_states/"],
 "selected_candidate_ids": ["shelf draw0 attempts 2-5: selected_id=2, sector -63..-65 deg, obj-frame ee ~(0.074,-0.155)"],
 "alternative_candidate_ids": ["attempt 6 breaker: sector +24 deg; audit replay D: crossbar_east"],
 "single_obstacle_failure_cards": ["single_obstacle_failure_cards.json - four runs, four distinct classes: S2/S4/S5/S7"],
 "plant_questions_for_agent_b": [
   "Why does the executed EE sweep clear the object by ~10 mm when the C3 plan predicts contact (OSC tracking vs planned clearance)?",
   "Why does measured EE z dip to -0.012 m (below table) during push sweeps - command or measurement?",
   "What lateral force does a crossbar-underside contact produce before topple (threshold)?"],
 "cost_questions_for_agent_c": [
   "Should kToReposUnproductive record the failed contact (memory gap enables the repetition)?",
   "Why does the argmin re-prefer the same sector 4x (static landscape + zero travel cost)?",
   "Is the 27 mm arrival standoff a candidate-geometry offset or an arrival-tolerance issue?"]}
json.dump(h, open(OUT + '/agent_a_handoff.json', 'w'), indent=1)
print('cards + handoff written')
