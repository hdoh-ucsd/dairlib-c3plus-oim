# xArm6-native controller baseline validation report

**Date:** 2026-09-05. **Branch:** `audit/xarm6-plant-fidelity`; `oim_t` sources verified identical to `c08e80611` (gates 380–388 landing). **Outputs:** `results/xarm6_native_controller_validation/` (+ run data at `/root/push_anything_ADMM/results/xarm6_native_controller_validation/`). One causal code change was made and validated (below); no Q/R, route, cost, horizon, friction, or OSC-gain changes.

## Provenance (run_manifest.yaml)

xArm6-native proven: `xarm6_sim` loads `xarm6.xml` (6-DOF xArm6, effort limits [50,50,32,32,32,20] N·m, joint-4 spring), channels `XARM_STATE_SIMULATION`/`OIM_XARM_TRAJECTORY` — distinct binaries, channels, and robot from the Franka `push_t_bt010_*` demos. Task: T from (0.381, 0.400, 0) to (0.381, −0.400, π); tolerances 0.05 m / 0.10 rad. Rates: sim 2 ms, OSC 500 Hz, planner update 20 ms, object feedback 20 Hz.

## Answers

1. **Plant valid?** YES — object/geometry/inertia verified (`xarm6_native_model_comparison.csv`); object rests exactly on the table; 22.8 N contact forces at sub-0.1 mm depth behave physically.
2. **Cartesian tracking?** YES — all acquisition waypoints reached in every trial; zero OSC warnings; gravity-hold drift 0 rad.
3. **Exact-contact translation?** YES — measured 12.3 s, 22.8 N initial-contact episodes move the object every trial (~5–7 cm net per run with the fix).
4. **Rotation?** YES — 0.30–0.43 rad net per run, correct sign.
5. **C3 prediction vs actual?** Not per-push measurable from this stack's logs (predicted deltas are not co-logged); structurally the DEFAULT reduced planner is frictionless/table-free with a fictitious 1 kg pusher — prediction fidelity is bounded by model construction, documented in the comparison table.
6. **Contact acquired reliably?** YES for the initial contact (5/5 trials engage and hold ≥ 566-update dwell credit windows); the arc's earlier failure was not acquisition but what happened after a 60 ms contact flicker.
7. **Full outer loop completes open_table?** **NO — 0/5.** With the fix the loop now runs its recovery + measured-cycle machinery (2 cycles per trial, previously 0), but exits at ~3–5 k of 60 k updates when cycle-recovery admission returns NULL.
8. **Earliest failure layer before the fix?** Outer-loop exit handling: a `physically_lost_latch` dwell RESET (3-update / 60 ms debounce) silently zeroed engagement without closing the transaction, so the single-shot waypoint loop ended with `initial_contact_response_complete=0` and the entire recovery/cycle machinery (gate at controller line 2603) was unreachable; the run then idled 81% of its budget in the exact-budget terminal hold.
9. **Exact minimal fix?** One change in the RESET branch (`xarm6_sampling_c3_controller.cc` ~line 2502): on `physically_lost_latch` after nonzero dwell, close the transaction as complete-but-unproductive (`initial_contact_response_complete=true`), routing control into the existing recovery + measured-cycle machinery. Validated: recovery phases execute, 2 measured cycles per trial, net object motion ~4× the pre-fix runs, `productive_handoff_preserved=1` in 5/5.
10. **Stable enough to serve as a comparison baseline?** **NO.** The next earliest layer is cycle-recovery candidate admission: at the post-cycle selector, 109/128 candidates are removed by the polarity (|normal_W.x|>0.5) and central-side pre-filters and the remaining 19 are rejected with zero predicted progress (`corridor_entry_knot=-1`); lateral recovery then fails its 2.0 mm reserve by 0.27 mm and the loop ends. Beyond that blocker, the measured throughput ceiling — one 566-update (11.3 s) dwell transaction per ~5–9 mm of object motion, plus release/settle overhead — makes 0.8 m + 180° structurally unreachable for this outer loop in any practical budget. These are pre-registered-threshold and architecture questions, not further one-line fixes; the user has independently judged this outer loop wrong (gate receipts ordered erased 2026-09-06).

## Recommendation

Do not build the comparison group on this outer loop. The Franka C++ outer loop completes the same class of task (open_table 3/3 in this audit's environment validation); the fair-comparison path is to port that outer loop (or the reference's) onto the validated xArm6 plant/OSC substrate, which this report certifies as sound. `xarm6_native_baseline.yaml` is intentionally NOT frozen — freezing an 0/5 controller would enshrine a non-baseline.

## Verdict

**B. XARM6_NATIVE_PLANT_VALID_BUT_CONTROLLER_NOT_YET_RELIABLE**

(Plant, OSC, Cartesian control, contact acquisition, and push physics all validate; the gates-arc outer loop remains the blocker: fixed at its first silent-exit layer this session, still admission- and throughput-limited at the next.)
