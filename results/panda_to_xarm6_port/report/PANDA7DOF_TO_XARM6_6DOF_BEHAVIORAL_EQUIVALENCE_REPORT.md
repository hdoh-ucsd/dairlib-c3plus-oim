# Panda 7-DOF → xArm6 6-DOF behavioral equivalence report

**Date:** 2026-09-06. **Branch:** `audit/xarm6-plant-fidelity`; port commit `000eef388` + calibration
rounds (this commit). **Outputs:** `results/panda_to_xarm6_port/` (+ raw runs at
`/root/push_anything_ADMM/results/panda_to_xarm6_port_runs*`).

## Answers to §23

1. **Robot-independent:** the ENTIRE planner class `sampling_based_c3_controller.cc` (zero robot
   references, grep-verified) — C3+ solve, sampling, ranking, buffers, reposition targets, progress,
   hysteresis — plus the goal generator and the LCM Cartesian contract
   (`TRACKING_TRAJECTORY_ACTOR`: 5-knot EE position/orientation/force trajectories, world frame).
2. **Panda-specific:** `AddFrankaToPlant` (model+weld), the state decode and FK wiring in the planner
   main(), the OSC plant/tasks (incl. the `panda_joint2` posture task = the explicit 7-DOF
   redundancy resolver), and the sim plant.
3. **xArm6 adapter built:** `AddXarm6ToPlant` (MJCF minus stick tool, damping 50→1 because the OSC
   ID-QP does not model joint damping, actuators added in code, the SAME Franka end-effector welded
   with the SAME transform), `--robot_model` switch in the three binaries, demo
   `push_t_bt010_open_table_xarm6` with a 6-dim IK-matched start (tip within 0.1 mm of the Panda's),
   and a 6-DOF OSC task hierarchy: joint2 task off, tool-roll rot weight 0 (the freed slack DOF),
   translation-primary weights W=50, Kd=40, accel clamp 10.
4. **Outer-loop semantics changed?** NO code or policy-parameter changes. Two robot-calibration
   values sanctioned as robot-specific by the brief: declared reach `robot_radius_limits` 0.75→0.70
   (measured: the Panda value put targets at the xArm6's near-singular full extension → overshoot to
   r=0.81 and a planner workspace crash) and safe lift height 0.06→0.10.
5. **Same candidates from same states?** Structurally yes (identical binary+params+stochastic
   sampler); bit-replay across robots is not defined because the FK feed necessarily differs.
6. **Same contact/reposition targets?** Yes by construction (identical generation code and contact
   geometry — same EE tip sphere, same weld, `panda_xarm6_contact_geometry.csv` all-identical).
7. **Same physical contact acquired?** Push-phase yes; the divergence is the STATE at the policy's
   reposition call: Panda leaves contact ~9.6 cm from the object center (free space), the xArm6 is
   still wedged ~4.5 cm from center (inside the face) because its lateral tracking lags the
   fast-rotating T.
8. **Comparable Cartesian push?** Yes while in contact: round-7 trials complete the ENTIRE rotation
   phase (final yaw errors 0.0004–0.23 rad; the Panda reference also spins the T at up to 6 rad/s)
   and produce 0.16–0.18 m of commanded translation progress.
9. **Panda behaviors that depend on redundancy:** the joint2 posture task (elbow placement), the
   absorption of unreachable wrist requests, and — decisively — surplus tracking authority: with 7
   DOF the QP satisfies translation AND orientation AND posture; on 6 DOF the same weighted stack
   sacrificed translation (W=1 vs rot W=10·Kp800), measured as startup runaway and 2 cm push-phase
   wedge.
10. **Handled without outer-loop changes:** delete the posture task, free tool roll (weight 0), make
    translation primary by weight (W=50) — all below the Cartesian interface.
11. **Exact-contact replay:** arm-in-loop harness still absent (also for Panda); scripted-pusher
    plant-side replays (prior audit) are deterministic and productive both directions. The r7 runs
    show productive in-contact pushing; the failure is confined to one transition.
12. **Full open_table success?** **NO — 0/5 in every round** (best config r7: full yaw + no crashes).
    The single remaining blocker, reproduced in 20+ trials and localized to one transition: when the
    policy commands a reposition, the xArm6's tip is still wedged against the T (lateral lag), and
    the PWL lift drags up the face at the sim's effective μ=1.0, scooping the T over (roll 90°)
    within ~0.5 s; a lying-down T then also exposes a pre-existing policy-side infinite sampler loop
    (PerimeterSampling spins at 100% CPU — present in the Panda code too, just never triggered).
13. **Valid comparison baseline?** Not yet. Policy equivalence: YES (by construction). Execution
    equivalence: NO at one transition.

## The seven measured rounds (one causal change each)

| Round | Change | Result |
|---|---|---|
| 1 | as-ported (Panda gains + Kd40) | crash: EE overshot to r=0.81 at far-shell target / unreachable repositions |
| — | radius 0.70 calibration | crashes bounded |
| 2 | (radius applied) | topple t≈8 at first reposition-from-contact |
| 3 | tilt weight 10→1 | no change — tilt stiffness exonerated (reverted) |
| 4 | lift height 0.10 | no change — scoop happens in the first cm of ascent |
| 5 | Kd back to 20 | startup workspace crash returns (z/x excursions) |
| 6 | + accel clamp 2 | crash persists — priority effect, not energy |
| 7 | translation W=50 (primary), Kd40, accel10 | **rotation phase completes fully, zero crashes; topple delayed to t=11–18 but persists** |

## Remaining work (ranked, execution-layer only)

1. **Pre-lift disengage** in the executor: before executing the PWL lift, retreat ~1–2 cm along the
   last contact normal at contact height (a below-interface acquisition/release micro-behavior,
   mirroring how the Panda's state naturally sits clear at reposition time). This is the §10-style
   "contact acquisition/release is part of the execution adapter" fix and touches no policy.
2. Verify against the μ literal: the sim's effective μ=1.0 (SDFs omit friction) makes any wedged
   lift catastrophic; the reference-conformant μ question (0.4615 intent) is documented in the
   environment-validation report — an authorized μ A/B would quantify how much of the scoop is
   friction-literal artifact.
3. Then re-run the 5-trial gate (§18: ≥3/5) and only then freeze `xarm6_panda_equivalent_baseline.yaml`.

## Verdict

**B. POLICY_EQUIVALENT_BUT_XARM6_EXECUTION_STILL_LIMITED**

Same Panda controller policy (byte-identical planner, same parameters, same contact geometry, same
Cartesian interface) → same Cartesian/contact intent → xArm6-specific execution realization that now
reproduces the rotation phase completely and crashes nowhere — but object-level behavior diverges at
exactly one execution transition (lift-from-wedged-contact → topple), which blocks repeatable task
success and therefore the baseline freeze.
