# OIM-side cross-check for the xArm6 5-joint port (for Agent: xarm6 execution fidelity validation)

Source: local OIM checkout `a954f00` (= upstream `main d6d80a65` for these files,
verified byte/region-identical earlier). All file:line refs below are in
`external/Object-Informed-Manipulation-MJX/`.

## 1. Architecture corroboration — your §1 verdict CONFIRMED

- **Exactly five velocity actuators**, joint 6 has NO joint: `oim/models/xarm6/xarm6.xml:201`
  ("link6 (the wrist flange): joint6 (wrist roll) is fixed"), actuators at
  `xarm6.xml:263-272`: `<velocity ... ctrlrange="-0.5 0.5" kv=300/300/200/200/200,
  forcerange=50/50/32/32/32>`. ctrl is a TARGET JOINT VELOCITY in rad/s
  (`xarm6.xml:228-231`); steady-state tracking = kv/(kv+damping=50) ≈ 80–86%
  (`xarm6.xml:240-243`, confirmed empirically upstream).
- **Action space is joint velocity directly**: the sim loop writes the sampled
  5-vector straight into the actuators — `oim/worlds/sim3d/run.py:520` and `:786`
  (`mj_data.ctrl[:] = us[i]`). The real arm path publishes the same 5 joint
  velocities with joint6 = 0: `oim/worlds/real3d/interface.py:11, :32, :40, :102, :168`.
- **The 5×5 damped Jacobian [vx vy vz wx wy] over the 5 dofs exists upstream too**
  (`run.py:119-131`): `jac = [jacp; jacr[:2]]` (5×5), damped inverse
  `solve(JᵀJ + damping·I, Jᵀ)` (`run.py:124-125`). NUANCE for your §3 doc: upstream
  uses it as the MPPI/ADMM **noise/task shaping map** (task_jac_inv), not a per-step
  Cartesian-servo IK — the optimizer's decision variable is qdot itself. Your
  Cartesian-target → damped-J → qdot adapter is an equivalent-intent realization of
  the same kinematic map (same J rows, same damping structure); worth one sentence of
  acknowledgment in XARM6_5JOINT_VELOCITY_ADAPTER.md so nobody mistakes it for a
  line-for-line transcription.
- **No torque control anywhere** on the xArm6 path; forcerange is a clamp, gravcomp=1
  passive (`xarm6.xml:121-124`), joint4 passive spring 175 via stiffness (matches
  what dairlib's `xarm_configured_actuation` reproduces).
- **No per-scene kv/ctrlrange/actuator overrides**: zero hits for kv/ctrlrange in
  `oim/utils/scenes.py` and `oim/configs/robots/xarm6.yaml`. The ±0.5 rad/s
  ctrlrange is a 2026-08-18 uniform safety ceiling (`xarm6.xml:252-261`) — global,
  not scene-tuned. Your qdot clamp ±0.5 matches.

## 2. Friction provenance — extension to all scenes (merge into friction_provenance.csv)

| component | pair | μ | source |
|---|---|---|---|
| OIM sim, ALL tabletop T scenes (open_table, single_obstacle, shelf_gap, ycb_clutter) | T–table | **0.3 0.3 0.005** explicit `<pair>` | `tee.xml:150-151`; `_tee_scene(...mass=0.1, mu=0.3)` default `scenes.py:417`, ycb entry `scenes.py:565` inherits it |
| OIM sim, icra_sign | C–table | **0.3** explicit pairs | `icra_sign.xml` `<contact>` block (c_spine/c_top_bar/c_bot_bar × table) |
| OIM sim default geom (governs EE-stick–object, object–obstacle) | any unpaired | **0.5 0.005 0.0001** | `common.xml:50` |
| OIM planner analytic | object support | μ=0.3, wrench budget μmg=0.2943 N | `scenes.py:376-377, 634-635` |
| `clutter` (non-tabletop scene family) | joints | `frictionloss` bound instead of table contact | `scenes.py:77-80` — only that family; NOT ycb_clutter |
| dairlib C++ (for contrast) | T–table | 0.4615 effective (object unspec μ1.0 × ground 0.3 harmonic) | ground.urdf:26-27 + missing SDF friction |
| dairlib C++ faithful C task (mine) | C–table | 0.3 (explicit SDF μ0.3 × ground 0.3) | push_c_glyph.sdf |

So your matched_mu targets (T–table 0.3, EE–T 0.5) are exactly the MJX benchmark
values — confirmed independently. For future obstacle scenes: object–obstacle
contacts in OIM are also **0.5** (default geom), not 0.3 — worth carrying into the
matched set when you get to §17.

## 3. §12 contact-conditioned prediction draft

`contact_conditioned_prediction.csv` (same dir): per-commitment rows
(scene, draw, t, cand, pred_dxy, act_dxy, pred_dyaw, act_dyaw, rho_xy, min_gap,
contact) over 2 s windows, produced by
`tools/c3plus_diagnostics/agent_d/pred iction_honesty.py` (this worktree) from the
P5-validation Franka draws (shelf draw0 success + draw1 plateau). Headline medians:
success draw ρ_xy|contact 1.79 (n=534), plateau draw 7.59 (n=526), both ρ≈0.09
without contact. CAVEAT for the report: ρ medians are ill-conditioned when the
predicted per-commitment displacement is ~0 (median pred 0.000–0.001 m on these
draws) — quote the absolute pred-vs-act columns alongside ρ, or filter to
pred_dxy > 5 mm rows. Methodology and the analyzer are reusable as-is for the
xArm6 runs: point it at a draw dir with state_trace.jsonl + candidate_ranking_costs.csv.
Prior-established anchor numbers you can cite: in-contact realization ρ≈0.61
(your fidelity study); my icra_faithful C-task runs: ρ|contact 0.02–0.10.
