# 7-DOF → 6-DOF redundancy gap analysis

## Panda OSC task inventory (franka_osc_controller.cc + shared osc_params.yaml)

| Task | DOF | Gains | Priority |
|---|---|---|---|
| EE translation (end_effector_tip) | 3 | W=I3, Kp=200, Kd=20, accel clamp ±10 | PRIMARY |
| `panda_joint2` posture (const 1.1 rad) | 1 | w=1, kp=200, kd=10 | SECONDARY — **the explicit redundancy resolver** (source comment :182-184) |
| EE orientation | 3 | W=10·I3, Kp=800, Kd=40 — **always in the QP** (the yaml `track_end_effector_orientation:false` only gates the trajectory source) | SECONDARY, heaviest gains |
| EE external force | — | W=I3 | feed-forward |
| accel/input regularization | — | W_accel 0.01×7; W_input_reg [1,...,1,10] (wrist roll ×10) | regularizer |

No explicit nullspace projection exists; redundancy is resolved softly through the weighted QP.

## DOF budget

Panda: 3 (trans) + 3 (rot) + 1 (joint-2 posture) = 7 objectives on 7 DOF — exactly determined.
xArm6: the same stack is over-constrained by exactly 1; there is no null space for a posture task,
and a kp=200 joint posture cost on a 6-DOF arm fights the PRIMARY translation task directly —
corrupting exactly the EE tracking the ranking rollout (cost_type 5, Kp_rollout [100,100,50]) assumes.

## What the 7th DOF buys the Panda

1. The joint-2 posture task (elbow placement / configuration bias) — **cannot exist on xArm6**.
2. Absorption of unreachable wrist-orientation requests — on xArm6 an unreachable orientation becomes
   a tracking error on the primary task.
3. The trailing W_input_reg=10 on the Panda wrist roll (joint 7) — no analogue.

## Which orientation components matter for pushing

The tool is a vertically-held sphere-tipped peg; the planner's LCS EE has only 3 translational DOF
(include_end_effector_orientation=false) and the commanded quaternion (sampling_based_c3_controller.cc:3516-3549)
is a ≤20° tilt about a horizontal axis whose z-component is zero by construction — **the policy never
commands tool roll**. Necessary components: the two tilt DOF (keep the peg near vertical for contact
height and shaft clearance). Tool roll is physically irrelevant (sphere contact).

## xArm6 task hierarchy (execution adapter, no outer-loop change)

PRIMARY: EE xy(z) translation (unchanged gains). HIGH: z. SECONDARY: tilt (rot x,y, W=10, Kp=800).
SOFT/FREE: tool roll — `EndEffectorRotW` z-entry set to 0 in osc_params_xarm6.yaml, recovering the
1 slack DOF the Panda got from joint redundancy. `panda_joint2` posture task gated off for xarm6.
Formulation is the same weighted task-space QP (OperationalSpaceControl) — only weights and the task
set differ, below the Cartesian interface.

## Cannot be simultaneously satisfied on xArm6

Full 6D EE pose + any posture preference. Resolved by dropping posture (delete task) and freeing tool
roll (weight 0). No outer-loop semantics are touched.
