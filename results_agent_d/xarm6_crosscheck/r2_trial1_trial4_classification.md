# r2 trial1 + trial4 forensic classification (offline, FK-verified)

Method: xArm6 FK (policyport model + end_effector_full weld, matching
AddXarm6ToPlant exactly) over state_trace.jsonl; 2D T-footprint gap; tilt from
object quaternion. Summary rows in r2_trial_summary.csv.

## trial1 — X5_CONTACT_ACQUISITION (standoff-conversion failure)

- Contact fraction **1.1%** over 382 s; productive windows 0.9%. The EE spends
  **86% of the run in the 0.5–6 cm standoff band** (gap p25–p75 = 2.0–2.5 cm —
  the candidate geometry's built-in ~2.7 cm standoff, arriving precisely and
  never converting to touch). 26 no-progress transitions, 25 reposition
  timeouts, only 1 completed reposition; final 0.446/2.52.
- This is the SAME class Agent A measured on the Franka (arrival at 26.9–27.0 mm,
  push sweep misses by ~10 mm) — i.e., NOT an xArm6-specific mapping defect and
  NOT policy divergence; the xArm6 executor tracks to the same standoff the
  policy commands. EE-z check: the arm descends correctly (z p10 = −0.003) —
  the misses are lateral, as on Franka.

## trial4 — RECLASSIFIED: time-truncated while converging (X10_TIMING), NOT a topple

- Exactly **one** trace record exceeds tilt 0.3: t=272.6 s, tilt 0.37,
  obj_z 0.015 — a ~0.2 s edge-catch/scoop that the T **survived** (tilt back to
  0.00, obj_z back to 0.0008 by the next sample; only 2 records ever exceed
  tilt 0.1). No guard trigger, run continued 116 s more.
- Post-blip the run kept converging: pos_err 0.24 → **best 0.009 m at t=367.5**
  (tilt 0.00) → trace ends t=389 at **0.017 m / 0.33 rad, still improving**.
  The wall budget, not physics, ended it. With r3's 1200 s budget this class
  converts to a plausible success (yaw 0.33 → 0.1 is the standard endgame).
- The transient at 272.6 is worth one sentence in the report as a surviving
  instance of the scoop mechanism (pre-lift release engaged? their release logs
  would confirm), but it is not the run's failure cause.

## Gate-relevant implication

Of the r2 failures: trial1 = acquisition (pre-existing shared class),
trial4 = wall-clock, trial5 = wall-clock (peer's own data), trial2 = reach
assert (fixed in r3), trial3 = topple. r3's fixes (reach 0.66/0.68 clamp +
1200 s) directly address 3 of the 5 failure modes; the acquisition class is
shared with the Franka baseline and therefore does NOT break comparison
fairness — both robots pay it.
