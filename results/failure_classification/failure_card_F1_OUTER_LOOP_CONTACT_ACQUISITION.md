# Failure card — F1 Outer-loop contact acquisition
**Representative run:** ycb_clutter/pair02 (s02→g02), TRUE_STALL, confidence HIGH

## Key numbers
- Duration 323.8 s, 22 583 steps; final e_pos 0.799 m / e_yaw 2.99 rad
- acquisition_failure_rate **0.985** (65/66 episodes achieve zero contact steps)
- 72 repositions, 65 no-progress exits, equivalent_failed_reselection_rate **0.924**
- physical_contact_fraction 1.3%; contactless_c3_fraction 0.965; phantom_churn 0.352 (moderate — discrete churn cycles rather than a park)

## Where to look
- Eval graph: `results/xarm6_c3plus_scene_smoke/runs/ycb_clutter/pair02/xarm6_c3plus_ycb_clutter_s02_to_g02_eval_metrics.png`
- Video `.../ycb_clutter/pair02/xarm6_c3plus_ycb_clutter_s02_to_g02.mp4`: **1:00** (first discrete reposition→hover→reposition cycle), **3:00** (same object-frame sector re-approached), **5:15** (end, object unmoved)

## Narrative
Unlike the park runs, this run executes real repositions — 72 of them — but 98.5% of the resulting C3 stints never touch the object, and 92% re-enter an object-frame approach sector that already failed. The churn structure exists, but the dominant fact is that contact is simply never acquired in YCB clutter.

## Causal explanation
Outer-loop contact-acquisition failure sustained by failure-memory-free reselection (F3 secondary). Not evidence about local C3 capability — C3 was never given a contact to exploit.
