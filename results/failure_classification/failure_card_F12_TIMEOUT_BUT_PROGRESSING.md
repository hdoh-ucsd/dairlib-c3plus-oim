# Failure card — F12 Timeout but progressing
**Representative run:** shelf_gap/pair05 (s05→g05), TIMEOUT_PROGRESSING, confidence HIGH

## Key numbers
- Duration 324.8 s trace (600 s wall cap); best e_pos **0.0737 m** / best e_yaw **0.0602 rad** — within 2.4 cm of the position gate
- Terminal-120 s window still improving (best-error improvement above the 1 cm / 0.05 rad thresholds)
- physical_contact_fraction 0.170 with 70 contact episodes; 48 repositions; phantom_churn 0.37 (F2 secondary — throughput tax)

## Where to look
- Eval graph: `results/xarm6_c3plus_scene_smoke/runs/shelf_gap/pair05/xarm6_c3plus_shelf_gap_s05_to_g05_eval_metrics.png`
- Video `.../shelf_gap/pair05/xarm6_c3plus_shelf_gap_s05_to_g05.mp4`: **3:30** (productive pushing bursts through the gap), **4:40** (error still shrinking), **5:20** (cap fires with the object nearly at goal)

## Narrative
Classic throughput failure: pushing bursts are productive and errors decrease monotonically into the terminal window, but ~37% of runtime is lost to contactless no-progress C3 episodes, so the run cannot close the last 2.4 cm before the cap.

## Causal explanation
The 600 s cap censors a run on-track to succeed. Counting this as a capability failure of C3+ would be wrong; it is a throughput/cap artifact with an outer-loop tax.
