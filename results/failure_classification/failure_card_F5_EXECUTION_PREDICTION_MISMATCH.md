# Failure card — F5 Execution/prediction mismatch
**Representative run:** open_task/pair01 (s01→g01), TRANSIENT_SUCCESS, confidence MEDIUM

## Key numbers
- Latched SUCCESS at **38.1 s**; inside tolerance only **0.3 s**; exited via yaw overshoot to final e_yaw **0.130 rad** (gate 0.10) with e_pos 0.021 m still inside
- Best-ever errors 0.021 m / 0.0009 rad — the plan reached the goal essentially exactly before overshooting
- Launcher stopped at 40.7 s (~2.6 s post-latch), so recovery was never attempted (dwell censoring)

## Where to look
- Eval graph: `results/xarm6_c3plus_scene_smoke/runs/open_task/pair01/xarm6_c3plus_open_task_s01_to_g01_eval_metrics.png`
- Video `.../open_task/pair01/xarm6_c3plus_open_task_s01_to_g01.mp4`: **0:35** (final push into tolerance), **0:38-0:41** (yaw carries through the gate and out)

## Narrative
The final push carries more yaw momentum than the plan predicted; the object crosses the tolerance band and exits on the far side 0.3 s later. The launcher's ~3 s post-latch stop then censors any correction.

## Causal explanation
Executed motion overshot the predicted terminal state (F5). MEDIUM confidence because the launcher stop prevents distinguishing a one-shot overshoot from an overshoot the controller would have recovered.
