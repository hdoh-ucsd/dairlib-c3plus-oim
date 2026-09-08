# Failure card — F6 Obstacle geometric block
**Representative run:** icra_sign/pair05 (s05→g05), TRUE_STALL, confidence HIGH

## Key numbers
- Duration 294.6 s, 15 990 steps; final e_pos 0.267 m / e_yaw 2.79 rad; **best e_yaw 0.0066 rad** at last progress (118.8 s)
- physical_contact_fraction **0.803** (campaign max) but productive_contact_fraction only **0.145**
- min_obstacle_clearance median ~ **-0.0001 m** (in obstacle contact); 98% of contact steps have clearance < 5 mm
- Only 11 repositions / 9 no-progress exits — the outer loop is comparatively healthy here; c3_contact_fraction 0.872

## Where to look
- Eval graph: `results/xarm6_c3plus_scene_smoke/runs/icra_sign/pair05/xarm6_c3plus_icra_sign_s05_to_g05_eval_metrics.png`
- Video `.../icra_sign/pair05/xarm6_c3plus_icra_sign_s05_to_g05.mp4`: **1:55** (last real progress — yaw essentially solved), **2:30** (object wedged against the C-glyph hull, sustained grinding), **4:30** (still grinding, zero error change)

## Narrative
The controller genuinely engages: yaw is driven to 0.0066 rad by 118.8 s. Then the object jams against the C-glyph obstacle hull; the pusher keeps rich physical contact for the remaining ~176 s (80% contact) but the commanded push direction is geometrically blocked — errors are dead-flat and yaw drifts back out to 2.79 rad while grinding.

## Causal explanation
Obstacle geometric block (F6) with a local-C3 flavor (F4 secondary): C3 keeps pushing into a hull it does not model as blocking. This is the closest thing in the campaign to a rich-contact fixed point, but clearance ~0 makes the obstacle, not the solver, the earliest defensible cause.
