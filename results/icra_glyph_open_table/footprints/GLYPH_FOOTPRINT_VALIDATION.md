# Glyph footprint validation + sampling coverage

Planner polygons: hardcoded selector footprints (systems/controllers/sampling_based_c3_controller.cc); hull source = icra_glyph_obstacles.csv. Offset = pusher radius 0.0195 + clearance 0.027 = 0.0465 m. Workspace: r <= 0.7, y <= 0.6.

## I

- Max boundary discrepancy (720 rays): **0.000 mm**; mean 0.000 mm
- Perimeter length: 0.2630 m; samples: 2000
- Per-edge density [samples/m]: min 6708 (edge 2, len 0.9 mm), max 7826 (edge 1)
- Edge sample counts: [626, 7, 6, 210, 12, 766, 12, 209, 12, 140]
- Workspace rejects (of 2000): start(0.30,0.40): 0, goal(0.50,-0.40): 405

## C

- Max boundary discrepancy (720 rays): **0.000 mm**; mean 0.000 mm
- Perimeter length: 0.5284 m; samples: 2000
- Per-edge density [samples/m]: min 3777 (edge 3, len 64.6 mm), max 3812 (edge 6)
- Edge sample counts: [121, 245, 121, 244, 148, 244, 122, 244, 121, 390]
- Workspace rejects (of 2000): start(0.30,0.40): 0, goal(0.50,-0.40): 379

The C's *geometric* boundary (blue/black outline) includes the concave mouth: the inner faces of the top/bottom bars and the inner spine wall at x=-0.0163. Uniform-by-arclength perimeter sampling spends a large fraction of its budget there, but the mouth interior is only ~0.039 m wide against an EE offset footprint of 2x0.0465 m: an offset sample placed inside the mouth collides with (or is vetoed against) the opposite bar, so the *physically useful pushing boundary* is essentially the convex outer perimeter (outer spine wall, bar tops/bottoms, bar tips). Effective coverage on the C is therefore lower than the raw per-edge densities suggest; mouth-edge samples mostly burn SampleIsAcceptable attempts.

## R

- Max boundary discrepancy (720 rays): **0.000 mm**; mean 0.000 mm
- Perimeter length: 0.3870 m; samples: 2000
- Per-edge density [samples/m]: min 4642 (edge 1, len 1.1 mm), max 5216 (edge 4)
- Edge sample counts: [513, 5, 419, 87, 75, 89, 275, 6, 525, 6]
- Workspace rejects (of 2000): start(0.30,0.40): 0, goal(0.50,-0.40): 749

## A

- Max boundary discrepancy (720 rays): **0.000 mm**; mean 0.000 mm
- Perimeter length: 0.3644 m; samples: 2000
- Per-edge density [samples/m]: min 5000 (edge 3, len 1.2 mm), max 5882 (edge 4)
- Edge sample counts: [178, 12, 587, 6, 8, 596, 7, 7, 586, 13]
- Workspace rejects (of 2000): start(0.30,0.40): 0, goal(0.50,-0.40): 693
