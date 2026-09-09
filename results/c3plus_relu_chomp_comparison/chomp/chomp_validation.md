# CHECKPOINT F — CHOMP metric independent validation
2026-09-09 · `tools/relu_chomp/chomp_validation.py` · evaluation-only, no controller code touched.

| check | result | detail |
|---|---|---|
| c(d) values | PASS | max|err|=0.00e+00 |
| continuity at 0 | PASS | gap=2.00e-07 |
| C1 at 0 (slope -1 both sides) | PASS | left=-1.0000 right=-1.0000 |
| continuity at eps | PASS | c(eps±)=[4.99999999e-17 0.00000000e+00] |
| constant-clearance transit (wall poly) | PASS | M=3.600000e-04 closed-form=3.600000e-04 |
| normalization M/L | PASS | norm=1.800000e-03 |
| approach ramp eps->0 = eps^2/6 | PASS | M=1.66666672e-05 exact=1.66666667e-05 |
| penetration ramp 0->-eps = eps^2 | PASS | M=1.00000000e-04 exact=1.00000000e-04 |
| constant-clearance arc (disc) | PASS | M=5.400000e-05 closed-form=5.400000e-05 |
| scene has the env box geometry | PASS | polys=1 discs=0 (all: 1/1) |
| joined rows vs forensics | PASS | n=1943 |
| footprint SDF == offline Drake d_phys (<2 mm max) | PASS | median=0.1 um  max=0.000 mm  min d: sdf=0.65 mm phys=0.65 mm |

**Overall: PASS**

Legs: (1) analytic potential checks; (2) closed-form functional identities driven through the real `eval_run` code path via monkeypatched providers (wall poly, disc, approach/penetration ramps); (3) the CHOMP distance substrate (footprint boundary SDF) reproduces the offline Drake `SignedDistancePairs` ground truth recorded by the single_obstacle penetration forensics on the s01g01 trace, box-pair geometry only, tilted rows excluded.

Independence from the ranking costs: different functional form (piecewise CHOMP potential vs exp/ReLU), different aggregation (arc-length trapezoid over a dense fixed body-point set vs per-knot sums), computed offline from the EXECUTED trajectory, never fed to any controller.
