# Paired baseline-vs-relu stats

Matched pairs: 4 (bootstrap 10000 resamples, seed 20260908; delta = relu - baseline)

| metric | n | median delta | 95% CI |
|---|---|---|---|
| T_goal | 0 | nan | [nan, nan] |
| min_clearance | 4 | 0.025144 | [-0.00052732, 0.062544] |
| chomp | 4 | -0.00027007 | [-0.001151, 0] |
| chomp_norm | 4 | -0.00031426 | [-0.0017187, 0] |
| final_pos_err | 4 | 0.073802 | [0.005638, 0.58895] |
| final_yaw_err | 4 | 1.3562 | [-0.02272, 2.6766] |

McNemar discordant counts: b (baseline-only success) = 1, c (relu-only success) = 0
