# Shelf-gap plateau divergence audit (Agent B — plant/execution fidelity)

Populations:
- `fixval` = /root/push_anything_ADMM/results/c3plus_p5_fix_validation/shelf_gap_fix/plain/draw{0,1,2} — **same settings**; draw0 SUCCESS (pos_err 0.018 m @ t=139.8 s), draw1/2 plateau ~0.29/0.31 m. Centerpiece comparison.
- `cmp_full` = obstacle_pushing_comparison_2026-09-06/shelf_gap/4_route_guidance_full_stack/draw{0,1,2} — all fail 0.31–0.45 m.
- `cmp_base` = .../1_baseline_no_route_guidance/draw{0,1,2} — secondary reference.

## Schema honesty (what the data actually contains)
- **No direct pusher-object gap or contact boolean exists anywhere.** `gap_state_trace.csv` and `obstacle_lcs_contacts.csv` are object-vs-OBSTACLE signals; `min_ctrl_phi` in metrics is obstacle phi.
- Actual EE position was recovered from `candidate_ranking_costs.csv` `candidate_id==0` rows (the current-location candidate; verified smooth ~30–70 Hz trace). `controller_cycle_costs.csv` `sel_ee_*` is the SELECTED SAMPLE pose (teleports ±8 cm between ticks) — not the plant EE.
- "Contact" is therefore a **proxy**: planar EE-to-object-CENTER distance d(t) ≤ d_c, with d_c calibrated per draw as the 5th percentile of d during object motion (>1 cm/s) + 2 mm (d_c came out 0.030–0.044 m; the T half-extent makes a 2 mm surface-gap threshold unattainable from these logs). Contact-fraction numbers are comparable within this audit, not absolute.
- `candidate_route_progress.csv` is **only populated in the cmp_full arm** (0 bytes in cmp_base and fixval) — prediction-vs-execution (Q4) is answerable only there.
- `osc.log` in every draw contains only startup info + a benign graphviz `Error:` line. **Zero tracking/saturation/limit warnings in any OSC log** — no evidence of low-level tracking failure.

## Q1 Contact acquisition (contact_acquisition.csv)
| pop/draw | contact_frac | n_ep | med_ep_s | productive 0.5s-window frac | final_xy |
|---|---|---|---|---|---|
| fixval/draw0 (SUCCESS) | 0.047 | 19 | 0.35 | **0.131** | 0.018 |
| fixval/draw1 | 0.038 | 9 | 0.18 | 0.049 | 0.291 |
| fixval/draw2 | 0.022 | 12 | 0.34 | 0.043 | 0.306 |
| cmp_full/draw0 | 0.020 | 6 | 0.20 | 0.032 | 0.364 |
| cmp_full/draw1 | 0.475* | 9 | 0.49 | 0.017 | 0.306 |
| cmp_full/draw2 | 0.008 | 4 | 0.43 | 0.024 | 0.450 |
| cmp_base/draw0 | 0.014 | 11 | 0.18 | 0.045 | 0.199 |
| cmp_base/draw1 | 0.494* | 3 | 0.66 | 0.012 | 0.431 |
| cmp_base/draw2 | 0.007 | 4 | 0.35 | 0.025 | 0.311 |

\* cmp_full/draw1 and cmp_base/draw1 are "parked-on-object" plateaus: EE sits within d_c of the object for 140+ s while the object is frozen (contact-proxy ≈1.0 in bins t=30–150 s, southward rate 0.02 mm/s). High proxy-contact with zero productivity.

Headline: the success run is not distinguished by contact fraction but by **productive windows** — 13.1% of 0.5 s windows move the object >5 mm, 2.7–3× every failure.

## Q2 Southward transport / plateau onset
Plateau onset = first time the running-min of object y comes within 1 cm of its final value.

| pop/draw | onset_s | y at onset | south total (m) | pusher after onset |
|---|---|---|---|---|
| fixval/draw0 | 119.1 | −0.277 | 0.587 | finishing at goal (success @139.8) |
| fixval/draw1 | **84.1** | −0.011 | 0.312 | contact-frac 0.000, C3-mode 97.7%, 66 repos events, EE path rate 0.017 m/s |
| fixval/draw2 | 155.0 | +0.007 | 0.296 | contact-frac 0.008, C3 51.6%, 45 repos |
| cmp_full/draw0 | 18.1 | +0.067 | 0.243 | contact 0.001, C3 95.8%, 75 repos, EE wanders 0.043 m/s |
| cmp_full/draw1 | 29.5 | +0.009 | 0.296 | EE parked on object (proxy-contact 0.50), C3 99.3% |
| cmp_full/draw2 | 18.1 | +0.145 | 0.156 | contact 0.001, C3 100% |
| cmp_base/draw0 | 39.9 | −0.105 | 0.405 | contact 0.002, C3 1.8% (stuck repositioning) |
| cmp_base/draw1 | 88.9 | +0.117 | 0.183 | parked on object, C3 2.9% |
| cmp_base/draw2 | 35.4 | +0.009 | 0.291 | contact 0.002, C3 0.0% |

All failed draws stall with the object **at the gap mouth (y ≈ −0.01…+0.15)**; the success run is the only fixval draw to transit the corridor (22.9 mm/s burst at t≈100–110 s, y 0.008→−0.274).

## Q3 Earliest divergence — success (fixval/draw0) vs plateau (draw1/2)
Early phase is NOT the divergence: draw1 was **ahead** of draw0 for its first 70 s (y=0.115 at t=20 s vs draw0's 0.278; both draw1/2 pre-plateau contact fractions 0.10/0.02 vs draw0 0.05). Divergence appears at **gap entry, in object yaw and subsequent contact loss**:

- draw0 arrives at the gap mouth (y≈0.06→0.01, t=60–95 s) with yaw walking to **+3.09 rad (≈177°, stem-aligned)** and then transits in one 10 s burst.
- draw1 reaches y≈0.05 by t=60 s at yaw **−2.74…−2.79 (≈−158°, ~22° misaligned)**; from t=84 s contact-proxy fraction drops to **0.000** for the remaining 110 s. First divergent signal: contact fraction in the t=80–90 s bin (draw0 0.058, draw1 0.000) together with yaw sign/offset.
- draw2 idles t=20–50 s (yaw 0.35, no approach — earliest visible anomaly for draw2), then reaches the mouth at t≈90 s at yaw −3.05→−2.70 and repeats the zero-contact stall (post-onset contact 0.008).
- Per-episode object displacement collapses to ~0 after onset in every failure (median 0.000 m; pre-onset medians also small because most episodes are grazes — transport happens in a few long productive pushes).
- **Obstacle lambda**: mean active `lambda_obs` pre-onset 0.018 (draw0), 0.078 (draw1), 0.011 (draw2); **post-onset it is 0.0000 in every draw of every population** (max 0.0) — after the plateau the planner predicts no obstacle interaction at all; the object is parked in/at the corridor and the pusher never re-engages it.
- Repositioning storm: draw1 logs 66 "Repositioning after not making progress in C3" → "reached repositioning target" → no-progress cycles, each accompanied by `!!! WARNING !!! Unsuccessful sample buffer overflow` (sample-pool starvation; 36 anomaly lines). draw0 has 2, draw2 has 2 such lines but 45 repos events.

Interpretation: the plateau is a **contact re-acquisition failure at the corridor mouth**, gated by object yaw at entry. With the stem ~20° off the corridor axis the sampler/repositioner cannot find an admissible pushing pose (buffer overflow warnings = candidate starvation), so the loop alternates C3-no-progress / reposition-to-target / still-no-contact indefinitely. No OSC-level tracking or saturation failure is involved.

## Q4 Prediction vs execution (cmp_full only; see prediction_vs_execution.csv)
Realized displacement = object displacement over 0.5 s after issue, projected on the predicted direction; rho = realized/predicted.

| draw | n_cycles | med pred (m) | med realized (m) | rho_med | rho in-contact | rho no-contact |
|---|---|---|---|---|---|---|
| draw0 | 133 | 0.0072 | −0.0003 | −0.036 | −0.000 (n=14) | −0.119 |
| draw1 | 13074 | 0.0029 | 0.0000 | 0.001 | 0.001 (n=6390) | 0.001 |
| draw2 | 9516 | 0.0017 | 0.0000 | 0.002 | 0.001 (n=181) | 0.002 |

Median rho ≈ **0.00** everywhere — selected candidates' predicted route progress is essentially never realized, in contact or not (consistent with the 2026-09-05 "dishonest predictor" verdict). Baseline and fixval arms log no route-progress rows, so Q4 is not computable there.

## Q5 OSC anomalies
grep (warn|limit|clamp|saturat|abort|error|fail) over osc.log: exactly 1 hit per draw = the benign graphviz config error at startup. sim.log: 0 hits. The only meaningful anomaly channel is planner.log's sample-buffer-overflow warnings, concentrated in the plateau phase (cmp_full/draw0: 55; fixval/draw1: 36).

Files: contact_acquisition.csv, prediction_vs_execution.csv, phase_table.csv, timeline_10s_bins.csv, scripts/analyze.py (all in this directory).
