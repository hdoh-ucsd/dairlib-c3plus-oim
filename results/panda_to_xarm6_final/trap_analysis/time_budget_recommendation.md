# T_validation recommendation — xArm6 5-joint baseline-validation gate

Data: r5 trials 1–5 + r4 trial1 (throughput_time_to_goal.csv, figures/throughput_rolling_window.png).

## Measured inputs

1. **Actuator / servo bound.** Commanded joint-velocity clamp is 0.5 rad/s (osc.log header
   `qdot_limit=0.5`). At ~80–86 % servo tracking the measured EE tip speed from the 1 Hz
   XARM6_5J prints is: median 0.003–0.006 m/s, p90 0.08–0.10 m/s, max 0.124–0.131 m/s across
   all six trials. Sustained transit speed during repositioning ≈ **0.10 m/s**. 36–80 % of 1 Hz
   samples sit at |qdot_cmd| ≥ 0.45 (velocity-saturated pressing), so the arm is
   actuator-limited, not planner-rate-limited, when it moves.

2. **Object transport rate in productive windows.** Best sustained 300 s window across trials:
   net object translation **0.045–0.095 m per 100 s** (0.45–0.95 mm/s); pos_err reduction up to
   0.092 m/100 s (r4t1, r5t1); yaw-err reduction up to **0.40 rad/100 s** (r5t3), typical
   0.20–0.26 rad/100 s.

3. **Near-success calibration (r2 trial4).** The one run that essentially solved the position
   task drove pos_err to 0.009 m in ~370 s. With a 0.8 m detour-inclusive transport path this is
   an end-to-end effective rate of 0.8 m / 370 s ≈ **2.2 mm/s**, i.e. a detour+overhead factor of
   roughly 2–4× over the raw pushing bursts — reposition/prelift overhead dominates
   (phase proxy: only ~5 s/run of ≥0.3 mm object-motion bins; 70–450 s repositioning;
   the rest idle/pressing — approximated from trace structure because planner.log
   Repositioning/Switching lines carry no timestamps).

## Arithmetic

- Position leg at the **worst productive** sustained rate (0.45 mm/s):
  0.8 m / 0.00045 m/s ≈ **1780 s**. At the r2t4 demonstrated rate (2.2 mm/s): 370 s.
- Yaw leg at the **typical** sustained rate (0.20 rad/100 s): π / 0.0020 rad/s ≈ **1570 s**.
  At best measured (0.40 rad/100 s): 785 s.
- Position and yaw progress are partly concurrent (the same pushes produce both), so the serial
  sum (~3350 s) is a hard upper bound; the max of the two legs (~1780 s) is the realistic
  worst-productive-case bound. Add 25 % margin for prelift/reposition clustering:
  1780 × 1.25 ≈ 2225 s.

**Recommendation: T_validation = 2400 s (40 min)** — round, ≥ 3× the current 800 s caps,
covers the worst measured productive rate on both legs with margin.

**Nonstationarity caveat:** the rates above come from the best sustained 300 s windows early in
each run; all six audited runs then transitioned to a hard stall (object never moved again after
t = 85–500 s), so extrapolating a window rate to a full-task time assumes the controller keeps
re-engaging — which the r5/r4 runs did NOT do. A longer cap alone will not convert these runs;
it only removes time-capping as a confound.

## Stall-detection principle (so a true stall isn't hidden)

Terminate early with verdict TRUE_STALL if, over the trailing **300 s**, pos_err improved
< 5 mm **and** ang_err improved < 0.02 rad **and** net object translation < 5 mm. All six
audited runs would have triggered this by t ≈ 400–800 s (measured final-300 s progress:
1.6–3.7 mm position, ~0 rad yaw), saving 40–90 % of wall time while a genuinely progressing run
is never cut (r2 trial4's productive windows exceed the floor by >10×).

**Scope:** this cap is for baseline validation only. The final benchmark applies the SAME cap to
C3+ and OIM on the same robot, so no relative bias is introduced.
