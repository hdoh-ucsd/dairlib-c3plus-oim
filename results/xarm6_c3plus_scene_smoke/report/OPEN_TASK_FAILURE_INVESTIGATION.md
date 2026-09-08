# open_task low-SR investigation — planner starvation under parallel load
2026-09-07 · branch `feature/oim-scene-sync-metrics`

## Question
The pair campaign scored open_task 1/5 at a 200 s cap, far below the Sep-6
matched benchmark (6/10 on the same task dirs at a **100 s** cap, successes
52.9–89.4 s). Why?

## Method
Controlled rerun: the identical `matched_open_table_xarm6_t2..t5` dirs, same
binaries, same 200 s cap and upstream 0.05 m/0.1 rad scoring — but **serial,
one trial at a time on an idle machine** (`tools/scene_smoke/rerun_open_solo.sh`,
outputs `runs/open_task/solo_pair0N/`). The campaign had run them 5-to-a-machine.

## Result: solo 3/4 vs loaded 0/4

| pair | 5-lane campaign (erased tier) | solo rerun | benchmark (100 s cap, reps a/b) |
|---|---|---|---|
| 2 | fail (0.340 m/0.129 rad) | **SUCCESS 89.2 s** | 52.9 s / 83.4 s |
| 3 | fail (0.326/2.186) | fail — ang solved (0.0005 rad), best pos 0.083 m | 34.7 s / fail |
| 4 | fail (0.224/0.412) | **SUCCESS 112.7 s** | 89.4 s / fail |
| 5 | fail (0.348/1.448) | **SUCCESS 75.7 s** | 67.4 s / 74.1 s |

Control-step throughput: solo 5,600–9,100 steps per trial (≈36–80 solves/s);
5-lane campaign ≈3,400 steps per 200 s (≈17 solves/s). Pair 1 solo earlier:
success at 49.0 s at ≈68 solves/s.

## Verdict
**CPU contention, not scene fidelity or a controller regression.** The sim
runs at realtime rate 1 regardless of load, while the C3+ planner solves as
fast as the CPU allows. At 5 concurrent trios on 16 cores the planner's
replanning rate drops ~4x, so per sim-second the controller gets a quarter of
the plan updates — enough to push every yaw-jittered pair past the cap.
Solo, the same dirs reproduce the benchmark's success profile (and pair 4
even needed 112.7 s — over the benchmark's 100 s cap, explaining its b-rep
"failure" there). Pair 3 is genuinely marginal (draw-level, 50% in the
benchmark), a solver-dynamics case rather than a load artifact.

## Recommendations for the real campaign
1. Do not run more than ~2 concurrent trios on this 16-core machine, or pin
   cores per lane; verify per-trial control-step rate ≥ the solo baseline
   (~35+ solves/s) before trusting an SR.
2. Alternatively decouple from wall time: run the sim at reduced realtime
   rate (or stepped lock-step) so planner solves per sim-second are
   load-invariant — then parallel lanes are safe.
3. Log solves/s in every trial manifest and treat it as a validity gate.
4. Cap: successes span 49–113 s here; 200 s is adequate margin for open_task
   once starvation is removed (obstacle scenes still need more).

Note: the loaded-tier numbers quoted above come from the erased 200 s
records (values preserved in PAIR_CAMPAIGN_SR_REPORT.md); the solo probe
records live under `runs/open_task/solo_pair0N/`.
