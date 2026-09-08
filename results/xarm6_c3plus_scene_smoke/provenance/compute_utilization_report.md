# Compute utilization report — xArm6 C3+ scene-sync + metric smoke (2026-09-07)

- **CPU count:** 16 logical cores (WSL2), 23 GB RAM. Machine idle at start (load 0.08).
- **Maximum concurrent static-analysis jobs:** 7 subagents (six scene audits +
  one metric source trace) ran fully concurrently in phase 1, while the main
  session concurrently built binaries and authored the harness.
- **Maximum concurrent simulations:** 5 isolated trios (sim + OSC + C3+ planner
  + recorder = 20 processes) on unique LCM udpm ports 7811–7815 with per-run
  output/tmp dirs and setsid process groups. The open_task metric-gate run ran
  alone first, per protocol.
- **Concurrent packaging:** 4 postprocess+3D-render jobs in parallel.
- **Mean CPU utilization:** 1-min load averaged **11.0 / 16 cores (~69%)**
  over the working window, peaking at **25.3** during the parallel-sim +
  bazel-build overlap (sampler: `provenance/cpu_samples.log`, 30 s cadence).
- **Total wall time:** ~1 h 25 m from job start (19:12) to final packaging
  (~20:40), including one unplanned full Drake rebuild (17 min at --jobs=12)
  after the prebuilt binary set was invalidated.
- **Estimated serial wall time:** 6 audits (~4 min each) + metric trace (4 min)
  + 6 sims (200 s each) + 6 packagings (~2 min each) + build ≈ 1 h 40 m of
  task time serialized ≈ **2 h 45 m+**; achieved ≈ 1 h 25 m →
  **speedup ≈ 2x wall-clock**, with the residual dominated by the two serial
  gates the protocol itself mandates (open_task before the fleet; the icra
  binary rebuild).
- Simulations are realtime-rate-1 bound (200 s floor each), so the parallel
  5-scene phase saved ~13 minutes versus serial; the audit/packaging phases
  saved ~30 minutes.
