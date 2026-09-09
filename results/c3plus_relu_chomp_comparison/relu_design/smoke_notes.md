# ReLU smoke gate (CHECKPOINT C) — PASS
4 runs (single_obstacle s01/s03, shelf_gap s01, slalom s01), 300 s cap, 2 lanes.
- [OBS-RANK] relu_footprint eps=0.01 w=200 banner present in all 4 planner logs
  (variant active; env-gating works).
- No NaN in any planner log; no runaway costs; sims/OSC/planner all ran
  (steps flowing, 1.3k–13.7k control steps).
- Ranking change confirmed offline (Checkpoint B: 58%/31% top-1 changes where
  geometry-hugging; flagship stall pose flips top-1). Physical nonpenetration
  machinery untouched (lcs_contact unchanged); outer loop untouched.
- No smoke run succeeded within 300 s — expected; smoke is not an SR gate.
Baseline path remains compile-identical when the env vars are absent.

NOTE 2026-09-09: the four 300 s smoke run dirs were MOVED to
relu_design/smoke_runs_300s/ (they are Checkpoint-C artifacts, not campaign
cells — the live relu grid reruns those cells at the canonical 600 s cap).
