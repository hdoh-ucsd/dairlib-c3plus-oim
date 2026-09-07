# xArm6 OIM-fidelity port + rerun campaign — status report (2026-09-07)

## 1. What changed (user-directed, committed & pushed to standalone mainline)
Four reference-matched modifications against OIM (`oim/models/xarm6_pusht_tabletop/tee.xml`,
`oim/models/xarm6/xarm6.xml`), @ 4b7951cd4 (+ ba264f57e ycb, e55fc6a89/a366bbbe9 glyph arc):
1. **Table**: `ground_oim_xarm6.urdf` — 0.80 × 1.523 m box, **long axis Y** (was 5 × 0.91 long-X;
   its 0.455 m y-edge caused every C glyph "topple" = table-edge fall), white rgba 0.95.
2. **White table in renders**: renderer rebuilt (`scripts/c3ab/render_xarm6_fidelity_run.py`) —
   white table, OIM-style camera, per-frame pos/ang error overlay.
3. **No EE gap**: old port welded the tool at the *Panda* flange offset → 10.7 cm of air between
   link6 and the sphere. New `end_effector_xarm6_stick.urdf` is welded FLUSH at link6.
4. **Stick EE**: capsule-style stick r = 0.00555 m, tip at 0.1794 m (OIM-exact) replaces the
   r = 0.0195 sphere; planner LCS model updated (`end_effector_simple_model_xarm6.urdf`, radius
   read by GetEERadiusFromPlant); workspace z floor −0.01 → −0.024 in all 38 xarm6 configs
   (thin stick's tip-center must reach contact height; physical floor −0.0235).
Also: green transparent **goal ghost** in all renders (user request); YCB scene completed
(spam_can + mustard_bottle hull prisms from scenes.py:248-266 — the two objects whose absence
excluded ycb from the original benchmark).

## 2. Benchmark rerun — COMPLETE (20 runs, matched protocol: tol 0.05 m/0.1 rad, 100 sim-s cap)
| Scene | Goal reached | Under 100 s cap | Pre-fidelity |
|---|---|---|---|
| open_table (5 pairs × 2) | **10/10** (85–247 s, final ≤0.020 m) | 1/10 | 6/10 reached, 70.8 s median |
| single_obstacle (5 × 2) | 1/10 (202 s) | 0/10 | 1/10 |

**Headline: the fidelity fix converted open_table from 60 % ability to 100 % ability** — every
run now parks the T within 2 cm/0.1 rad; the OIM gap there is purely speed (OIM: 15–18 s).
single_obstacle is unchanged: its blocker is planner acquisition churn near the obstacle
(no-progress repositioning + sample-buffer overflow), not robot/scene geometry.

## 3. shelf_gap rerun — COMPLETE (10 runs, 900 s wall)
**3/10 latched** (119 s, 200 s, 274 s; the fast two final-pass at 0.011–0.019 m) vs
**0/10 pre-fidelity**. First shelf successes under the matched protocol family.

## 4. YCB clutter — first-ever C++ runs (port @ ba264f57e), 9/10 done
**2/9 latched**: trial 5 clean (197 s, final 0.004 m/0.093 rad), trial 4 latched 302 s then yaw
drifted. Failures stall at 0.46–0.53 m among the clutter. All four OIM obstacles verified
(cube, sugar box, spam_can, mustard_bottle at reference poses).

## 5. Glyphs under new fidelity — IN PROGRESS (I lane mostly done, C/R/A queued)
Early I-glyph results are **regressed** vs the old-fidelity campaign (0/8 vs 7/13 latched;
stalls at 0.3–0.49 m). Suspected: glyph task params (mesh-sampler standoff, sampling heights,
EE targets) were tuned around the old r = 0.0195 sphere; the 5.5 mm stick changes contact
geometry on the 54 mm-tall letters. C/R/A results pending; a param-conformance pass may be
needed — flagged, NOT tuned (no changes made mid-campaign).
Context — old-fidelity glyph verdicts (13 trials each): A 10/13, R 7/13, I 7/13,
C 1/13 with 11 table-edge falls (root-caused: goal 5.5 cm from the old narrow table's lip +
EE hook-drag in C's open mouth; forensics in icra_glyph_open_table_runs/analysis/).
The two interim C mitigations (goal inboard 3/13; QP object bound 0/6) were falsified as
fixes and reverted — the OIM table's ±0.76 m edges remove the hazard structurally.

## 6. Videos
Rolling render pass over EVERY run (success + failure): white table, stick EE, transparent
green goal ghost, obstacles included (oimframe SDFs). 29 rendered so far →
`results/fidelity_videos_2026-09-07/` (+ D mirror). Full sweep re-runs at campaign end.

## 7. Receipts & code
- Runs: results/fidelity_bench_2026-09-07/ (+bench_scores.txt), fidelity_campaign_2026-09-07/,
  fidelity_ycb_2026-09-07/, icra_glyph_open_table_runs/ (old-fidelity glyph arc + C forensics).
- Commits (standalone mainline): a366bbbe9 glyph campaign, e6de4dcb8 object bound,
  e55fc6a89 goal move, 4b7951cd4 fidelity port, ba264f57e ycb scene, plus the revert commit.
- D mirror synced for bench + glyph arcs; campaign/ycb sync at completion.

## 8. Open items
- Glyph campaign completion (C/R/A) + verdict on the I regression.
- YCB trial 10; final scoring table; full video sweep + D sync.
- If glyph regression confirms: param-conformance pass for letter tasks under stick EE
  (requires authorization — no tuning done).
