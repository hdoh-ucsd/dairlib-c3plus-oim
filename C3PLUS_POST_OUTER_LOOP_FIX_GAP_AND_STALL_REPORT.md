# C3+ Post-Outer-Loop-Fix Audit: Remaining Implementation Gap and Stall Forensics

Date: 2026-09-04. Branch `c3plus-channel-route-v1` @ `fa8af6c41` (standalone repo `hdoh-ucsd/dairlib-c3plus-oim`).
Evidence: run data under `results/obstacle_pushing_comparison_2026-09-06/`, MJX source trace of
`external/Object-Informed-Manipulation-MJX/oim/algs/c3_dynamic.py`, forensics scripts in the job tmp dir
(`shelf_stall_forensics.py`, `shelf_z_check.py`, `so_stall_check.py`).

## Primary verdict

**Verdict F (multiple causes), with a dominant single mechanism per video:**

- `shelf_gap_baseline_no_route_guidance.mp4` (shelf_gap/1_baseline/draw0) — **CASE P5: reposition /
  contact-acquisition deadlock**, produced by a **pusher-obstacle model gap (P1-mechanism)** plus the
  **absence of any reposition timeout in the default outer loop**.
- `single_obstacle` baseline video (single_obstacle/1_baseline/draw2) — **CASE P3: genuine C3+
  local-progress fixed point** (the same class MJX exhibits), *not* a port bug.

**Required causal chain (shelf video):**
the selected reposition target (0.391, −0.073) lies on the far side of the front shelf from the pusher →
the PWL reposition trajectory's lateral leg runs at lift height z = 0.06 m, **below the shelf top at
z = 0.111 m**, and the straight-line leg provably crosses the shelf volume → the shelf exists only in the
simulator (the planner LCS, candidate ranking, reposition trajectory generator, and arrival check are all
pusher-obstacle blind; the swept-feasibility check is env-gated log-only) → the simulated EE is physically
blocked at z ≈ 0.062, parked 2.3 mm from the shelf face, so the 5 mm arrival radius **never fires** →
repos→repos stickiness keeps re-selecting the same frozen target (10,739 of 10,919 stall cycles) and the
baseline arm has **no reposition timeout** (`transaction_v1_1` env-gated off) → the controller spends 98 %
of the final 165 s in reposition mode and the object never moves again (final error 0.297 m).

## 1. Forensic numbers — shelf video (shelf_gap/1_baseline/draw0)

| Signal | Value | Meaning |
|---|---|---|
| Pusher–shelf gap, whole run | min **+0.0023 m** (t=81 s); 0/13,341 cycles inside inflated shelf | The pusher **never penetrates or rests on** the shelf. It is *pressed against* it by reposition commands. |
| Mode during stall | C3 fraction **0.02** (98 % reposition) | The stall is a reposition stall, not a pushing stall. |
| Selected candidate during stall | id 1 (current repos target) **10,739 / 10,919** cycles | Target frozen at (0.391, −0.073, 0.005) — min = max over the entire stall. |
| Actual EE height during stall | median **0.062 m**, max 0.063 m | EE pinned at the lateral-leg altitude (lift 0.06) — the descend phase never starts. |
| Lateral leg at z=0.06 vs shelf | **crosses shelf volume**; shelf top z = **0.111 m** | Geometric proof the commanded path runs through the physical shelf. |
| Object–obstacle LCS contact | active rows healthy, max λ = **0.647** | The *object*-obstacle contact model (this branch's lcs_contact) worked; it is not the culprit. |

Answer to "is the pusher physically stuck on the obstacle?": **no** — it is stuck *against* it
(2–4 mm standoff), continuously commanded into a wall it cannot pass. The outer loop did not "give up";
it is structurally trapped: arrival can never fire, cost-based early return loses to repos-stickiness with
the target's stale/optimistic cost, and no timeout exists to break the cycle.

## 2. Forensic numbers — single_obstacle video (draw2)

Second half of the run: C3 (pushing) mode **67 %**, selected candidates rotate (3/1/2), pusher-obstacle
gap min **+0.022 m** (median +0.049 — well clear), object stalled at (0.379, −0.122), 0.191 m from goal.
This is the classic C3+ plateau: pushing bursts that no longer produce net progress, fresh candidates all
near break-even (documented in the fidelity report: only ~2 % of push windows move the object > 5 mm).
No obstacle interaction, no reposition deadlock. **CASE P3.**

## 3. Outer-loop comparison: MJX vs C++ port (pre/post-fix)

| Aspect | MJX (`c3_dynamic.py`) | C++ post-fix @ fa8af6c41 | Gap? |
|---|---|---|---|
| Candidate costs | **Fresh vmap solve of all 10 candidates every tick**; no cost persisted across ticks | Fresh C3 per candidate per tick (independent-batch conformant); **buffer candidate carries a stale cost** | stale-buffer cost remains a C++-only divergence |
| Candidate generation | Deterministic dense boundary mesh (8/edge), analytic score, top-k=8; unsuccessful-buffer masking (radius 0.03, FIFO 8) | Random perimeter/mesh sampling per lineage | different but both cover the boundary |
| Stall detector | window=16 ticks, drop=0.5, hist reset to 1e12 on mode flip/goal/crossing | window=180 loops, drop=0.01, C3-loops-only (post-fix) | post-fix matches dairlib semantics; MJX's own constants differ from both |
| Hysteresis | 0.6/0.7 (c3→repos), 0.9/0.5 (repos→c3), 0.7/0.7 (repos→repos), position-phase switch at 0.05 m | single relative 0.9 (repos→c3), repos-stickiness on selection | partial |
| Reposition execution | **one-step planar velocity command** on a safety ring (r_safe = bounding + 0.02 + 0.02); planar 2-DOF pusher, **no lift/lower**; orbit/retreat logic each tick | precomputed **3-phase PWL trajectory** (lift 0.06 → lateral → descend), tracked to completion | **critical port-specific artifact**: the C++ lateral leg has a fixed altitude that can sit below obstacle tops |
| Reposition exit | arrival (0.02 m) **or** cost disjunct `curr < 0.1·best_other` — can exit *before* arrival, re-evaluated **every tick** | arrival (5 mm) or relative-cost hysteresis; in the baseline arm effectively arrival-gated, target frozen by stickiness | **MJX cannot deadlock the same way**: exit conditions re-fire each tick and the target competes against fresh costs |
| Reposition timeout | none — but not needed (see above) | none by default; `transaction_v1_1` adds one, **env-gated off** | timeout fix exists but is not on in baseline |
| Pusher vs obstacles (planner) | **blind** (obstacle terms are object-only; ring has no obstacle term) | **blind** (same) | shared limitation, *not* a port gap |
| Pusher vs obstacles (sim) | MuJoCo collides pusher–obstacle (contype 1/1) | Drake collides pusher–shelf | both sims block what the planner ignores |

So the P1 "planner blind to pusher-obstacle" gap is **inherited from MJX by design**, but MJX's planar
one-step reposition with per-tick exit re-evaluation makes it self-recovering, while the C++ port's
latched PWL trajectory + frozen target + no timeout turns the same blindness into an absorbing state.
The outer-loop *progress logic* itself (post-fix) is conformant; the residual gaps are (i) the stale
buffer-candidate cost, (ii) reposition executor semantics (PWL altitude + arrival-only exit + no timeout).

## 4. Classification of the 6 candidate cases

- **P1** (missing pusher-obstacle model): mechanism present in both stacks; *contributing* cause for shelf.
- **P2** (candidate search collapse): **excluded** — 3+ candidates evaluated every cycle throughout.
- **P3** (genuine C3+ fixed point): **primary for the single_obstacle video**; matches MJX-class behavior.
- **P4** (cost/ranking corruption): **excluded** — object-obstacle λ healthy, ranking rotates when in C3.
- **P5** (reposition deadlock): **primary for the shelf video**.
- **P6** (command pipeline freeze): **excluded** — EE commands active (pressed into shelf, bbox motion).

## 5. Recommended fixes (not yet landed; would be off-reference-of-MJX in mechanism but restore its *intent*)

1. Enable a default reposition timeout (promote `transaction_v1_1`'s `repos_timeout_loops` out of env-gating).
2. Make the swept-feasibility check (already implemented, log-only) *veto* reposition targets whose PWL
   path intersects known obstacle AABBs, or raise lift height above obstacle tops.
3. Re-evaluate the reposition-exit cost disjunct every loop against **fresh** target cost (kills the
   stale-buffer stickiness), matching MJX's per-tick fresh-solve semantics.

## 6. Provenance

| Artifact | Commit / location |
|---|---|
| Post-fix runs (all videos) | `fa8af6c41`, branch `c3plus-channel-route-v1` |
| Outer-loop fix chain | e17cf46ea → 78195e85f → 5acb731e3 → e9426f61f → fa8af6c41 |
| MJX reference | `external/Object-Informed-Manipulation-MJX`, `oim/algs/c3_dynamic.py:865-994` |
| shelf video run | `results/obstacle_pushing_comparison_2026-09-06/shelf_gap/1_baseline_no_route_guidance/draw0` |
| single_obstacle video run | `.../single_obstacle/1_baseline_no_route_guidance/draw2` |

Caveats: single draws per condition — no repeatability claim; §11 deterministic state-restore replay
remains unbuilt (documented limitation), so pre-fix vs post-fix comparisons are behavioral, not bitwise.

> **Results-cleanup note (2026-09-07):** the raw run directories cited in this report were removed in the results canonicalization (superseded/pre-fidelity data). The conclusions stand on this report itself; canonical replacement evidence lives in `results/canonical_failure_evidence/` and the final dataset in `results/final_oim_c3plus_comparison/`.
