# C3+ Stepwise Repeated-Miss and Run-Failure Report (Agent A)

Base commit `0abc0eb7c` (controller code identical to `43d5efb96`), branch `c3plus-channel-route-v1`;
audit branch `audit/stepwise-repeated-miss` (worktree `.claude/worktrees/audit-repeated-miss`).
Evidence: `results/parallel_c3plus_audit/agent_a_stepwise/` (synchronized index, attempt table,
similarity CSV, decision/reposition traces, forced-contact replays, failure cards, handoff JSON).
No behavioral controller changes were made; all analysis is passive, over existing run logs plus a
standalone pydrake replay rig.

## 1. Video-to-run mapping

`shelf_gap_p5fix_tight_success.mp4` = `results/c3plus_p5_fix_validation/shelf_gap_fix/plain/draw0`
(verified by final metrics 0.0181 m / 0.063 rad @ t=142.8 s and obstacle overlay geometry; the run
SUCCEEDS at the end — the repeated misses are its opening phase). Full config in
`video_run_mapping.yaml`. Four current plain single_obstacle runs mapped (3 P5-validation draws +
serial V0).

## 2. The measured repeated attempts (question 1)

Segmenting by controller events (not video): **the repetition is 4 transactions** (attempts 2-5,
t=5.2-24.9 s), matching the video's "approximately four". Each transaction:

| Attempt | t (s) | Sector (obj frame) | Min pusher-object gap | Object dxy | End reason |
|---|---|---|---|---|---|
| 2 | 5.2-9.2 | -64.5 deg | 1.2 mm (brief graze) | 23 mm | kToReposUnproductive @ 180 cycles |
| 3 | 10.9-14.3 | -63.5 deg | **9.5 mm — never touched** | 0.0 mm | kToReposUnproductive @ 180 cycles |
| 4 | 16.1-19.8 | -65.4 deg | **14.1 mm — never touched** | 0.0 mm | kToReposUnproductive @ 180 cycles |
| 5 | 21.5-24.9 | -63.1 deg | **10.4 mm — never touched** | 0.0 mm | kToReposUnproductive @ 180 cycles |

Attempt 6 (t=26.6 s) draws a different sector (+24 deg), acquires contact (gap -18 mm), moves the
object 80 mm — the cycle breaks and the run eventually succeeds.

Anatomy of one transaction (identical across 2-5): reposition arrives at the commanded point with
**exactly ~27.0 mm pusher-object surface gap** (0.02694-0.02700 m at every arrival — this is the
candidate geometry's built-in standoff; arrival tracking itself is precise), mode flips to C3 via
`kToC3ReachedReposTarget`, the push phase then executes a ~241 mm sweeping EE motion whose closest
approach to the T is 9.5-14.1 mm — it passes the object without touching — and after exactly 180
cycles (the no-progress window) `kToReposUnproductive` fires and the loop re-selects the same
sector.

## 3. Are the control sequences identical? (question 2, solver layer)

No. From the per-knot QP logs (`input_sequence_similarity_shelf_draw0.csv`): between attempts 2-5,
dU_inf = 0.19-0.77 N, dx0_inf ~ 0.19, sub-goals differ by up to 0.15 — **BEHAVIORALLY_SIMILAR,
not numerically identical**. Rules out R1 (same problem re-solved) and R2 (stale/cached solution):
each attempt is a fresh solve of a freshly re-created, slightly different problem that produces the
same physical outcome. Candidate IDs are stable (selected_id=2) but map to freshly sampled contacts
in the same sector, not one cached candidate.

## 4. First layer of repetition (question 2 answer)

- **Created at: contact acquisition / push execution.** The transaction fails at the standoff-to-
  contact conversion: arrival is declared by an EE-position check with no contact requirement
  (27 mm gap), and the subsequent C3 push sweep misses by ~10 mm. Deterministic forced-contact
  replay (variant C, 3 reps, flocked) at the attempt-3 state PROVES the sector is not dead: a
  pusher placed directly at the same sector moves the object 29 mm and rotates it -1.02 rad.
  Per the section-11 interpretation table: **C succeeds -> contact acquisition caused the miss.**
- **Persisted by: outer-loop memory.** `AddToUnsuccessfulBuffer` is called on the Xbox-force,
  v1.1-timeout, and kToC3Cost paths but **NOT on kToReposUnproductive** (cc:2647-2652) — an
  unproductive push leaves no failure memory; with travel cost 0 and a near-static landscape the
  argmin re-prefers the same sector. Verified at HEAD (not copied from historical audits). Also
  re-verified at HEAD: buffer stale scalar-cost reuse still exists in `current` mode
  (skipped only in transaction modes); current contact is index-0, excluded from argmin.
- Not the C3 solver (solutions healthy and distinct), not candidate generation per se (the sampler
  eventually draws the breaking sector), not command/state sync (EE tracks its plan; the plan
  itself clears the object).

**Repeated-attempt verdict: MULTIPLE_CAUSES_RANKED**
1. R5_CONTACT_ACQUISITION_FAILURE (creation — dominant)
2. R4_OUTER_LOOP_MEMORY_OR_TRANSITION_ERROR (persistence — enables the x4 repetition)
3. R6_FORCE_TRANSFER_FAILURE (minor: attempt 2's 1.2 mm graze moved 23 mm but unproductively)

## 5. Why each plain single_obstacle run fails (question 3)

Four runs, four DISTINCT classes (`single_obstacle_failure_cards.json`):
- draw0: **S2_LOCAL_FIXED_POINT** — parks north of the obstacle at 0.40 m for 185 s; at the plateau
  cycle all three alternate candidates are hard-filtered to 1e12 and the one productive
  circumnavigation contact (stem-tip east) is never drawn.
- draw1: **S4_WORKSPACE_ABORT** — 33-deg tilt transient at t~14 s sends the object +x; the chase
  crosses the outer radius (last EE radius 0.748 m); hard abort at t=49.5 s.
- draw2: **S5_TOPPLE** — converging (0.60 -> 0.21 m) when the T tips 123 deg during a push;
  deterministic replays show crossbar-underside contacts topple the T; no tilt guard exists.
- serial V0: **S7_PUSH_PRODUCTION_PLATEAU** (near miss 0.056 m; ended by the harness wall cap).
Earliest divergence vs the open_table success comparator: the successful run's early misses were
broken by a productive draw within ~4 transactions; draw0's pool-emptying near the obstacle has no
such escape (hard filters + never-drawn circumnavigation contact).

## 6. Numerical causal chain (required)

x0 with EE at (0.574, 0.302, z 0.017), object (0.492, 0.278, yaw -0.38), sub-goal 0.15 m south
-> ranking selects the -63deg-sector candidate (fresh solve, J_rank ~ static landscape, travel
cost 0) -> reposition arrives 27.0 mm short of the surface and `kToC3ReachedReposTarget` fires on
EE position alone -> the executed 241 mm push sweep clears the T by 9.5-14.1 mm (zero contact,
u_inf-distinct from the previous attempt by 0.19-0.77 N) -> actual object response 0.0 mm ->
after exactly 180 cycles the no-progress trigger fires kToReposUnproductive, which records
NOTHING in the unsuccessful buffer -> the unchanged cost landscape re-selects the same sector ->
the transaction repeats (4x) until the random sampler draws the +24 deg sector whose sweep
happens to intersect the object.

No behavioral fix is implemented in this branch (investigation only).
