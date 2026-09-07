#!/usr/bin/env python3
"""Build canonical_failure_evidence/ (Task A) + final-benchmark validation (Task B)."""
import csv, glob, json, math, os, re, shutil, subprocess

R = "/root/push_anything_ADMM/results"
EV = os.path.join(R, "canonical_failure_evidence")
WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant"
STG = os.path.join(WT, "results", "cleanup_staging")
os.makedirs(EV, exist_ok=True)
os.makedirs(STG, exist_ok=True)

def mk(name):
    d = os.path.join(EV, name); os.makedirs(d, exist_ok=True); return d

def cp(src, dstdir, newname=None):
    dst = os.path.join(dstdir, newname or os.path.basename(src))
    shutil.copy2(src, dst); return dst

def downsample_1hz(src, dst):
    last = -1.0; n = 0
    with open(src) as f, open(dst, "w") as g:
        for line in f:
            line = line.strip()
            if not line: continue
            try: t = json.loads(line).get("t")
            except json.JSONDecodeError: continue
            if t is None: continue
            if t - last >= 1.0 - 1e-9:
                g.write(line + "\n"); last = t; n += 1
    return n

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

# ---------------- Task A ----------------
# 1 contact_acquisition
d = mk("contact_acquisition")
src = f"{R}/c3plus_p5_fix_validation/shelf_gap_fix/plain/draw1"
cp(f"{src}/state_trace.jsonl", d)
cp(f"{WT}/results/parallel_c3plus_audit/agent_b_xarm6_plant/contact_acquisition.csv", d)
open(os.path.join(d, "README.md"), "w").write("""# Contact acquisition failure (canonical)

**Class:** reposition churn / low contact-to-push conversion (shared Franka-plant class).
**Source run:** `results/c3plus_p5_fix_validation/shelf_gap_fix/plain/draw1` (shelf_gap, post-P5-fix stack, 2026-09-05).

Headline numbers:
- 66 completed repositions ("reached repositioning target"; 60 "Repositioning after not making progress" triggers) over 194 s sim; run FAILS (final pos_err 0.291 m, ang_err 0.356 rad, first_success_t null).
- Cross-run measurement (contact_acquisition.csv, agent_b_xarm6_plant audit): this population shows episode conversion ~14% (productive-window fractions 0.017-0.032 vs contact fractions up to 0.475) - repositions land, contacts form, pushes do not convert to goal progress.
- Sister draw0 of the same campaign succeeded (0.018 m @ 139.8 s), isolating acquisition/conversion, not the stack, as the failure axis.

Evidence: `state_trace.jsonl` (full, 10 Hz, 465 KB), `contact_acquisition.csv` (per-draw conversion metrics).
""")

# 2 successful_shelf_traversal
d = mk("successful_shelf_traversal")
src = f"{R}/c3plus_p5_fix_validation/shelf_gap_fix/plain/draw0"
cp(f"{src}/state_trace.jsonl", d)
cp(f"{src}/success.log", d)
open(os.path.join(d, "README.md"), "w").write("""# Successful shelf traversal (canonical success case)

**Source run:** `results/c3plus_p5_fix_validation/shelf_gap_fix/plain/draw0` (shelf_gap, P5-fix validation campaign, 2026-09-05).

Headline numbers:
- First-ever tight shelf_gap success under the fixed stack: SUCCESS t=139.8 s, final pos_err 0.0180 m, ang_err 0.0632 rad (success.log FINAL line).
- Traversal includes the 22.9 mm/s corridor burst through the gap.
- P5 fixes in play: default reposition timeout + swept-path veto + escape zone + placeholder exclusion (commits 5fbec41ef + 43d5efb96).

Evidence: `state_trace.jsonl` (full, 10 Hz), `success.log`.
""")

# 3 local_fixed_point + push_plateau
fc = list(csv.DictReader(open(f"{R}/final_oim_c3plus_comparison/metrics/failure_classification.csv")))
def trial_dir(scene, pair, rep):
    if rep == "a":
        p = f"{R}/final_oim_c3plus_comparison_runs/c3plus_matched/{scene}/trial{pair}"
        if os.path.isdir(p): return p
        p = f"{R}/final_oim_c3plus_comparison_runs/c3plus_matched_b/{scene}/t{pair}_a"
        return p
    return f"{R}/final_oim_c3plus_comparison_runs/c3plus_matched_b/{scene}/t{pair}_{rep}"

f4 = next(r for r in fc if r["method"] == "c3plus" and r["class"] == "F4")
f1 = next(r for r in fc if r["method"] == "c3plus" and r["class"] == "F1")
for name, row, title in [("local_fixed_point", f4, "Local fixed point (F4)"),
                         ("push_plateau", f1, "Push plateau / contact-acquisition churn (F1)")]:
    d = mk(name)
    td = trial_dir(row["scene"], row["pair"], row["rep"])
    cp(f"{td}/state_trace.jsonl", d)
    open(os.path.join(d, "README.md"), "w").write(f"""# {title} - canonical benchmark failure

**Class {row['class']}** per `final_oim_c3plus_comparison/metrics/failure_classification.csv`.
**Source run:** `{td.replace(R + '/', 'results/')}` (final matched C3+ vs OIM benchmark, protocol: 100 s cap, tol 0.05 m / 0.1 rad).

Classifier evidence: {row['evidence']}
Scene={row['scene']} pair={row['pair']} rep={row['rep']} method=c3plus.

Evidence: `state_trace.jsonl` (full trial trace, 10 Hz).
""")

# 4 topple
d = mk("topple")
cp(f"{R}/panda_to_xarm6_port_runs_r2/xarm6_trial3/state_trace.jsonl", d)
open(os.path.join(d, "README.md"), "w").write("""# Topple - wedged-lift object topple (6-DOF xArm6 port)

**Source run:** `results/panda_to_xarm6_port_runs_r2/xarm6_trial3` (panda-to-xarm6 port arc, round 2).

The 6-DOF port's EE wedges under the object edge and lifts, toppling the T off its
resting plane - the canonical topple failure of the port arc.
Video: `P6_r2_t3` in `figures/trajectories/port_arc_videos/`.

Evidence: `state_trace.jsonl` (full, ~900 KB).
""")

# 5 armature_chatter
d = mk("armature_chatter")
cp(f"{R}/panda_to_xarm6_final_probe4/effort_trace.jsonl", d)
open(os.path.join(d, "README.md"), "w").write("""# Armature chatter - 500 Hz period-2 effort oscillation

**Source run:** `results/panda_to_xarm6_final_probe4` (xArm6 final-probe series).

Headline numbers:
- Effective rotor inertia mismatch: I_eff ~= 0.027 (xArm6 armature) vs 1.0 (the Panda-tuned
  assumption baked into the executor gains) - a ~37x under-damped plant.
- Result: period-2 chatter at the 500 Hz control rate - alternating-sign effort commands
  visible in effort_trace.jsonl (the effort_trace is sampled through the probe window).

Evidence: `effort_trace.jsonl` (full, ~730 KB; fields t/q/v/effort at 5-joint layout).
""")

# 6 terminal_hold
d = mk("terminal_hold")
lines6 = [l for l in open(f"{R}/panda_to_xarm6_final_probe6/osc.log", errors="replace") if "XARM6_5J" in l]
open(os.path.join(d, "probe6_osc_xarm6_5j_excerpt.log"), "w").writelines(lines6)
lines7 = [l for l in open(f"{R}/panda_to_xarm6_final_probe7/osc.log", errors="replace") if "XARM6_5J" in l]
open(os.path.join(d, "probe7_osc_convergence_excerpt.log"), "w").writelines(lines7)
open(os.path.join(d, "README.md"), "w").write("""# Terminal hold - proportional capture (v_des = -Kc*err)

**Source runs:** `results/panda_to_xarm6_final_probe6` (capture behavior) and
`results/panda_to_xarm6_final_probe7` (convergence).

- probe6 excerpt: XARM6_5J executor lines around capture; v_des is exactly the proportional
  law -Kc*err (Kc=4.0, alpha=2.0, lambda=0.05, kv=[300,300,200,200,200], efforts=[50,50,32,32,32],
  qdot_limit=0.5) - the terminal hold is a pure P-capture, no feedforward.
- probe7 excerpt: converged terminal hold - p_tip settles ~4 mm from p_des in xy with a
  frozen v_des and |qdot_cmd| ~0.029 (t=61..65 identical to 4 decimals), i.e. a static
  equilibrium of the P-law against the effort caps rather than true zero-error convergence.

Evidence: `probe6_osc_xarm6_5j_excerpt.log`, `probe7_osc_convergence_excerpt.log`.
""")

# 7 workspace_abort
d = mk("workspace_abort")
src = f"{R}/panda_to_xarm6_port_runs/xarm6_trial1"
tail = subprocess.run(["tail", "-n", "40", f"{src}/planner.log"], capture_output=True, text=True).stdout
open(os.path.join(d, "planner_log_tail.log"), "w").write(tail)
n = downsample_1hz(f"{src}/state_trace.jsonl", os.path.join(d, "state_trace_1hz.jsonl"))
open(os.path.join(d, "README.md"), "w").write(f"""# Workspace abort - EE overshoot past the workspace ellipsoid

**Source run:** `results/panda_to_xarm6_port_runs/xarm6_trial1` (panda-to-xarm6 port, round 1).

Headline: the EE overshoots to r = 0.81 m; the planner dies on
`DRAKE_DEMAND` in `CheckForWorkspaceLimitViolations()`
(sampling_based_c3_controller.cc:3186, workspace half-space check) - see planner_log_tail.log:
the final lines show reposition/c3 switching followed by the hard abort.

Evidence: `planner_log_tail.log` (last 40 lines incl. the abort), `state_trace_1hz.jsonl`
(downsampled to 1 Hz, {n} samples).
""")

# 8 reposition_deadlock
d = mk("reposition_deadlock")
src = f"{R}/obstacle_pushing_comparison_2026-09-06/shelf_gap/1_baseline_no_route_guidance/draw0"
cp(f"{src}/metrics.json", d)
tail = subprocess.run(["tail", "-n", "60", f"{src}/planner.log"], capture_output=True, text=True).stdout
open(os.path.join(d, "planner_log_tail.log"), "w").write(tail)
open(os.path.join(d, "README.md"), "w").write("""# Reposition deadlock (P5, pre-fix) - the shelf video failure

**Source run:** `results/obstacle_pushing_comparison_2026-09-06/shelf_gap/1_baseline_no_route_guidance/draw0`
- per `C3PLUS_POST_OUTER_LOOP_FIX_GAP_AND_STALL_REPORT.md` this exact draw is the shelf
  video's CASE P5 reposition/contact-acquisition deadlock (the report pins the deadlock to
  1_baseline draw0, not the 4_route_guidance arm; canonical choice follows the report).

Mechanism: PWL reposition lift 0.06 m < shelf top 0.111 m, planner pusher-obstacle blind,
no default reposition timeout -> perpetual reposition churn. metrics.json: 206.2 s,
final_xy 0.3645 m, success false, 0 around-and-past route candidates selected.

**Fixed by** P5 commits `5fbec41ef` + `43d5efb96` (default timeout + swept-path veto +
escape zone + placeholder exclusion); post-fix validation = `c3plus_p5_fix_validation/shelf_gap_fix`
(see `canonical_failure_evidence/successful_shelf_traversal/`).

Evidence: `metrics.json`, `planner_log_tail.log`.
""")

# 9 executor_descent_stall
d = mk("executor_descent_stall")
src = f"{R}/panda_to_xarm6_port_final_r1/xarm6_trial1"
n = downsample_1hz(f"{src}/state_trace.jsonl", os.path.join(d, "state_trace_1hz.jsonl"))
open(os.path.join(d, "README.md"), "w").write(f"""# Executor descent stall - per-joint clamp starves vertical motion

**Source run:** `results/panda_to_xarm6_port_final_r1/xarm6_trial1` (xArm6 port final round 1).

The executor's per-joint velocity clamp scales each joint independently; on descent the
clamp truncates the joints carrying the -z task direction, so the tip's vertical approach
stalls above the contact height and pushes never engage.

Evidence: `state_trace_1hz.jsonl` (downsampled to 1 Hz, {n} samples).
""")

print("Task A dirs done.")

# ---------------- Task B ----------------
CAP, POS_TOL, ANG_TOL = 100.0, 0.05, 0.1

def score_trace_jsonl(path):
    ts, pos, ang = [], [], []
    for line in open(path):
        line = line.strip()
        if not line: continue
        try: r = json.loads(line)
        except json.JSONDecodeError: continue
        t = r.get("t")
        if t is None or t > CAP: break
        if len(r.get("obj", [])) < 7: continue
        ts.append(t); pos.append(r["pos_err"]); ang.append(abs(wrap(r["ang_err"])))
    if not ts: return None
    succ_t = next((t for t, p, a in zip(ts, pos, ang) if p < POS_TOL and a < ANG_TOL), None)
    return dict(success=int(succ_t is not None), t_success=succ_t,
                final_pos=pos[-1], final_yaw=ang[-1])

def score_oim(tdir):
    txt = open(os.path.join(tdir, "run.log"), errors="replace").read()
    m = re.search(r"saved run to (\S+\.json)", txt)
    if not m: return None, "no run JSON in run.log"
    jp = m.group(1)
    if not os.path.isfile(jp): return None, "missing JSON " + jp
    dta = json.load(open(jp))
    goal = dta["static"]["goal"]
    ts, pos, ang = [], [], []
    for t, p in zip(dta["dynamic"]["time"], dta["dynamic"]["object_pose"]):
        if t > CAP: break
        ts.append(t)
        pos.append(math.hypot(p[0] - goal[0], p[1] - goal[1]))
        ang.append(abs(wrap(p[2] - goal[2])))
    if not ts: return None, "no samples"
    succ_t = next((t for t, p, a in zip(ts, pos, ang) if p < POS_TOL and a < ANG_TOL), None)
    return dict(success=int(succ_t is not None), t_success=succ_t,
                final_pos=pos[-1], final_yaw=ang[-1], run_json=jp), None

stored = {}
for r in csv.DictReader(open(f"{R}/final_oim_c3plus_comparison/metrics/trial_metrics.csv")):
    stored[(r["method"], r["scene"], r["pair"], r["rep"])] = r

rows_out, sizes, mismatches = [], [], 0
def compare(key, rec, tdir, err=None):
    global mismatches
    s = stored.get(key)
    trial = "/".join(key)
    if s is None:
        rows_out.append([trial, "ALL", "", "", "NOT_IN_STORED_CSV"]); return
    if rec is None:
        rows_out.append([trial, "ALL", "", "", "RECOMPUTE_FAILED:" + str(err)]); return
    for metric, fmt in [("success", "%d"), ("t_success", "%.1f"),
                        ("final_pos", "%.4f"), ("final_yaw", "%.4f")]:
        sv = s[metric]
        rv = rec[metric]
        rstr = "" if rv is None else (fmt % rv)
        ok = (sv == rstr) or (sv == "" and rv is None)
        if not ok:
            try: ok = abs(float(sv) - float(rv)) < (0.06 if metric == "t_success" else 5e-4)
            except (TypeError, ValueError): ok = False
        if not ok: mismatches += 1
        rows_out.append([trial, metric, sv, rstr, "MATCH" if ok else "MISMATCH"])

# C3+ trials
for base, repdefault in [("c3plus_matched", "a"), ("c3plus_matched_b", None)]:
    for tdir in sorted(glob.glob(f"{R}/final_oim_c3plus_comparison_runs/{base}/*/*")):
        if not os.path.isdir(tdir): continue
        scene = os.path.basename(os.path.dirname(tdir))
        b = os.path.basename(tdir)
        m1 = re.match(r"trial(\d+)$", b); m2 = re.match(r"t(\d+)_([a-z])$", b)
        if m1: pair, rep = m1.group(1), "a"
        elif m2: pair, rep = m2.group(1), m2.group(2)
        else: continue
        tr = os.path.join(tdir, "state_trace.jsonl")
        if os.path.isfile(tr): sizes.append((tr, os.path.getsize(tr)))
        rec = score_trace_jsonl(tr) if os.path.isfile(tr) else None
        compare(("c3plus", scene, pair, rep), rec, tdir, "no trace")

# OIM trials
for base in ["oim_matched", "oim_matched_b"]:
    for tdir in sorted(glob.glob(f"{R}/final_oim_c3plus_comparison_runs/{base}/*/*")):
        if not os.path.isdir(tdir): continue
        scene = os.path.basename(os.path.dirname(tdir))
        b = os.path.basename(tdir)
        m1 = re.match(r"trial(\d+)$", b); m2 = re.match(r"t(\d+)_([a-z])$", b)
        if m1: pair, rep = m1.group(1), "a"
        elif m2: pair, rep = m2.group(1), m2.group(2)
        else: continue
        rec, err = score_oim(tdir)
        compare(("oim", scene, pair, rep), rec, tdir, err)

with open(os.path.join(STG, "final_benchmark_validation.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["trial", "metric", "stored", "recomputed", "match"])
    w.writerows(rows_out)

maxsz = max(s for _, s in sizes)
print(f"C3+ traces: {len(sizes)}, max {maxsz/1e6:.2f} MB, total {sum(s for _,s in sizes)/1e6:.2f} MB")
print("NO COMPACTION NEEDED" if maxsz < 5e6 else "COMPACTION NEEDED")
nm = sum(1 for r in rows_out if r[4] == "MATCH")
print(f"validation rows: {len(rows_out)}, MATCH {nm}, non-match {len(rows_out)-nm}")
for r in rows_out:
    if r[4] != "MATCH": print("  ", r)
