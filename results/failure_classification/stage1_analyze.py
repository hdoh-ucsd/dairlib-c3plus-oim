#!/usr/bin/env python3
"""Stage-1 failure classification for the 30-run xArm6 C3+ campaign. READ-ONLY on run data."""
import json, math, os, re
import csv
import numpy as np
from multiprocessing import Pool

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics"
RUNS = os.path.join(WT, "results/xarm6_c3plus_scene_smoke/runs")
OUT = os.path.join(WT, "results/failure_classification")
SCENES = ["open_task", "single_obstacle", "shelf_gap", "ycb_clutter", "icra_sign", "slalom"]
POS_TOL, ANG_TOL = 0.05, 0.10
DWELL_S = 5.0
CRASHED = {("single_obstacle", "04"), ("single_obstacle", "05")}

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def quat_yaw(q):
    w, x, y, z = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def load_trace(d):
    t, pe, ae, xs, ys, yaws = [], [], [], [], [], []
    with open(os.path.join(d, "state_trace.jsonl")) as f:
        for l in f:
            if not l.strip(): continue
            r = json.loads(l)
            t.append(r["t"]); pe.append(r["pos_err"]); ae.append(r["ang_err"])
            o = r["obj"]; xs.append(o[4]); ys.append(o[5]); yaws.append(quat_yaw(o[:4]))
    return {k: np.array(v) for k, v in
            zip("t pe ae x y yaw".split(), (t, pe, ae, xs, ys, yaws))}

def window_idx(t, span):
    """indices of last `span` seconds"""
    return np.where(t >= t[-1] - span)[0]

def terminal_stats(tr, span):
    i = window_idx(tr["t"], span)
    if len(i) < 2: return None
    s, e = i[0], i[-1]
    dx = np.diff(tr["x"][i]); dy = np.diff(tr["y"][i])
    return dict(
        d_epos=tr["pe"][e] - tr["pe"][s],
        d_eyaw=tr["ae"][e] - tr["ae"][s],
        net_disp=math.hypot(tr["x"][e]-tr["x"][s], tr["y"][e]-tr["y"][s]),
        path_len=float(np.sum(np.hypot(dx, dy))),
        net_yaw=abs(wrap(tr["yaw"][e] - tr["yaw"][s])),
        best_pos_improve=float(tr["pe"][s] - tr["pe"][i].min()),
        best_yaw_improve=float(tr["ae"][s] - tr["ae"][i].min()),
    )

def flattest_window(tr, span=30.0):
    """peak-to-trough of pos/yaw err in the 30 s window with min pos_err std."""
    t = tr["t"]; best = None
    starts = t[t <= t[-1] - span]
    for s0 in starts[::5]:
        m = (t >= s0) & (t < s0 + span)
        if m.sum() < 10: continue
        sd = tr["pe"][m].std()
        if best is None or sd < best[0]:
            best = (sd, float(np.ptp(tr["pe"][m])), float(np.ptp(tr["ae"][m])))
    return (best[1], best[2]) if best else (np.nan, np.nan)

def inside_spans(tr):
    ins = (tr["pe"] < POS_TOL) & (tr["ae"] < ANG_TOL)
    t = tr["t"]
    dt = np.diff(t); dt = np.append(dt, np.median(dt) if len(dt) else 0.1)
    time_inside = float(dt[ins].sum())
    # longest continuous inside span
    longest, cur = 0.0, 0.0
    for k in range(len(ins)):
        if ins[k]:
            cur += dt[k]; longest = max(longest, cur)
        else:
            cur = 0.0
    return time_inside, longest, bool(ins.any())

def contact_stats(d):
    csvs = [f for f in os.listdir(d) if f.endswith("_metrics.csv")]
    if not csvs: return {}
    st_l, c_l, pe_l, ae_l = [], [], [], []
    with open(os.path.join(d, csvs[0])) as f:
        for r in csv.DictReader(f):
            st_l.append(float(r["sim_time"]))
            c_l.append(float(r["physical_contact_active"]) > 0.5)
            pe_l.append(float(r["position_error_m"]))
            ae_l.append(float(r["orientation_error_rad"]))
    c = np.array(c_l); st = np.array(st_l)
    n = len(c)
    frac = float(c.mean())
    # episodes
    edges = np.diff(c.astype(int))
    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)
    if c[0]: starts = [0] + starts
    if c[-1]: ends = ends + [n]
    eps = list(zip(starts, ends))
    dur_s = [max(st[e-1] - st[s], 0.0) for s, e in eps]
    total_min = max((st[-1] - st[0]) / 60.0, 1e-9)
    # productive: over +/-3-step window, d(e_pos)/dt < -1mm/s OR d(e_yaw)/dt < -0.01 rad/s
    pe = np.array(pe_l); ae = np.array(ae_l)
    k = 3
    prod = np.zeros(n, bool)
    lo = np.clip(np.arange(n) - k, 0, n-1); hi = np.clip(np.arange(n) + k, 0, n-1)
    dt = np.maximum(st[hi] - st[lo], 1e-6)
    dpe = (pe[hi] - pe[lo]) / dt; dae = (ae[hi] - ae[lo]) / dt
    prod = (dpe < -1e-3) | (dae < -1e-2)
    pcf = float(prod[c].mean()) if c.any() else 0.0
    return dict(physical_contact_fraction=frac,
                productive_contact_fraction=pcf,
                n_contact_episodes=len(eps),
                mean_episode_steps=float(np.mean([e-s for s, e in eps])) if eps else 0.0,
                mean_episode_s=float(np.mean(dur_s)) if dur_s else 0.0,
                contact_acquisition_rate=len(eps) / total_min,
                total_control_steps=n)

def analyze(args):
    scene, pair = args
    d = os.path.join(RUNS, scene, f"pair{pair}")
    run_id = f"{scene}_pair{pair}"
    tr = load_trace(d)
    res = {}
    rj = [f for f in os.listdir(d) if f.endswith("_result.json")]
    if rj: res = json.load(open(os.path.join(d, rj[0])))
    first_t = res.get("first_success_t")
    official = first_t is not None
    time_inside, longest, any_cross = inside_spans(tr)
    sustained = longest >= DWELL_S
    # Dwell censoring: the launcher stops the run ~2.5 s after the recorder
    # SUCCESS latch, so a 5 s post-success window cannot exist. If the trace
    # ends inside tolerance and ends <5 s after first_success, the dwell is
    # right-censored, not failed.
    inside_at_end = bool((tr["pe"][-1] < POS_TOL) and (tr["ae"][-1] < ANG_TOL))
    dwell_censored = bool(official and not sustained and inside_at_end
                          and (tr["t"][-1] - float(first_t or 0)) < DWELL_S)
    transient = (official or any_cross) and not sustained and not dwell_censored

    term = {sp: terminal_stats(tr, sp) for sp in (30, 60, 120)}
    fp, fy = flattest_window(tr)
    hold_fp = hold_fy = np.nan
    if official:
        m = tr["t"] >= float(first_t)
        if m.sum() > 5:
            hold_fp = float(np.ptp(tr["pe"][m])); hold_fy = float(np.ptp(tr["ae"][m]))

    cs = contact_stats(d)

    t120 = term[120] or term[60] or term[30]
    if (scene, pair) in CRASHED:
        outcome = "RUNTIME_TERMINATION"
    elif official and (sustained or dwell_censored):
        outcome = "SUCCESS"
    elif transient:
        outcome = "TRANSIENT_SUCCESS"
    elif t120 and (t120["best_pos_improve"] > 0.01 or t120["best_yaw_improve"] > 0.05):
        outcome = "TIMEOUT_PROGRESSING"
    elif t120:
        outcome = "TRUE_STALL"
    else:
        outcome = "UNKNOWN_OUTCOME"

    row = dict(run_id=run_id, scene=scene, pair=pair,
               duration_trace_s=float(tr["t"][-1]),
               official_success=official,
               first_success_time_s=first_t,
               time_inside_goal_s=round(time_inside, 2),
               longest_inside_span_s=round(longest, 2),
               sustained_success=sustained, dwell_censored=dwell_censored,
               transient_success=transient,
               task_outcome=outcome,
               final_pos_err=float(tr["pe"][-1]), final_yaw_err=float(tr["ae"][-1]),
               best_pos_err=float(tr["pe"].min()), best_yaw_err=float(tr["ae"].min()),
               flat30_pos_ptp=fp, flat30_yaw_ptp=fy,
               hold_pos_ptp=hold_fp, hold_yaw_ptp=hold_fy)
    row.update({k: cs.get(k) for k in
                ("total_control_steps", "physical_contact_fraction",
                 "productive_contact_fraction", "n_contact_episodes",
                 "mean_episode_steps", "mean_episode_s", "contact_acquisition_rate")})

    trow = dict(run_id=run_id, scene=scene, pair=pair)
    for sp in (30, 60, 120):
        tt = term[sp]
        for k in ("d_epos", "d_eyaw", "net_disp", "path_len", "net_yaw",
                  "best_pos_improve", "best_yaw_improve"):
            trow[f"{k}_{sp}s"] = round(tt[k], 6) if tt else np.nan
    return row, trow

if __name__ == "__main__":
    jobs = [(s, p) for s in SCENES for p in ("01", "02", "03", "04", "05")]
    with Pool(16) as pool:
        out = pool.map(analyze, jobs)
    rows, trows = zip(*out)
    def wcsv(path, recs):
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(recs[0].keys()))
            w.writeheader(); w.writerows(recs)
    wcsv(os.path.join(OUT, "stage1_outcomes.csv"), list(rows))
    wcsv(os.path.join(OUT, "terminal_progress.csv"), list(trows))
    for r in rows:
        print("%-26s %-20s dur=%7.1f fs=%s tin=%7.2f long=%6.2f bp=%.4f by=%.4f cf=%.3f pcf=%.3f" % (
            r["run_id"], r["task_outcome"], r["duration_trace_s"],
            r["first_success_time_s"], r["time_inside_goal_s"],
            r["longest_inside_span_s"], r["best_pos_err"], r["best_yaw_err"],
            r["physical_contact_fraction"] or 0, r["productive_contact_fraction"] or 0))
    from collections import Counter
    print("\nOutcome counts:", dict(Counter(r["task_outcome"] for r in rows)))
    fps = np.array([r["flat30_pos_ptp"] for r in rows])
    fys = np.array([r["flat30_yaw_ptp"] for r in rows])
    print("Noise floor pooled (flattest-30s ptp): pos med=%.4f p90=%.4f  yaw med=%.4f p90=%.4f"
          % (np.nanmedian(fps), np.nanpercentile(fps, 90),
             np.nanmedian(fys), np.nanpercentile(fys, 90)))
    hp = np.array([r["hold_pos_ptp"] for r in rows if not math.isnan(r["hold_pos_ptp"])])
    hy = np.array([r["hold_yaw_ptp"] for r in rows if not math.isnan(r["hold_yaw_ptp"])])
    if len(hp):
        print("Post-success hold ptp: pos med=%.4f max=%.4f  yaw med=%.4f max=%.4f"
              % (np.median(hp), hp.max(), np.median(hy), hy.max()))
    floor_p = float(np.nanmedian(fps)); floor_y = float(np.nanmedian(fys))
    for mult in (1, 2, 4):
        thp, thy = 2*mult*floor_p, 2*mult*floor_y
        flips = []
        for r, tr_ in zip(rows, trows):
            if r["task_outcome"] not in ("TIMEOUT_PROGRESSING", "TRUE_STALL"): continue
            newp = tr_["best_pos_improve_120s"] > max(thp, 0.0) or \
                   tr_["best_yaw_improve_120s"] > max(thy, 0.0)
            lab = "TIMEOUT_PROGRESSING" if newp else "TRUE_STALL"
            if lab != r["task_outcome"]: flips.append((r["run_id"], lab))
        print(f"sensitivity x{mult}: thresholds pos>{thp:.4f} yaw>{thy:.4f} flips={flips}")
