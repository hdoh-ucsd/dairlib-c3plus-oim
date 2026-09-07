#!/usr/bin/env python3
"""Unified scoring/aggregation for the matched C3+ vs OIM benchmark.

Protocol: cap 100 sim s; success = pos_err < 0.05 m AND ang_err < 0.1 rad
simultaneously at some t <= cap. Paired start/goal poses per (scene, pair).

Idempotent; tolerates partial/missing trials (campaigns still running).
Re-run at any time:
  /root/miniconda3/envs/push_anything_ADMM/bin/python3 score_all.py
"""
import csv
import glob
import json
import math
import os
import re
import sys

RUNS = "/root/push_anything_ADMM/results/final_oim_c3plus_comparison_runs"
OUT = "/root/push_anything_ADMM/results/final_oim_c3plus_comparison"
CAP = 100.0
POS_TOL, ANG_TOL = 0.05, 0.1
MOTION_MM = 0.005          # "meaningful motion" threshold (m)
PROG_WIN = 30.0            # final window for stall classification (s)
PROG_POS, PROG_ANG = 0.005, 0.05  # improvement thresholds in that window


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def last_motion_time(ts, xys):
    """Latest t after which the object still moves > MOTION_MM (from that t)."""
    if len(ts) < 2:
        return 0.0
    # reverse scan keeping bounding box of future xy positions
    minx = maxx = xys[-1][0]
    miny = maxy = xys[-1][1]
    last = 0.0
    for i in range(len(ts) - 2, -1, -1):
        x, y = xys[i]
        dx = max(abs(maxx - x), abs(x - minx))
        dy = max(abs(maxy - y), abs(y - miny))
        if math.hypot(dx, dy) > MOTION_MM:
            last = ts[i]
            break
        minx, maxx = min(minx, x), max(maxx, x)
        miny, maxy = min(miny, y), max(maxy, y)
    return last


def score_series(ts, pos, ang, xys):
    """Common per-trial scoring given capped, aligned series."""
    m = {}
    m["t_end"] = ts[-1] if ts else 0.0
    succ_t = None
    for t, p, a in zip(ts, pos, ang):
        if p < POS_TOL and a < ANG_TOL:
            succ_t = t
            break
    m["success"] = succ_t is not None
    m["t_success"] = succ_t
    m["final_pos"], m["final_yaw"] = (pos[-1], ang[-1]) if ts else (None, None)
    m["best_pos"] = min(pos) if pos else None
    m["best_yaw"] = min(ang) if ang else None
    m["last_motion_t"] = last_motion_time(ts, xys) if ts else 0.0
    # stall classification (only meaningful for failures)
    m["stall_class"] = ""
    if ts and not m["success"]:
        t0 = ts[-1] - PROG_WIN
        idx = [i for i, t in enumerate(ts) if t >= t0]
        if idx:
            i0 = idx[0]
            prog = (pos[i0] - pos[-1] > PROG_POS) or (ang[i0] - ang[-1] > PROG_ANG)
            m["stall_class"] = "TIMEOUT_PROGRESSING" if prog else "TRUE_STALL"
    return m


def count(path, needle):
    try:
        with open(path, errors="replace") as f:
            return f.read().count(needle)
    except OSError:
        return None


# ---------------------------------------------------------------- C3+ loading
def load_c3plus_trial(tdir):
    trace = os.path.join(tdir, "state_trace.jsonl")
    if not os.path.isfile(trace):
        return None
    ts, pos, ang, xys = [], [], [], []
    try:
        with open(trace) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue  # torn tail line of a live run
                t = r.get("t")
                if t is None or t > CAP:
                    break
                obj = r.get("obj", [])
                if len(obj) < 7:
                    continue
                ts.append(t)
                pos.append(r.get("pos_err", float("nan")))
                ang.append(abs(wrap(r.get("ang_err", float("nan")))))
                xys.append((obj[4], obj[5]))
    except OSError:
        return None
    if not ts:
        return None
    m = score_series(ts, pos, ang, xys)
    plog = os.path.join(tdir, "planner.log")
    m["repositions"] = count(plog, "Repositioning")
    m["reached_repos"] = count(plog, "reached repositioning target")
    m["buffer_overflows"] = count(plog, "buffer overflow")
    m["topple_guards"] = count(plog, "TOPPLE_GUARD")
    m["exhausted"] = count(plog, "EXHAUSTED")
    m["crashes"] = count(plog, "Traceback")
    m["releases"] = count(os.path.join(tdir, "osc.log"), "PRELIFT_RELEASE")
    notes = []
    if m["t_end"] < CAP - 1 and not m["success"]:
        notes.append("trace ends %.1fs (partial/crash?)" % m["t_end"])
    if m["crashes"]:
        notes.append("%d traceback(s) in planner.log" % m["crashes"])
    m["notes"] = "; ".join(notes)
    return m


# ---------------------------------------------------------------- OIM loading
def load_oim_trial(tdir):
    rlog = os.path.join(tdir, "run.log")
    if not os.path.isfile(rlog):
        return None
    try:
        txt = open(rlog, errors="replace").read()
    except OSError:
        return None
    mm = re.search(r"saved run to (\S+\.json)", txt)
    if not mm:
        return {"notes": "run.log present, no saved-run JSON yet (in progress?)",
                "incomplete": True}
    jpath = mm.group(1)
    if not os.path.isfile(jpath):
        return {"notes": "saved-run JSON missing: %s" % jpath, "incomplete": True}
    try:
        d = json.load(open(jpath))
    except (OSError, json.JSONDecodeError):
        return {"notes": "unreadable run JSON: %s" % jpath, "incomplete": True}
    dyn, st = d.get("dynamic", {}), d.get("static", {})
    goal = st.get("goal")
    times = dyn.get("time", [])
    poses = dyn.get("object_pose", [])
    if not goal or not times or not poses:
        return {"notes": "run JSON lacks time/pose/goal", "incomplete": True}
    gx, gy, gyaw = goal[0], goal[1], goal[2]
    ts, pos, ang, xys = [], [], [], []
    for t, p in zip(times, poses):
        if t > CAP:
            break
        ts.append(t)
        pos.append(math.hypot(p[0] - gx, p[1] - gy))
        ang.append(abs(wrap(p[2] - gyaw)))
        xys.append((p[0], p[1]))
    if not ts:
        return {"notes": "no samples within cap", "incomplete": True}
    m = score_series(ts, pos, ang, xys)
    m["run_json"] = jpath
    notes = []
    if m["t_end"] < CAP - 1 and not m["success"]:
        notes.append("run ends %.1fs < cap without success" % m["t_end"])
    m["notes"] = "; ".join(notes)
    return m


# --------------------------------------------------------- failure classifier
def classify_c3plus(m):
    """Earliest-cause class from planner.log signals + trace shape."""
    if m.get("crashes"):
        return "F13", "planner traceback (crash)"
    if m["stall_class"] == "TIMEOUT_PROGRESSING":
        return "F10", "still progressing at cap"
    repos = m.get("repositions") or 0
    # F1: contact acquisition — object barely moved, reposition churn dominates
    if m["last_motion_t"] < 10.0 and repos >= 5:
        return "F1", ("object motion stops by t=%.1fs with %d repositions "
                      "(contact-acquisition churn)" % (m["last_motion_t"], repos))
    # F4: local fixed point — pose stable for a long tail, planner alive
    quiet = m["t_end"] - m["last_motion_t"]
    if quiet >= PROG_WIN:
        return "F4", ("pose stable for final %.0fs (last motion t=%.1fs), "
                      "planner alive, %d repositions" % (quiet, m["last_motion_t"], repos))
    return "F13", "unclassified stall (motion until t=%.1fs)" % m["last_motion_t"]


def classify_oim(m):
    """OIM internals are NOT instrumented here — trajectory shape only;
    conservative F10/F4/F13."""
    if m["stall_class"] == "TIMEOUT_PROGRESSING":
        return "F10", "still progressing at cap (trajectory shape only)"
    quiet = m["t_end"] - m["last_motion_t"]
    if quiet >= PROG_WIN:
        return "F4", ("pose stable for final %.0fs - consistent with a local "
                      "fixed point; OIM internals not instrumented" % quiet)
    return "F13", "unclassified from trajectory shape; OIM internals not instrumented"


# ----------------------------------------------------------------- enumerate
def enumerate_trials():
    rows = []

    def add(method, scene, pair, rep, m, tdir):
        rows.append(dict(method=method, scene=scene, pair=pair, rep=rep,
                         m=m, tdir=tdir))

    def scan(base, loader, method):
        for sdir in sorted(glob.glob(os.path.join(base, "*"))):
            if not os.path.isdir(sdir):
                continue
            scene = os.path.basename(sdir)
            for tdir in sorted(glob.glob(os.path.join(sdir, "trial*"))):
                k = re.search(r"trial(\d+)", tdir)
                if k:
                    add(method, scene, int(k.group(1)), "a", loader(tdir), tdir)
            for tdir in sorted(glob.glob(os.path.join(sdir, "t*_*"))):
                k = re.search(r"^t(\d+)_([a-z])$", os.path.basename(tdir))
                if k:
                    add(method, scene, int(k.group(1)), k.group(2),
                        loader(tdir), tdir)

    scan(os.path.join(RUNS, "c3plus_matched"), load_c3plus_trial, "c3plus")
    scan(os.path.join(RUNS, "c3plus_matched_b"), load_c3plus_trial, "c3plus")
    scan(os.path.join(RUNS, "oim_matched"), load_oim_trial, "oim")
    scan(os.path.join(RUNS, "oim_matched_b"), load_oim_trial, "oim")
    return rows


def fmt(v, nd=4):
    if v is None:
        return ""
    if isinstance(v, float):
        return "%.*f" % (nd, v)
    return str(v)


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return None
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def main():
    os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "figures"), exist_ok=True)
    rows = enumerate_trials()

    # ---------------- trial_metrics.csv
    tm_path = os.path.join(OUT, "metrics", "trial_metrics.csv")
    with open(tm_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["method", "scene", "pair", "rep", "success", "t_success",
                    "final_pos", "final_yaw", "best_pos", "best_yaw",
                    "last_motion_t", "stall_class", "repositions", "releases",
                    "buffer_overflows", "notes"])
        for r in rows:
            m = r["m"]
            if m is None:
                w.writerow([r["method"], r["scene"], r["pair"], r["rep"],
                            "", "", "", "", "", "", "", "", "", "", "",
                            "no data yet (trial dir exists)"])
                continue
            if m.get("incomplete"):
                w.writerow([r["method"], r["scene"], r["pair"], r["rep"],
                            "", "", "", "", "", "", "", "", "", "", "",
                            m.get("notes", "incomplete")])
                continue
            w.writerow([
                r["method"], r["scene"], r["pair"], r["rep"],
                int(m["success"]), fmt(m["t_success"], 1),
                fmt(m["final_pos"]), fmt(m["final_yaw"]),
                fmt(m["best_pos"]), fmt(m["best_yaw"]),
                fmt(m["last_motion_t"], 1), m["stall_class"],
                fmt(m.get("repositions")), fmt(m.get("releases")),
                fmt(m.get("buffer_overflows")), m.get("notes", "")])

    # ---------------- scene_summary.csv
    complete = [r for r in rows if r["m"] and not r["m"].get("incomplete")]
    groups = {}
    for r in complete:
        groups.setdefault((r["scene"], r["method"]), []).append(r["m"])
    ss_path = os.path.join(OUT, "metrics", "scene_summary.csv")
    with open(ss_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scene", "method", "n", "successes", "success_rate",
                    "median_t_success", "median_final_pos", "median_final_yaw"])
        for (scene, meth) in sorted(groups):
            ms = groups[(scene, meth)]
            succ = [m for m in ms if m["success"]]
            w.writerow([scene, meth, len(ms), len(succ),
                        fmt(len(succ) / len(ms), 2),
                        fmt(median([m["t_success"] for m in succ]), 1),
                        fmt(median([m["final_pos"] for m in ms])),
                        fmt(median([m["final_yaw"] for m in ms]))])

    # ---------------- failure_classification.csv
    fc_path = os.path.join(OUT, "metrics", "failure_classification.csv")
    with open(fc_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["method", "scene", "pair", "rep", "class", "evidence"])
        for r in complete:
            m = r["m"]
            if m["success"]:
                continue
            cls, ev = (classify_c3plus(m) if r["method"] == "c3plus"
                       else classify_oim(m))
            w.writerow([r["method"], r["scene"], r["pair"], r["rep"], cls, ev])

    # ---------------- figures
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        scenes = sorted({s for (s, _) in groups})
        methods = ["c3plus", "oim"]
        colors = {"c3plus": "#3b6ea5", "oim": "#c46a4a"}
        x = range(len(scenes))
        width = 0.38

        fig, ax = plt.subplots(figsize=(7, 4))
        for i, meth in enumerate(methods):
            vals, ns = [], []
            for s in scenes:
                ms = groups.get((s, meth), [])
                ns.append(len(ms))
                vals.append(sum(m["success"] for m in ms) / len(ms) if ms else 0)
            bars = ax.bar([xi + (i - 0.5) * width for xi in x], vals, width,
                          label=meth.upper(), color=colors[meth])
            for b, n in zip(bars, ns):
                ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.02,
                        "n=%d" % n, ha="center", fontsize=8)
        ax.set_xticks(list(x)); ax.set_xticklabels(scenes)
        ax.set_ylim(0, 1.15); ax.set_ylabel("success rate")
        ax.set_title("Matched C3+ vs OIM: success rate (pos<0.05, yaw<0.1, cap 100s)")
        ax.legend(); fig.tight_layout()
        fig.savefig(os.path.join(OUT, "figures", "success_rates.png"), dpi=150)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 4))
        for i, meth in enumerate(methods):
            vals = []
            for s in scenes:
                ms = [m for m in groups.get((s, meth), []) if m["success"]]
                vals.append(median([m["t_success"] for m in ms]) if ms else 0)
            ax.bar([xi + (i - 0.5) * width for xi in x], vals, width,
                   label=meth.upper(), color=colors[meth])
        ax.set_xticks(list(x)); ax.set_xticklabels(scenes)
        ax.set_ylabel("median time-to-goal among successes (s)")
        ax.set_title("Matched C3+ vs OIM: time to goal (0 = no successes yet)")
        ax.legend(); fig.tight_layout()
        fig.savefig(os.path.join(OUT, "figures", "time_to_goal.png"), dpi=150)
        plt.close(fig)
    except Exception as e:  # figures are best-effort
        print("WARN: figure generation failed: %s" % e, file=sys.stderr)

    # ---------------- console interim table
    print("wrote %s (%d trial rows, %d scored)" % (tm_path, len(rows), len(complete)))
    print(open(ss_path).read())


if __name__ == "__main__":
    main()
