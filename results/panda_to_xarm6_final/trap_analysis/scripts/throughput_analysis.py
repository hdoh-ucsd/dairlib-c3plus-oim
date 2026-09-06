#!/usr/bin/env python3
"""Task A: throughput & time budget for xArm6 5-joint port (r5 trials 1-5 + r4 trial1)."""
import json, math, os, re, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA = "/root/push_anything_ADMM/results"
OUT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant/results/panda_to_xarm6_final/trap_analysis"
os.makedirs(os.path.join(OUT, "figures"), exist_ok=True)

TRIALS = [("r5_trial%d" % i, f"{DATA}/panda_to_xarm6_port_final_r5/xarm6_trial{i}") for i in range(1, 6)]
TRIALS.append(("r4_trial1", f"{DATA}/panda_to_xarm6_port_final_r4/xarm6_trial1"))


def load_trace(d):
    T, pos, yaw, obj = [], [], [], []
    with open(os.path.join(d, "state_trace.jsonl")) as f:
        for line in f:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            T.append(r["t"]); pos.append(r["pos_err"]); yaw.append(r["ang_err"])
            obj.append(r["obj"][4:6])
    return np.array(T), np.array(pos), np.array(yaw), np.array(obj)


def load_osc(d):
    ts, tips, qd = [], [], []
    pat = re.compile(r"XARM6_5J t=(\d+) p_des=\s*(\S+)\s+(\S+)\s+(\S+) p_tip=\s*(\S+)\s+(\S+)\s+(\S+) \|qdot_cmd\|=(\S+)")
    prelift = 0
    with open(os.path.join(d, "osc.log")) as f:
        for line in f:
            if line.startswith("PRELIFT_RELEASE=START"):
                prelift += 1
            m = pat.search(line)
            if m:
                ts.append(int(m.group(1)))
                tips.append([float(m.group(i)) for i in (5, 6, 7)])
                qd.append(float(m.group(8)))
    return np.array(ts), np.array(tips), np.array(qd), prelift


def rolling_rate(T, val, win):
    """progress (decrease of val) per 100 s over trailing window `win`, at each t."""
    out_t, out_r = [], []
    for i in range(len(T)):
        t = T[i]
        j = np.searchsorted(T, t - win)
        if T[i] - T[j] < win * 0.9:
            continue
        out_t.append(t)
        out_r.append((val[j] - val[i]) / (T[i] - T[j]) * 100.0)
    return np.array(out_t), np.array(out_r)


def net_translation(T, obj, win):
    out_t, out_d = [], []
    for i in range(len(T)):
        j = np.searchsorted(T, T[i] - win)
        if T[i] - T[j] < win * 0.9:
            continue
        out_t.append(T[i])
        out_d.append(np.linalg.norm(obj[i] - obj[j]) / (T[i] - T[j]) * 100.0)
    return np.array(out_t), np.array(out_d)


def phase_seconds(T, obj, tips_t, tips):
    """Approximate push vs reposition seconds from motion structure:
    a 1 s bin is 'push' if the object moved >0.3 mm in it, else if EE moved >5 mm it's 'reposition',
    else 'idle/press'. planner.log Repositioning/Switching lines carry no timestamps, so this is
    a trace-structure proxy, stated as such."""
    push = repos = idle = 0
    tipmap = {int(t): p for t, p in zip(tips_t, tips)}
    for s in range(int(T[0]) + 1, int(T[-1])):
        i0 = np.searchsorted(T, s); i1 = np.searchsorted(T, s + 1)
        if i1 >= len(T) or i1 <= i0:
            continue
        dobj = np.linalg.norm(obj[min(i1, len(T)-1)] - obj[i0])
        dtip = None
        if s in tipmap and (s + 1) in tipmap:
            dtip = np.linalg.norm(tipmap[s + 1] - tipmap[s])
        if dobj > 3e-4:
            push += 1
        elif dtip is not None and dtip > 5e-3:
            repos += 1
        else:
            idle += 1
    return push, repos, idle


rows = []
fig, axes = plt.subplots(3, 1, figsize=(11, 12), sharex=True)
colors = plt.cm.tab10(np.linspace(0, 1, 10))

for ci, (name, d) in enumerate(TRIALS):
    T, pos, yaw, obj = load_trace(d)
    ot, tips, qd, prelift = load_osc(d)
    dur = T[-1]
    # rolling rates
    r100 = rolling_rate(T, pos, 100); r300 = rolling_rate(T, pos, 300)
    y300 = rolling_rate(T, yaw, 300)
    tr300 = net_translation(T, obj, 300)
    # final-300s window progress
    j = np.searchsorted(T, T[-1] - 300)
    fin_pos = pos[j] - pos[-1]
    fin_yaw = yaw[j] - yaw[-1]
    progressing = (fin_pos > 0.005) or (fin_yaw > 0.02)
    # best sustained 300s rates
    best_pos_rate = np.nanmax(r300[1]) if len(r300[1]) else float("nan")
    best_yaw_rate = np.nanmax(y300[1]) if len(y300[1]) else float("nan")
    # EE speed from 1 Hz tips
    sp = np.linalg.norm(np.diff(tips, axis=0), axis=1) / np.diff(ot)
    sp = sp[np.diff(ot) == 1] if len(sp) else sp
    sat_frac = float(np.mean(qd >= 0.45)) if len(qd) else float("nan")
    push, repos, idle = phase_seconds(T, obj, ot, tips)
    # time-to-goal estimate for progressing trials
    ttg = ""
    if progressing:
        parts = []
        if pos[-1] > 0.02 and best_pos_rate > 1e-4:
            parts.append(pos[-1] / (best_pos_rate / 100.0))
        if yaw[-1] > 0.1 and best_yaw_rate > 1e-4:
            parts.append(yaw[-1] / (best_yaw_rate / 100.0))
        if parts:
            ttg = f"{max(parts):.0f}"
    rows.append(dict(
        trial=name, duration_s=round(dur, 1), final_pos_err=round(pos[-1], 4),
        final_yaw_err=round(yaw[-1], 4),
        net_transl_per100s_best300=round(float(np.nanmax(tr300[1])) if len(tr300[1]) else np.nan, 4),
        pos_err_red_per100s_best300=round(float(best_pos_rate), 4),
        yaw_red_per100s_best300=round(float(best_yaw_rate), 4),
        final300_pos_progress=round(float(fin_pos), 4), final300_yaw_progress=round(float(fin_yaw), 4),
        push_s=push, reposition_s=repos, idle_s=idle,
        prelift_release_events=prelift,
        ee_speed_median=round(float(np.median(sp)), 4) if len(sp) else np.nan,
        ee_speed_p90=round(float(np.percentile(sp, 90)), 4) if len(sp) else np.nan,
        ee_speed_max=round(float(np.max(sp)), 4) if len(sp) else np.nan,
        qdot_sat_frac=round(sat_frac, 3),
        classification="PROGRESSING_BUT_TIME_CAPPED" if progressing else "TRUE_STALL",
        est_additional_time_to_goal_s=ttg,
    ))
    c = colors[ci]
    axes[0].plot(*rolling_rate(T, pos, 100), color=c, lw=0.8, alpha=0.6)
    axes[0].plot(*r300, color=c, lw=1.8, label=name)
    axes[1].plot(*rolling_rate(T, yaw, 100), color=c, lw=0.8, alpha=0.6)
    axes[1].plot(*y300, color=c, lw=1.8, label=name)
    axes[2].plot(*tr300, color=c, lw=1.8, label=name)

axes[0].set_ylabel("pos_err reduction\n[m / 100 s]"); axes[0].axhline(0, color="k", lw=0.5)
axes[1].set_ylabel("yaw_err reduction\n[rad / 100 s]"); axes[1].axhline(0, color="k", lw=0.5)
axes[2].set_ylabel("net object translation\n[m / 100 s] (300 s win)")
axes[2].set_xlabel("t [s]")
axes[0].set_title("Rolling progress rates (thin=100 s window, thick=300 s window)")
for a in axes:
    a.legend(fontsize=7, ncol=3); a.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "figures", "throughput_rolling_window.png"), dpi=130)

with open(os.path.join(OUT, "throughput_time_to_goal.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)

for r in rows:
    print(r)
