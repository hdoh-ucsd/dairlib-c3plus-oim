#!/usr/bin/env python3
"""Offline C3+ cost reconstruction + redesigned per-run diagnostics (v2).

Reconstructs, at the MEASURED state of every control step, the local C3
objective components actually used by the frozen xArm6 C3+ stack (see
results/cost_analysis/cost_semantics_notes.md):

  J_trans(k): post-latch (once e_pos < 0.25 m, sticky)  10000*dx^2 + 10000*dy^2 + 6000*dz^2
              pre-latch                                  12500*(dx^2 + dy^2 + dz^2)
              (dz ~ 0: metrics.csv carries planar pose only; z error treated as 0)
  J_rot(k)  = 510 * wrap(yaw - goal_yaw)^2 post-latch; 5 * e^2 pre-latch (negligible by design)
  J_C3_task = J_trans + J_rot   (the valid local-objective total; CASE C)
  J_obs_rank(k) = sum_obstacles 5000 * exp(-(d - r)/0.04)   [RANKING layer, obstacle-blind
              local C3 — plotted but NEVER summed into J_C3_task]
              d = distance from object CENTER to disc center / polygon (signed: negative inside).

IMPORTANT CAVEAT (applies to every 'prediction' analysis downstream): the controller's true
per-candidate PREDICTED rollout costs were never logged (CostLogger env SAMPLING_C3_COST_LOG_DIR
unset for this campaign). The measured-state cost at an episode's start step is only a weak
proxy for the chosen candidate's predicted cost; all outputs label it accordingly.
"""
import csv
import math
import os
import sys
from multiprocessing import Pool

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics"
RUNS = os.path.join(WT, "results/xarm6_c3plus_scene_smoke/runs")
FC = os.path.join(WT, "results/failure_classification")
OUT = os.path.join(WT, "results/cost_analysis")
SCFG = os.path.join(WT, "tools/scene_smoke/scene_configs")

SCENES = ["icra_sign", "open_task", "shelf_gap", "single_obstacle", "slalom", "ycb_clutter"]
CRASH_RUNS = {"single_obstacle/pair04", "single_obstacle/pair05"}  # runtime termination (F10)

W_XY, W_Z, W_PRE = 10000.0, 6000.0, 12500.0
W_ROT_POST, W_ROT_PRE = 510.0, 5.0
LATCH = 0.25
W_OBS, SIG_OBS = 5000.0, 0.04


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def poly_signed_dist(px, py, poly):
    """Signed distance point->convex polygon (negative inside)."""
    n = len(poly)
    dmin = math.inf
    inside = False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        rx, ry = px - x1, py - y1
        t = max(0.0, min(1.0, (rx * ex + ry * ey) / (ex * ex + ey * ey)))
        dx, dy = rx - t * ex, ry - t * ey
        d2 = dx * dx + dy * dy
        if d2 < dmin:
            dmin = d2
        if (y1 > py) != (y2 > py) and px < x1 + ex * (py - y1) / ey:
            inside = not inside  # even-odd raycast, winding-agnostic
    dmin = math.sqrt(dmin)
    return -dmin if inside else dmin


def load_scene_obstacles(scene):
    with open(os.path.join(SCFG, scene + ".yaml")) as f:
        cfg = yaml.safe_load(f)
    obs = cfg.get("obstacles") or {}
    discs = [tuple(map(float, d)) for d in (obs.get("discs") or [])]
    polys = [[tuple(map(float, v)) for v in p] for p in (obs.get("polygons") or [])]
    return discs, polys


def j_obs_series(x, y, discs, polys):
    j = np.zeros_like(x)
    for ox, oy, r in discs:
        j += W_OBS * np.exp(-(np.hypot(x - ox, y - oy) - r) / SIG_OBS)
    if polys:
        for i in range(len(x)):
            for p in polys:
                j[i] += W_OBS * math.exp(-poly_signed_dist(x[i], y[i], p) / SIG_OBS)
    return j


def read_metrics(scene, pair):
    d = os.path.join(RUNS, scene, pair)
    fn = [f for f in os.listdir(d) if f.endswith("_metrics.csv")][0]
    cols = ["control_step", "sim_time", "object_x", "object_y", "object_yaw",
            "goal_x", "goal_y", "goal_yaw", "position_error_m",
            "orientation_error_rad", "physical_contact_active"]
    data = {c: [] for c in cols}
    with open(os.path.join(d, fn)) as f:
        for row in csv.DictReader(f):
            for c in cols:
                data[c].append(float(row[c]))
    return d, fn[:-len("_metrics.csv")], {c: np.asarray(v) for c, v in data.items()}


def load_episodes():
    eps = {}
    with open(os.path.join(FC, "c3_segment_audit.csv")) as f:
        for row in csv.DictReader(f):
            eps.setdefault(row["run_id"], []).append(row)
    return eps


def load_failure_audit():
    fa = {}
    with open(os.path.join(FC, "failure_audit.csv")) as f:
        for row in csv.DictReader(f):
            # run_id like open_task_pair01 -> scene/pair01
            rid = row["run_id"]
            for sc in SCENES:
                if rid.startswith(sc + "_"):
                    fa[sc + "/" + rid[len(sc) + 1:]] = row
                    break
    return fa


def process_run(args):
    scene, pair, episodes, fa_row = args
    run_id = f"{scene}/{pair}"
    d, stem, m = read_metrics(scene, pair)
    k = m["control_step"]
    epos, eth = m["position_error_m"], m["orientation_error_rad"]
    dx = m["object_x"] - m["goal_x"]
    dy = m["object_y"] - m["goal_y"]
    eyaw = wrap(m["object_yaw"] - m["goal_yaw"])

    # latch: first step with e_pos < 0.25, sticky
    latched = np.zeros(len(k), bool)
    idx = np.argmax(epos < LATCH) if np.any(epos < LATCH) else None
    if idx is not None:
        latched[idx:] = True
    j_trans = np.where(latched, W_XY * dx**2 + W_XY * dy**2, W_PRE * (dx**2 + dy**2))
    j_rot = np.where(latched, W_ROT_POST * eyaw**2, W_ROT_PRE * eyaw**2)
    j_task = j_trans + j_rot
    discs, polys = load_scene_obstacles(scene)
    j_obs = j_obs_series(m["object_x"], m["object_y"], discs, polys)
    has_obs = bool(discs or polys)

    outcome = fa_row["task_outcome"]
    pfail = fa_row["primary_failure_class"]
    succ_t = fa_row["first_success_time_s"]
    succ_step = None
    if succ_t not in ("", "nan"):
        st = float(succ_t)
        if not math.isnan(st):
            succ_step = k[int(np.argmin(np.abs(m["sim_time"] - st)))]

    # ---- figure ----
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(16, 5.5), sharex=True)
    contact = m["physical_contact_active"] > 0.5

    for ax in (axL, axR):
        # episode shading (C3-like/contact-rich, light blue)
        for ep in episodes:
            s, e = float(ep["start_step"]), float(ep["end_step"])
            e = min(e, k[-1])
            if s <= k[-1]:
                ax.axvspan(s, e, color="#add8e6", alpha=0.35, lw=0)
        # contact on/off ticks
        trans = np.flatnonzero(np.diff(contact.astype(int)))
        ons = [k[i + 1] for i in trans if contact[i + 1]]
        offs = [k[i + 1] for i in trans if not contact[i + 1]]
        if len(ons) + len(offs) > 100:
            dec = math.ceil((len(ons) + len(offs)) / 100)
            ons, offs = ons[::dec], offs[::dec]
        for x0 in ons:
            ax.axvline(x0, color="green", lw=0.4, alpha=0.6, ymax=0.05)
        for x0 in offs:
            ax.axvline(x0, color="red", lw=0.4, alpha=0.6, ymax=0.05)
        if succ_step is not None:
            ax.axvline(succ_step, color="green", lw=1.6, ls="-", label="first success")
        if run_id in CRASH_RUNS:
            ax.plot([k[-1]], [ax.get_ylim()[0]], "rx", ms=12, mew=3, clip_on=False,
                    label="runtime termination")
            ax.axvline(k[-1], color="red", lw=1.2, ls=":")
        else:
            ax.axvline(k[-1], color="gray", lw=1.2, ls="--", label="timeout/end")

    axL.plot(k, epos, color="#1f77b4", lw=1.0, label="position error (m)")
    axL.plot(k, eth, color="#ff7f0e", lw=1.0, label="orientation error (rad)")
    axL.axhline(0.05, color="#1f77b4", ls=":", lw=1, label="0.05 m")
    axL.axhline(0.10, color="#ff7f0e", ls=":", lw=1, label="0.10 rad")
    axL.set_title(f"{run_id} — Task diagnostics\n(shaded = C3-like episodes; ticks = contact on/off)")
    axL.set_xlabel("control step")
    axL.set_ylabel("error")
    axL.legend(fontsize=7, loc="upper right")

    axR.plot(k, j_trans, color="#2ca02c", lw=0.9, label="J_trans")
    axR.plot(k, j_rot, color="#ff7f0e", lw=0.9, label="J_rot")
    axR.plot(k, j_task, color="black", lw=1.8, label="J_C3_task = J_trans + J_rot")
    lbl = "J_obs (ranking layer)" + ("" if has_obs else " ≡ 0 (no obstacles)")
    axR.plot(k, j_obs, color="#d62728", lw=1.0, ls="--", label=lbl)
    axR.set_yscale("symlog", linthresh=1.0)
    axR.set_title(f"C3+ cost decomposition (measured state; outcome={outcome})\n"
                  "J_obs is the RANKING layer — local C3 is obstacle-blind (CASE C)")
    axR.set_xlabel("control step")
    axR.set_ylabel("cost (symlog)")
    axR.legend(fontsize=7, loc="upper right")

    fig.tight_layout()
    fig.savefig(os.path.join(d, stem + "_cost_diagnostics_v2.png"), dpi=110)
    plt.close(fig)

    # ---- summary row ----
    denom = epos + np.abs(eth)
    r = float(np.corrcoef(j_task, denom)[0, 1]) if len(k) > 2 else float("nan")
    summary = dict(
        run_id=run_id, scene=scene, outcome=outcome, primary_failure=pfail,
        initial_J_trans=j_trans[0], final_J_trans=j_trans[-1],
        initial_J_rot=j_rot[0], final_J_rot=j_rot[-1],
        max_J_obs=float(j_obs.max()), terminal_J_obs=float(j_obs[-1]),
        mean_J_trans=float(j_trans.mean()), mean_J_rot=float(j_rot.mean()),
        mean_J_obs=float(j_obs.mean()),
        cost_progress_correlation=r,
        physical_contact_fraction=float(contact.mean()),
    )

    # ---- transactions (per C3-like episode) ----
    tx = []
    kmax = int(k[-1])
    step_index = {int(s): i for i, s in enumerate(k)}
    def at(step):
        step = min(int(step), kmax)
        while step not in step_index and step > 0:
            step -= 1
        return step_index.get(step, 0)
    for ep in episodes:
        s, e = int(float(ep["start_step"])), int(float(ep["end_step"]))
        if s > kmax:
            continue
        i0, i1 = at(s), at(e)
        cf = float(ep["contact_fraction"])
        tx.append(dict(
            run_id=run_id, episode_idx=ep["episode_idx"],
            start_step=s, end_step=min(e, kmax),
            contact_fraction=cf,
            J_start=float(j_task[i0]), J_end=float(j_task[i1]),
            dJ_measured=float(j_task[i1] - j_task[i0]),
            d_e_pos=float(epos[i1] - epos[i0]),
            d_e_yaw=float(abs(eth[i1]) - abs(eth[i0])),
            contactless=cf < 0.05,
        ))
    return summary, tx


def main():
    eps = load_episodes()
    fa = load_failure_audit()
    only = sys.argv[1:] if len(sys.argv) > 1 else None
    jobs = []
    for sc in SCENES:
        for p in sorted(os.listdir(os.path.join(RUNS, sc))):
            rid = f"{sc}/{p}"
            if only and rid not in only:
                continue
            jobs.append((sc, p, eps.get(rid, []), fa[rid]))
    with Pool(16) as pool:
        results = pool.map(process_run, jobs)
    if only:
        print("validation-only run done:", only)
        return
    summaries = [s for s, _ in results]
    txs = [t for _, tl in results for t in tl]

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "all_runs_cost_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summaries[0]))
        w.writeheader()
        w.writerows(summaries)
    with open(os.path.join(OUT, "transactions.csv"), "w", newline="") as f:
        f.write("# NOTE: true PREDICTED candidate costs were never logged (SAMPLING_C3_COST_LOG_DIR unset);\n")
        f.write("# J_start is the MEASURED-state task cost at episode start — a weak proxy for the chosen\n")
        f.write("# candidate's predicted rollout cost. All 'prediction' analyses are proxy-based.\n")
        w = csv.DictWriter(f, fieldnames=list(txs[0]))
        w.writeheader()
        w.writerows(txs)

    # ---- aggregate figures ----
    # 1. cost_by_failure_class
    classes = sorted({s["primary_failure"] for s in summaries})
    fig, ax = plt.subplots(figsize=(11, 5))
    xpos = np.arange(len(classes))
    for off, key, col in [(-0.25, "mean_J_trans", "#2ca02c"), (0.0, "mean_J_rot", "#ff7f0e"),
                          (0.25, "mean_J_obs", "#d62728")]:
        vals = [np.mean([s[key] for s in summaries if s["primary_failure"] == c]) for c in classes]
        ax.bar(xpos + off, vals, width=0.24, color=col, label=key)
    ax.set_yscale("log")
    ax.set_xticks(xpos)
    ax.set_xticklabels([c.replace("_", "\n") for c in classes], fontsize=7)
    ax.set_ylabel("per-step mean cost (log)")
    ax.set_title("Mean reconstructed C3+ cost components by primary failure class\n"
                 "(J_obs = ranking layer, never part of the local C3 objective)")
    ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "cost_by_failure_class.png"), dpi=110); plt.close(fig)

    # 2. cost_vs_realized_progress
    fig, ax = plt.subplots(figsize=(8, 6))
    for cl, col, lab in [(True, "#d62728", "contactless (<5% contact)"), (False, "#1f77b4", "contact-rich")]:
        xs = [t["dJ_measured"] for t in txs if t["contactless"] == cl]
        ys = [t["d_e_pos"] + t["d_e_yaw"] for t in txs if t["contactless"] == cl]
        ax.scatter(xs, ys, s=10, alpha=0.5, color=col, label=f"{lab} (n={len(xs)})")
    ax.set_xscale("symlog", linthresh=1); ax.axhline(0, color="gray", lw=0.6); ax.axvline(0, color="gray", lw=0.6)
    ax.set_xlabel("episode ΔJ_measured (J_C3_task end − start, symlog)")
    ax.set_ylabel("realized progress Δe_pos + Δe_yaw (negative = progress)")
    ax.set_title("Episode measured-cost change vs realized progress\n(PROXY: predicted candidate costs were never logged)")
    ax.legend(); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "cost_vs_realized_progress.png"), dpi=110); plt.close(fig)

    # 3. contact_vs_cost_prediction_error
    fig, ax = plt.subplots(figsize=(8, 6))
    cfr = np.array([t["contact_fraction"] for t in txs])
    prog = np.array([-(t["d_e_pos"] + t["d_e_yaw"]) for t in txs])
    lowc = cfr < 0.05
    noprog = np.abs(prog) < 0.01
    ax.scatter(cfr[~(lowc & noprog)], prog[~(lowc & noprog)], s=10, alpha=0.5, color="#1f77b4", label="other episodes")
    ax.scatter(cfr[lowc & noprog], prog[lowc & noprog], s=14, alpha=0.7, color="#d62728",
               label=f"low-contact & no-progress (n={int((lowc & noprog).sum())})")
    ax.set_xlabel("episode physical contact fraction")
    ax.set_ylabel("realized progress −(Δe_pos + Δe_yaw)")
    ax.set_title("Contact fraction vs realized progress per C3-like episode\n"
                 "(PROXY-BASED: true predicted costs unlogged; measured-state cost only)")
    ax.legend(); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "contact_vs_cost_prediction_error.png"), dpi=110); plt.close(fig)

    # 4. terminal_cost_by_outcome
    outcomes = sorted({s["outcome"] for s in summaries})
    fig, ax = plt.subplots(figsize=(9, 5))
    xpos = np.arange(len(outcomes))
    for off, key, col in [(-0.25, "final_J_trans", "#2ca02c"), (0.0, "final_J_rot", "#ff7f0e"),
                          (0.25, "terminal_J_obs", "#d62728")]:
        vals = [np.mean([s[key] for s in summaries if s["outcome"] == o]) for o in outcomes]
        ax.bar(xpos + off, vals, width=0.24, color=col, label=key)
    ax.set_yscale("symlog", linthresh=0.1)
    ax.set_xticks(xpos); ax.set_xticklabels(outcomes, fontsize=8)
    ax.set_ylabel("terminal cost (symlog)")
    ax.set_title("Terminal reconstructed cost components by task outcome")
    ax.legend(); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "terminal_cost_by_outcome.png"), dpi=110); plt.close(fig)

    # headline stats
    dj_less = np.median([t["dJ_measured"] for t in txs if t["contactless"]])
    dj_rich = np.median([t["dJ_measured"] for t in txs if not t["contactless"]])
    pr_less = np.median([-(t["d_e_pos"] + t["d_e_yaw"]) for t in txs if t["contactless"]])
    pr_rich = np.median([-(t["d_e_pos"] + t["d_e_yaw"]) for t in txs if not t["contactless"]])
    n_less = sum(t["contactless"] for t in txs)
    print(f"episodes: {len(txs)} total, {n_less} contactless")
    print(f"median dJ: contactless {dj_less:.2f} vs contact-rich {dj_rich:.2f}")
    print(f"median progress: contactless {pr_less:.5f} vs contact-rich {pr_rich:.5f}")
    for s in summaries:
        if s["outcome"] == "SUCCESS":
            print(f"SUCCESS {s['run_id']}: final J_task = {s['final_J_trans']+s['final_J_rot']:.2f}")
    rs = [s["cost_progress_correlation"] for s in summaries]
    print(f"cost-progress Pearson r: median {np.median(rs):.3f}, min {min(rs):.3f}")


if __name__ == "__main__":
    main()
