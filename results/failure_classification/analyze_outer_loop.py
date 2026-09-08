#!/usr/bin/env python3
"""Outer-loop episode analysis for the 30-run xArm6 C3+ scene-smoke campaign."""
import glob, os, re, json, math
import numpy as np
import pandas as pd
from concurrent.futures import ProcessPoolExecutor

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics"
RUNS = os.path.join(WT, "results/xarm6_c3plus_scene_smoke/runs")
OUT = os.path.join(WT, "results/failure_classification")

GAP_C3 = 0.010          # m: near-contact always counts as C3-like
SPEED_C3 = 0.015        # m/s: dwell (smoothed tip speed) -> C3-like
SPEED_REPO = 0.03       # m/s: fast transit -> reposition-like (hysteresis exit)
SMOOTH = 11             # steps rolling-median window for tip speed
MIN_EP = 60             # steps, minimum episode length
MERGE_GAP = 15          # steps: merge episodes separated by shorter transits
NOISE_EPOS = 0.002      # m, progress noise floor
NOISE_EYAW = 0.01       # rad
CONTACTLESS_THRESH = 0.70

PATTERNS = {
    "c3_entry": "Switching to C3 because reached repositioning target",
    "reposition_noprog": "Repositioning after not making progress in C3",
    "reposition_goodsample": "Repositioning because found good sample",
    "cost_cross": "Crossed cost switching threshold.",
    "workspace_viol": "WorkspaceLimitViolations",
}

def log_counts(path):
    counts = {k: 0 for k in PATTERNS}
    vocab = {}
    try:
        with open(path, errors="replace") as f:
            for line in f:
                for k, p in PATTERNS.items():
                    if p in line:
                        counts[k] += 1
                l = line.strip()
                if re.search(r"Switching|Repositioning|Crossed|[Ww]orkspace|[Tt]opple|[Ee]xhaust|abort|assert", l):
                    key = re.sub(r"\d+(\.\d+)?", "#", l)[:160]
                    vocab[key] = vocab.get(key, 0) + 1
    except FileNotFoundError:
        pass
    return counts, vocab

def segment(df):
    """Return list of (start_idx, end_idx) C3-like episodes (inclusive idx into df)."""
    gap = df["pusher_object_gap"].to_numpy()
    spd = (np.hypot(df["tip_x"].diff(), df["tip_y"].diff()) / df["sim_time"].diff()) \
        .rolling(SMOOTH, center=True, min_periods=1).median().to_numpy()
    lab = np.zeros(len(gap), dtype=bool)
    in_c3 = False
    for i in range(len(gap)):
        g, v = gap[i], spd[i]
        near = np.isfinite(g) and g < GAP_C3
        if in_c3:
            # stay unless clear fast transit away from contact
            in_c3 = near or not (np.isfinite(v) and v > SPEED_REPO)
        else:
            in_c3 = near or (np.isfinite(v) and v < SPEED_C3)
        lab[i] = in_c3
    # episodes
    raw = []
    i = 0
    n = len(lab)
    while i < n:
        if lab[i]:
            j = i
            while j + 1 < n and lab[j + 1]:
                j += 1
            raw.append([i, j])
            i = j + 1
        else:
            i += 1
    merged = []
    for e in raw:
        if merged and e[0] - merged[-1][1] <= MERGE_GAP:
            merged[-1][1] = e[1]
        else:
            merged.append(e)
    return [(a, b) for a, b in merged if b - a + 1 >= MIN_EP]

def sector(dx, dy, yaw):
    # object-frame standoff vector
    c, s = math.cos(-yaw), math.sin(-yaw)
    ox, oy = c * dx - s * dy, s * dx + c * dy
    a = math.atan2(oy, ox) % (2 * math.pi)
    return int(a // (math.pi / 4))

def analyze(rundir):
    scene = os.path.basename(os.path.dirname(rundir))
    pair = os.path.basename(rundir)
    run_id = f"{scene}/{pair}"
    counts, vocab = log_counts(os.path.join(rundir, "planner.log"))
    csvs = glob.glob(os.path.join(rundir, "xarm6_c3plus_*_metrics.csv"))
    if not csvs:
        return dict(run_id=run_id, scene=scene, pair=pair, error="no metrics csv",
                    **counts), [], vocab
    df = pd.read_csv(csvs[0])
    eps = segment(df)
    total_time = float(df["sim_time"].iloc[-1] - df["sim_time"].iloc[0]) if len(df) else 0.0
    dt = total_time / max(len(df) - 1, 1)

    rows = []
    failed_sectors = set()
    phantom_time = 0.0
    acq_fail = 0
    noprog = 0
    equiv = 0
    for k, (i, j) in enumerate(eps):
        sub = df.iloc[i:j + 1]
        contact = float(sub["physical_contact_active"].mean())
        disp = float(np.hypot(sub["object_x"].iloc[-1] - sub["object_x"].iloc[0],
                              sub["object_y"].iloc[-1] - sub["object_y"].iloc[0]))
        yawc = float(abs((sub["object_yaw"].iloc[-1] - sub["object_yaw"].iloc[0] + math.pi) % (2 * math.pi) - math.pi))
        epr = float(sub["position_error_m"].iloc[0] - sub["position_error_m"].iloc[-1])
        eyr = float(sub["orientation_error_rad"].iloc[0] - sub["orientation_error_rad"].iloc[-1])
        progress = (epr > NOISE_EPOS) or (eyr > NOISE_EYAW)
        exit_reason = "progress" if progress else "no-progress"
        sec = sector(sub["tip_x"].iloc[0] - sub["object_x"].iloc[0],
                     sub["tip_y"].iloc[0] - sub["object_y"].iloc[0],
                     sub["object_yaw"].iloc[0])
        eq = sec in failed_sectors
        if not progress:
            noprog += 1
            failed_sectors.add(sec)
        if eq:
            equiv += 1
        dur_t = float(sub["sim_time"].iloc[-1] - sub["sim_time"].iloc[0])
        contactless = contact < (1 - CONTACTLESS_THRESH)
        if contactless and not progress:
            phantom_time += dur_t
        if sub["physical_contact_active"].sum() == 0:
            acq_fail += 1
        rows.append(dict(run_id=run_id, episode_idx=k,
                         start_step=int(sub["control_step"].iloc[0]), end_step=int(sub["control_step"].iloc[-1]),
                         start_t=round(float(sub["sim_time"].iloc[0]), 3), end_t=round(float(sub["sim_time"].iloc[-1]), 3),
                         duration_steps=j - i + 1, contact_fraction=round(contact, 4),
                         obj_disp_m=round(disp, 5), yaw_change_rad=round(yawc, 5),
                         epos_reduction=round(epr, 5), eyaw_reduction=round(eyr, 5),
                         exit_reason=exit_reason, object_frame_sector=sec,
                         equivalent_to_prior_failed=eq))
    c3_mask_steps = sum(j - i + 1 for i, j in eps)
    c3_steps_contactless = 0
    for i, j in eps:
        c3_steps_contactless += int((df["physical_contact_active"].iloc[i:j + 1] == 0).sum())
    n_ep = len(eps)
    log_c3 = counts["c3_entry"]
    summary = dict(
        run_id=run_id, scene=scene, pair=pair,
        reposition_count=counts["reposition_noprog"],
        goodsample_reposition_count=counts["reposition_goodsample"],
        cost_cross_count=counts["cost_cross"],
        workspace_violation=counts["workspace_viol"],
        c3_entry_count_log=log_c3,
        c3_episodes_behavioral=n_ep,
        reconciliation_ratio=round(n_ep / log_c3, 3) if log_c3 else float("nan"),
        contactless_c3_fraction=round(c3_steps_contactless / c3_mask_steps, 4) if c3_mask_steps else float("nan"),
        c3_contact_fraction=round(1 - c3_steps_contactless / c3_mask_steps, 4) if c3_mask_steps else float("nan"),
        phantom_churn_runtime_fraction=round(phantom_time / total_time, 4) if total_time else float("nan"),
        acquisition_failure_rate=round(acq_fail / n_ep, 4) if n_ep else float("nan"),
        equivalent_failed_reselection_rate=round(equiv / n_ep, 4) if n_ep else float("nan"),
        no_progress_exit_count=noprog,
        total_sim_time_s=round(total_time, 1),
        n_steps=len(df),
        final_epos=round(float(df["position_error_m"].iloc[-1]), 4),
        final_eyaw=round(float(df["orientation_error_rad"].iloc[-1]), 4),
    )
    return summary, rows, vocab

def main():
    rundirs = sorted(glob.glob(os.path.join(RUNS, "*", "pair*")))
    with ProcessPoolExecutor(16) as ex:
        results = list(ex.map(analyze, rundirs))
    summaries, all_rows, vocab_all = [], [], {}
    for s, rows, vocab in results:
        summaries.append(s)
        all_rows.extend(rows)
        for k, v in vocab.items():
            vocab_all[k] = vocab_all.get(k, 0) + v
    pd.DataFrame(all_rows).to_csv(os.path.join(OUT, "c3_segment_audit.csv"), index=False)
    sdf = pd.DataFrame(summaries)
    sdf.to_csv(os.path.join(OUT, "outer_loop_summary.csv"), index=False)
    with open(os.path.join(OUT, "vocab_census.json"), "w") as f:
        json.dump(dict(sorted(vocab_all.items(), key=lambda kv: -kv[1])), f, indent=1)
    print(sdf.to_string())
    print("\nPer-scene medians:")
    print(sdf.groupby("scene")[["phantom_churn_runtime_fraction", "contactless_c3_fraction",
                                "acquisition_failure_rate", "equivalent_failed_reselection_rate",
                                "reconciliation_ratio"]].median().to_string())

if __name__ == "__main__":
    main()
