#!/usr/bin/env python3
import os, json, glob
import numpy as np, pandas as pd

BASE = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics/results/xarm6_c3plus_scene_smoke"
roster = pd.read_csv(f"{BASE}/metrics/pair_campaign_summary.csv", dtype={"pair": str})
fails = roster[~roster.success_trace].copy()

rows = []
for _, r in fails.iterrows():
    scene, pair = r.scene, r.pair
    csvs = glob.glob(f"{BASE}/runs/{scene}/pair{pair}/*_metrics.csv")
    assert len(csvs) == 1, (scene, pair, csvs)
    df = pd.read_csv(csvs[0])
    n = len(df)
    Wi = int(n * 0.75)
    W = df.iloc[Wi:]
    H = df.iloc[n // 2:]
    pos = df.position_error_m.values
    ang = df.orientation_error_rad.values

    contact_frac = float((df.physical_contact_active > 0).mean())
    best_pos, best_ang = float(pos.min()), float(ang.min())
    final_pos, final_ang = float(pos[-1]), float(ang[-1])
    init_pos, init_ang = float(pos[0]), float(ang[0])
    pos_prog_W = float(W.position_error_m.iloc[0] - W.position_error_m.iloc[-1])
    ang_prog_W = float(W.orientation_error_rad.iloc[0] - W.orientation_error_rad.iloc[-1])
    ang_prog_total = init_ang - final_ang
    ang_prog_frac = float(ang_prog_total / init_ang) if init_ang > 1e-6 else 1.0
    med_clear_W = float(W.min_obstacle_clearance.median())
    pos_std_H = float(H.position_error_m.std())
    # last sim_time where running-best pos error improved by >5mm
    runmin = np.minimum.accumulate(pos)
    imp = np.where(np.diff(runmin) < -0.005)[0]
    last_prog_t = float(df.sim_time.iloc[imp[-1] + 1]) if len(imp) else float(df.sim_time.iloc[0])
    eval_W = float(W.evaluation_total.mean())
    eval_all = float(df.evaluation_total.mean())

    if contact_frac < 0.05 and (init_pos - best_pos) < 0.05:
        cls = "NEVER_ACQUIRED"
    elif med_clear_W < 0.03 and pos_prog_W < 0.01:
        cls = "OBSTACLE_BLOCKED"
    elif (best_pos < 0.15 or final_pos < 0.2) and final_ang > 0.3 and ang_prog_W < 0.05:
        cls = "ROTATION_STALL"
    elif final_ang < 0.3 and final_pos > 0.05 and pos_prog_W < 0.01:
        cls = "LAST_DECIMETER_STALL"
    elif final_ang > 1.5 and ang_prog_frac < 0.4:
        cls = "EARLY_ROTATION_GRIND"
    elif pos_prog_W > 0.02 or ang_prog_W > 0.1:
        cls = "SLOW_MONOTONE_PROGRESS"
    elif pos_std_H > 0.05:
        cls = "OSCILLATION_CHURN"
    else:
        cls = "OTHER_PLATEAU"

    crash = ""
    try:
        with open(f"{BASE}/runs/{scene}/pair{pair}/launcher.log") as fh:
            if "Aborted" in fh.read():
                crash = f"CRASH(planner abort @t={df.sim_time.iloc[-1]:.0f}s) "
    except OSError:
        pass
    notes = (f"{crash}n={n} init_pos={init_pos:.3f} init_ang={init_ang:.3f} "
             f"pos_std_lasthalf={pos_std_H:.3f} eval_W={eval_W:.0f} eval_all={eval_all:.0f}")
    rows.append(dict(scene=scene, pair=pair, cls=cls, contact_frac=round(contact_frac, 3),
                     med_clearance_W=round(med_clear_W, 4), pos_prog_W=round(pos_prog_W, 4),
                     ang_prog_W=round(ang_prog_W, 4), ang_prog_frac_total=round(ang_prog_frac, 3),
                     last_progress_t=round(last_prog_t, 1), final_pos=round(final_pos, 4),
                     final_ang=round(final_ang, 4), best_pos=round(best_pos, 4),
                     best_ang=round(best_ang, 4), notes=notes))

out = pd.DataFrame(rows).rename(columns={"cls": "class"})
out.to_csv(f"{BASE}/metrics/failure_classification_600s.csv", index=False)
print(out.to_string(index=False))
print()
print(out["class"].value_counts())
