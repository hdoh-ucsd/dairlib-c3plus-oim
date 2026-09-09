#!/usr/bin/env python3
"""Paired baseline-vs-relu comparison from the per-run CSVs.

Reads metrics/{baseline,relu}_all_runs.csv, metrics/safety_per_run.csv,
chomp/per_run_chomp.csv. Writes:
  metrics/paired_comparison.csv  one row per scene/start/goal, both variants
  metrics/scene_summary.csv      per-scene + aggregate medians, both variants
  metrics/stats.md               paired stats: median paired diffs + 10k
                                 bootstrap 95% CIs, McNemar b/c counts
Idempotent full rebuild (derived files only). Usage: paired_compare.py
"""
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C

M = os.path.join(C.ROOT, "metrics")
RNG = np.random.default_rng(20260908)
NBOOT = 10000


def read_csv(path):
    if not os.path.isfile(path):
        return []
    with open(path) as f:
        return [r for r in csv.DictReader(f) if not r.get(
            list(r)[0], "").startswith("#")]


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def load_all():
    """Index: (variant, scene, start, goal) -> merged metrics dict."""
    idx = {}
    for v in C.VARIANTS:
        for r in read_csv(os.path.join(M, f"{v}_all_runs.csv")):
            idx[(v, r["scene"], int(r["start"]), int(r["goal"]))] = dict(r)
    for r in read_csv(os.path.join(M, "safety_per_run.csv")):
        k = (r["variant"], r["scene"], int(r["start"]), int(r["goal"]))
        if k in idx:
            idx[k].update({f"safety_{a}": r[a] for a in
                           ("min_clearance_m", "n_neg_samples",
                            "max_penetration_m", "contact_episodes")})
    for r in read_csv(os.path.join(C.ROOT, "chomp", "per_run_chomp.csv")):
        k = (r["variant"], r["scene"], int(r["start"]), int(r["goal"]))
        if k in idx:
            idx[k]["M_CHOMP"] = r["M_CHOMP"]
            idx[k]["M_CHOMP_norm"] = r["M_CHOMP_norm"]
    return idx


PAIR_METRICS = [  # (name, per-run field, lower_is_better?)
    ("T_goal", "first_success_t", True),
    ("min_clearance", "safety_min_clearance_m", False),
    ("chomp", "M_CHOMP", True),
    ("chomp_norm", "M_CHOMP_norm", True),
    ("final_pos_err", "final_pos_err_m", True),
    ("final_yaw_err", "final_yaw_err_rad", True),
]


def boot_ci(diffs):
    d = np.asarray([x for x in diffs if not math.isnan(x)])
    if len(d) == 0:
        return float("nan"), float("nan"), float("nan"), 0
    med = float(np.median(d))
    if len(d) == 1:
        return med, med, med, 1
    samp = RNG.choice(d, size=(NBOOT, len(d)), replace=True)
    meds = np.median(samp, axis=1)
    return med, float(np.percentile(meds, 2.5)), \
        float(np.percentile(meds, 97.5)), len(d)


def median_of(rows, field):
    v = [fnum(r.get(field)) for r in rows]
    v = [x for x in v if not math.isnan(x)]
    return float(np.median(v)) if v else float("nan")


def main():
    idx = load_all()
    keys = sorted({(sc, s, g) for (_, sc, s, g) in idx})

    # ---- paired_comparison.csv ----
    pc_fields = ["scene", "start", "goal"]
    for v in ("baseline", "relu"):
        pc_fields += [f"{v}_success", f"{v}_T_goal", f"{v}_min_clearance",
                      f"{v}_chomp", f"{v}_final_pos_err", f"{v}_final_yaw_err"]
    pc_fields += ["d_T_goal", "d_min_clearance", "d_chomp", "d_final_pos_err",
                  "d_final_yaw_err"]
    pc_rows = []
    for sc, s, g in keys:
        b = idx.get(("baseline", sc, s, g))
        r = idx.get(("relu", sc, s, g))
        row = dict(scene=sc, start=s, goal=g)
        for v, m in (("baseline", b), ("relu", r)):
            row[f"{v}_success"] = m["success"] if m else ""
            row[f"{v}_T_goal"] = (m.get("first_success_t", "") if m else "")
            row[f"{v}_min_clearance"] = \
                (m.get("safety_min_clearance_m", "") if m else "")
            row[f"{v}_chomp"] = (m.get("M_CHOMP", "") if m else "")
            row[f"{v}_final_pos_err"] = (m.get("final_pos_err_m", "") if m
                                         else "")
            row[f"{v}_final_yaw_err"] = (m.get("final_yaw_err_rad", "") if m
                                         else "")
        for name, fb in (("T_goal", "T_goal"), ("min_clearance",
                                                "min_clearance"),
                         ("chomp", "chomp"), ("final_pos_err",
                                              "final_pos_err"),
                         ("final_yaw_err", "final_yaw_err")):
            bb, rr = fnum(row[f"baseline_{fb}"]), fnum(row[f"relu_{fb}"])
            row[f"d_{name}"] = ("" if math.isnan(bb) or math.isnan(rr)
                                else f"{rr - bb:.6g}")
        pc_rows.append(row)
    C.write_csv(os.path.join(M, "paired_comparison.csv"), pc_fields, pc_rows)

    # ---- scene_summary.csv ----
    ss_fields = ["scene", "variant", "n_runs", "success_rate",
                 "median_T_goal_s", "median_min_clearance_m",
                 "median_chomp", "median_chomp_norm", "penetration_runs",
                 "median_final_pos_err_m", "median_final_yaw_err_rad"]
    ss_rows = []
    scenes = sorted({sc for (sc, _, _) in keys}) + ["ALL"]
    for sc in scenes:
        for v in C.VARIANTS:
            rows = [m for (vv, ss, _, _), m in idx.items()
                    if vv == v and (sc == "ALL" or ss == sc)]
            if not rows:
                continue
            succ = [int(r["success"]) for r in rows]
            pen = sum(1 for r in rows
                      if fnum(r.get("safety_n_neg_samples", "0")) > 0)
            ss_rows.append(dict(
                scene=sc, variant=v, n_runs=len(rows),
                success_rate=f"{np.mean(succ):.3f}",
                median_T_goal_s=f"{median_of(rows, 'first_success_t'):.4g}",
                median_min_clearance_m=(
                    f"{median_of(rows, 'safety_min_clearance_m'):.4g}"),
                median_chomp=f"{median_of(rows, 'M_CHOMP'):.4g}",
                median_chomp_norm=f"{median_of(rows, 'M_CHOMP_norm'):.4g}",
                penetration_runs=pen,
                median_final_pos_err_m=(
                    f"{median_of(rows, 'final_pos_err_m'):.4g}"),
                median_final_yaw_err_rad=(
                    f"{median_of(rows, 'final_yaw_err_rad'):.4g}")))
        # paired delta row (relu - baseline over pairs present in both)
        pairs = [(idx[("baseline", ssc, s, g)], idx[("relu", ssc, s, g)])
                 for (ssc, s, g) in keys
                 if (sc == "ALL" or ssc == sc)
                 and ("baseline", ssc, s, g) in idx
                 and ("relu", ssc, s, g) in idx]
        if pairs:
            def dmed(field):
                d = [fnum(r.get(field)) - fnum(b.get(field))
                     for b, r in pairs]
                d = [x for x in d if not math.isnan(x)]
                return f"{np.median(d):.4g}" if d else "nan"
            ss_rows.append(dict(
                scene=sc, variant="delta(relu-baseline)", n_runs=len(pairs),
                success_rate=f"{np.mean([int(r['success']) for _, r in pairs]) - np.mean([int(b['success']) for b, _ in pairs]):+.3f}",
                median_T_goal_s=dmed("first_success_t"),
                median_min_clearance_m=dmed("safety_min_clearance_m"),
                median_chomp=dmed("M_CHOMP"),
                median_chomp_norm=dmed("M_CHOMP_norm"),
                penetration_runs="",
                median_final_pos_err_m=dmed("final_pos_err_m"),
                median_final_yaw_err_rad=dmed("final_yaw_err_rad")))
    C.write_csv(os.path.join(M, "scene_summary.csv"), ss_fields, ss_rows)

    # ---- stats.md (paired stats over ALL matched pairs) ----
    pairs = [(idx[("baseline", sc, s, g)], idx[("relu", sc, s, g)])
             for (sc, s, g) in keys
             if ("baseline", sc, s, g) in idx and ("relu", sc, s, g) in idx]
    lines = ["# Paired baseline-vs-relu stats", "",
             f"Matched pairs: {len(pairs)} (bootstrap {NBOOT} resamples, "
             "seed 20260908; delta = relu - baseline)", ""]
    fld = {"T_goal": "first_success_t",
           "min_clearance": "safety_min_clearance_m",
           "chomp": "M_CHOMP", "chomp_norm": "M_CHOMP_norm",
           "final_pos_err": "final_pos_err_m",
           "final_yaw_err": "final_yaw_err_rad"}
    lines.append("| metric | n | median delta | 95% CI |")
    lines.append("|---|---|---|---|")
    for name, f in fld.items():
        diffs = [fnum(r.get(f)) - fnum(b.get(f)) for b, r in pairs]
        med, lo, hi, n = boot_ci(diffs)
        lines.append(f"| {name} | {n} | {med:.5g} | [{lo:.5g}, {hi:.5g}] |")
    # McNemar
    b_cnt = sum(1 for bb, rr in pairs
                if int(bb["success"]) == 1 and int(rr["success"]) == 0)
    c_cnt = sum(1 for bb, rr in pairs
                if int(bb["success"]) == 0 and int(rr["success"]) == 1)
    lines += ["", f"McNemar discordant counts: b (baseline-only success) = "
              f"{b_cnt}, c (relu-only success) = {c_cnt}", ""]
    with open(os.path.join(M, "stats.md"), "w") as f:
        f.write("\n".join(lines))
    print(f"wrote paired_comparison.csv ({len(pc_rows)} rows), "
          f"scene_summary.csv ({len(ss_rows)} rows), stats.md "
          f"({len(pairs)} matched pairs)")


if __name__ == "__main__":
    main()
