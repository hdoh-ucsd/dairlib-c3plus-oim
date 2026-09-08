#!/usr/bin/env python3
"""Synthesis stage: failure_audit.csv, per-scene summary, confusion matrix (+PNG)."""
import pandas as pd, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics"
FC = os.path.join(WT, "results/failure_classification")

s1 = pd.read_csv(os.path.join(FC, "stage1_outcomes.csv"))
ol = pd.read_csv(os.path.join(FC, "outer_loop_summary.csv"))
tp = pd.read_csv(os.path.join(FC, "terminal_progress.csv"))

ol["run_key"] = ol.scene + "_pair" + ol.pair.str.replace("pair", "")
s1["run_key"] = s1.run_id
tp["run_key"] = tp.run_id

df = s1.merge(ol[["run_key","reposition_count","no_progress_exit_count",
                  "equivalent_failed_reselection_rate","contactless_c3_fraction",
                  "c3_contact_fraction","phantom_churn_runtime_fraction",
                  "acquisition_failure_rate"]], on="run_key")
df = df.merge(tp[["run_key","best_pos_improve_60s","best_pos_improve_120s"]], on="run_key")
assert len(df) == 30, len(df)

# (primary, secondary, confidence, ev_start, ev_end(None=duration), explanation)
C = {
 "open_task_pair01": ("F5_EXECUTION_PREDICTION_MISMATCH","","MEDIUM",38.1,40.7,
   "Latched at 38.1 s but exited tolerance 0.3 s later via a yaw overshoot to 0.130 rad the executed push did not arrest, and the launcher stopped before recovery."),
 "open_task_pair02": ("NONE","","HIGH",79.1,81.7,"Official success at 79.1 s; dwell right-censored at 2.7 s by launcher stop."),
 "open_task_pair03": ("NONE","","HIGH",125.8,128.5,"Official success at 125.8 s; dwell right-censored at 2.9 s."),
 "open_task_pair04": ("NONE","","HIGH",60.2,62.7,"Official success at 60.2 s; dwell right-censored at 2.6 s."),
 "open_task_pair05": ("NONE","","HIGH",114.0,116.7,"Official success at 114.0 s; dwell right-censored at 2.8 s."),
 "shelf_gap_pair01": ("NONE","","HIGH",186.2,189.2,"Official success at 186.2 s (only non-open-task success); dwell right-censored at 3.2 s."),
 # TIMEOUT_PROGRESSING
 "icra_sign_pair01": ("F12_TIMEOUT_BUT_PROGRESSING","","HIGH",191.0,311.0,
   "Terminal-120s window still improving (yaw progress 1.06 rad in earlier coarse pass, last progress at 306.6 s); errors monotone shrinking at cap."),
 "icra_sign_pair02": ("F12_TIMEOUT_BUT_PROGRESSING","F6_OBSTACLE_GEOMETRIC_BLOCK","MEDIUM",183.6,303.6,
   "Still improving in the terminal window (best e_yaw 0.056 rad) but pos held at 0.25-0.28 m with median obstacle clearance 8 mm, so the sign geometry slows the remaining translation."),
 "icra_sign_pair03": ("F12_TIMEOUT_BUT_PROGRESSING","","HIGH",193.1,313.1,
   "Terminal-120s best-error improvement exceeds thresholds; slow but ongoing progress against the sign obstacles at cap."),
 "icra_sign_pair04": ("F12_TIMEOUT_BUT_PROGRESSING","","HIGH",179.8,299.8,
   "Reached best e_yaw 0.134 rad and still improving in the terminal window when the 600 s wall cap fired."),
 "single_obstacle_pair03": ("F12_TIMEOUT_BUT_PROGRESSING","F3_FAILURE_MEMORY_RESELECTION","HIGH",240.1,360.1,
   "Still progressing at cap (best e_yaw 0.018 rad, 83 good-sample repositions) but 92% of C3 episodes re-approach an already-failed object-frame sector, taxing throughput."),
 "shelf_gap_pair05": ("F12_TIMEOUT_BUT_PROGRESSING","F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","HIGH",204.8,324.8,
   "Best failure of the campaign (best 0.074 m / 0.060 rad) and improving in the terminal window; 37% of runtime lost to contactless no-progress C3 episodes slowed it below the cap."),
 # RUNTIME_TERMINATION
 "single_obstacle_pair04": ("F7_REPOSITION_WORKSPACE_FAILURE","F10_RUNTIME_NUMERICAL","HIGH",0.0,15.3,
   "Planner aborted at 15.3 s on the CheckForWorkspaceLimitViolations assert (sampling_based_c3_controller.cc:3397) during a reposition; object untouched thereafter."),
 "single_obstacle_pair05": ("F7_REPOSITION_WORKSPACE_FAILURE","F10_RUNTIME_NUMERICAL","HIGH",0.0,21.2,
   "Planner aborted at 21.2 s on the same workspace-limit assert during a reposition; sim coasted to the cap with the object frozen at e_pos 0.60 m."),
 # TRUE_STALL — F2 churn/park family
 "single_obstacle_pair01": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,326.7,
   "93.7% of runtime is contactless no-progress C3 with the tip parked at a 7.5 cm standoff while the log flips modes 142 times (contactless C3 0.988, contact fraction 1.3%)."),
 "single_obstacle_pair02": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F1_OUTER_LOOP_CONTACT_ACQUISITION","MEDIUM",30.0,324.1,
   "Park variant of the churn loop: 141 logged repositions with almost no tip travel, contactless C3 0.986 and contact fraction 1.4%; behavioral phantom fraction under-counts because the tip never moves."),
 "shelf_gap_pair02": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,333.8,
   "47% of runtime in contactless no-progress C3 (contactless 0.935) with 97 repositions and 90% equivalent-sector reselection sustaining the loop; object contact only 3.4% of steps."),
 "shelf_gap_pair03": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,321.6,
   "Purest churn cycle in the campaign: 105 episodes with 104 no-progress exits, 99% acquisition failure and 96% equivalent-sector reselection; 62% of runtime is phantom churn."),
 "shelf_gap_pair04": ("F1_OUTER_LOOP_CONTACT_ACQUISITION","F3_FAILURE_MEMORY_RESELECTION","MEDIUM",30.0,326.8,
   "69% of C3 episodes achieve zero contact and 93% reselect an already-failed sector, but churn runtime fraction (0.27) is weaker than the acquisition failure itself; contact fraction 9.2% never becomes productive engagement."),
 "ycb_clutter_pair01": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,305.6,
   "86% of runtime is contactless no-progress C3 (contactless 0.965, contact fraction 3.4%); the controller optimizes against a believed contact that does not exist."),
 "ycb_clutter_pair02": ("F1_OUTER_LOOP_CONTACT_ACQUISITION","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,323.8,
   "98.5% of C3 episodes achieve zero contact across 72 repositions with 92% equivalent-sector reselection; churn runtime fraction (0.35) is secondary to the near-total acquisition failure."),
 "ycb_clutter_pair03": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,309.8,
   "83% of runtime is phantom churn (contactless C3 0.933, 91 repositions) with the object parked at e_yaw 2.86 rad."),
 "ycb_clutter_pair04": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,316.3,
   "69% phantom-churn runtime with 97.7% acquisition failure and 96.6% equivalent-sector reselection over 87 episodes — the churn cycle plus failure-memory loop in full."),
 "ycb_clutter_pair05": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F1_OUTER_LOOP_CONTACT_ACQUISITION","HIGH",30.0,314.5,
   "85% phantom-churn runtime; 126 logged repositions largely without tip travel (reconciliation 0.095) while contactless C3 is 0.970 and contact fraction 2.9%."),
 "slalom_pair01": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","MEDIUM",30.0,318.5,
   "45% phantom-churn runtime with 92 repositions and 88% equivalent reselection; the 12% contact it does get is only 7.6% productive (campaign-lowest), so no real engagement forms."),
 "slalom_pair02": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,319.0,
   "58% phantom-churn runtime, 90% acquisition failure and 95% equivalent-sector reselection over 108 episodes; contactless C3 0.972."),
 "slalom_pair03": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION","HIGH",30.0,301.1,
   "79% phantom-churn runtime with 115 logged repositions and the tip largely parked (reconciliation 0.235); contactless C3 0.958."),
 "slalom_pair04": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F1_OUTER_LOOP_CONTACT_ACQUISITION","HIGH",30.0,295.5,
   "93% phantom-churn runtime parked at a ~1.7 cm standoff while the log flips modes 104 times; contactless C3 0.982, contact fraction 1.8%."),
 "slalom_pair05": ("F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F1_OUTER_LOOP_CONTACT_ACQUISITION","MEDIUM",30.0,312.1,
   "Park variant: 135 logged repositions with almost no tip travel (reconciliation 0.044), contactless C3 0.992 and campaign-lowest contact fraction 0.8%; behavioral churn fraction under-counts the parked hover."),
 "icra_sign_pair05": ("F6_OBSTACLE_GEOMETRIC_BLOCK","F4_LOCAL_C3_FIXED_POINT","HIGH",118.8,294.6,
   "80% physical contact but only 14.5% productive with median obstacle clearance ~0 (negative; 98% of contact steps within 5 mm of the obstacle): the object is ground against the C-glyph hull after last progress at 118.8 s, so pushing that direction cannot progress."),
}

rows = []
for _, r in df.iterrows():
    k = r.run_key
    prim, sec, conf, es, ee, expl = C[k]
    rows.append(dict(
        run_id=k, scene=r.scene, start_id=f"s{int(r['pair']):02d}", goal_id=f"g{int(r['pair']):02d}",
        duration_s=r.duration_trace_s, total_control_steps=r.total_control_steps,
        official_success=r.official_success, sustained_success=r.sustained_success,
        first_success_time_s=r.first_success_time_s,
        final_position_error_m=round(r.final_pos_err,4), final_orientation_error_rad=round(r.final_yaw_err,4),
        best_position_error_m=round(r.best_pos_err,4), best_orientation_error_rad=round(r.best_yaw_err,4),
        terminal_progress_60s=round(r.best_pos_improve_60s,4), terminal_progress_120s=round(r.best_pos_improve_120s,4),
        physical_contact_fraction=round(r.physical_contact_fraction,4),
        productive_contact_fraction=round(r.productive_contact_fraction,4),
        c3_contact_fraction=round(r.c3_contact_fraction,4),
        contactless_c3_fraction=round(r.contactless_c3_fraction,4),
        reposition_count=r.reposition_count, no_progress_exit_count=r.no_progress_exit_count,
        equivalent_failed_reselection_rate=round(r.equivalent_failed_reselection_rate,4),
        task_outcome=r.task_outcome, primary_failure_class=prim, secondary_failure_class=sec,
        confidence=conf, evidence_start_s=es, evidence_end_s=ee,
        one_sentence_explanation=expl))
audit = pd.DataFrame(rows)
audit.to_csv(os.path.join(FC, "failure_audit.csv"), index=False)

# ---- per-scene summary ----
def fam(p):
    if p in ("F1_OUTER_LOOP_CONTACT_ACQUISITION","F2_OUTER_LOOP_PHANTOM_CONTACT_CHURN","F3_FAILURE_MEMORY_RESELECTION"): return "outer_loop"
    return {"F4_LOCAL_C3_FIXED_POINT":"local_c3","F5_EXECUTION_PREDICTION_MISMATCH":"execution_mismatch",
            "F6_OBSTACLE_GEOMETRIC_BLOCK":"obstacle_block","F7_REPOSITION_WORKSPACE_FAILURE":"workspace",
            "F8_OBJECT_STABILITY_TOPPLE":"stability","F10_RUNTIME_NUMERICAL":"runtime",
            "F12_TIMEOUT_BUT_PROGRESSING":"timeout_progressing_class","F13_UNKNOWN_MULTIPLE_CAUSES":"unknown",
            "NONE":"none"}.get(p,"unknown")
audit["family"] = audit.primary_failure_class.map(fam)

srows=[]
for scene, g in audit.groupby("scene"):
    srows.append(dict(scene=scene, n_runs=len(g),
        successes=(g.task_outcome=="SUCCESS").sum(),
        transient=(g.task_outcome=="TRANSIENT_SUCCESS").sum(),
        timeout_progressing=(g.task_outcome=="TIMEOUT_PROGRESSING").sum(),
        true_stalls=(g.task_outcome=="TRUE_STALL").sum(),
        outer_loop=(g.family=="outer_loop").sum(),
        local_c3=(g.family=="local_c3").sum(),
        execution_mismatch=(g.family=="execution_mismatch").sum(),
        obstacle_block=(g.family=="obstacle_block").sum(),
        workspace=(g.family=="workspace").sum(),
        stability=(g.family=="stability").sum(),
        runtime=(g.family=="runtime").sum(),
        unknown=(g.family=="unknown").sum()))
summ = pd.DataFrame(srows)
summ.to_csv(os.path.join(FC, "failure_summary_by_scene.csv"), index=False)

# ---- confusion matrix ----
fam_cols = ["none","outer_loop","local_c3","execution_mismatch","obstacle_block","workspace","timeout_progressing_class"]
cm = pd.crosstab(audit.task_outcome, audit.family).reindex(
    index=["SUCCESS","TRANSIENT_SUCCESS","TIMEOUT_PROGRESSING","TRUE_STALL","RUNTIME_TERMINATION"],
    columns=fam_cols, fill_value=0)
cm.to_csv(os.path.join(FC, "causal_confusion_matrix.csv"))

fig, ax = plt.subplots(figsize=(10, 3.6))
ax.axis("off")
labels = ["NONE","Outer loop\n(F1/F2/F3)","Local C3\n(F4)","Exec mismatch\n(F5)","Obstacle block\n(F6)","Workspace\n(F7)","Timeout-prog\n(F12)"]
tab = ax.table(cellText=cm.values, rowLabels=cm.index, colLabels=labels, loc="center", cellLoc="center")
tab.auto_set_font_size(False); tab.set_fontsize(9); tab.scale(1, 1.7)
for (r_, c_), cell in tab.get_celld().items():
    if r_ == 0 or c_ == -1: cell.set_text_props(weight="bold"); cell.set_facecolor("#e8e8f0")
    elif cm.values[r_-1, c_] > 0: cell.set_facecolor("#dce9f7")
ax.set_title("xArm6 C3+ 600 s campaign: task outcome vs primary causal family (n=30)", pad=18)
fig.tight_layout()
fig.savefig(os.path.join(FC, "causal_confusion_matrix.png"), dpi=160, bbox_inches="tight")

# consistency checks
assert len(audit)==30
assert summ.n_runs.sum()==30 and cm.values.sum()==30
assert (summ[["outer_loop","local_c3","execution_mismatch","obstacle_block","workspace","stability","runtime","unknown"]].sum().sum()
        + (audit.primary_failure_class=="NONE").sum() + (audit.family=="timeout_progressing_class").sum())==30
print(audit.primary_failure_class.value_counts())
print(cm)
