"""Sections 2-8 of the xArm6 kinematic-trap diagnostic brief. Offline replication."""
import os, csv, json
import numpy as np
import trap_common as tc
from trap_common import Exec, W, LAM, KC, QDOT_MAX

ex = Exec()
OUT = tc.OUT
MATDIR = os.path.join(OUT, "kinematic_trap_matrices")
FIGDIR = os.path.join(OUT, "figures")

# ---------------- Section 1: event detection ----------------
events = []
trial_series = {}
for run, tr in tc.TRIALS:
    rows, st, sq = tc.load_trial(run, tr)
    t = np.array([r["t"] for r in rows])
    err = np.array([np.linalg.norm(r["p_des"] - r["p_tip"]) for r in rows])
    rad = np.array([np.linalg.norm(r["p_des"][:2]) for r in rows])
    trial_series[(run, tr)] = (rows, st, sq, t, err, rad)
    i = 0
    n = len(rows)
    while i < n:
        if err[i] > 0.01 and rad[i] <= 0.68 + 1e-9:
            j = i
            while (j + 1 < n and err[j + 1] > 0.01 and rad[j + 1] <= 0.68 + 1e-9
                   and (err[i] - err[j + 1]) < 0.002):
                j += 1
            if t[j] - t[i] >= 20.0:
                # median q over interval
                mask = (st >= t[i]) & (st <= t[j])
                qmed = np.median(sq[mask], axis=0)
                mid = (i + j) // 2
                events.append(dict(run=run, trial=tr, i0=i, i1=j,
                                   t_start=t[i], t_end=t[j],
                                   p_des=rows[mid]["p_des"], p_tip_log=rows[mid]["p_tip"],
                                   err_start=err[i], err_end=err[j],
                                   q=qmed, radius=rad[mid]))
                i = j + 1
                continue
        i += 1

print(f"{len(events)} events detected")
for e in events:
    print(e["run"], e["trial"], f"{e['t_start']:.0f}-{e['t_end']:.0f}s err {e['err_start']:.4f}->{e['err_end']:.4f}")

# ---------------- Section 2+3: executor replication + SVD ----------------
ev_rows, svd_rows = [], []
for k, e in enumerate(events):
    q, p_des = e["q"], e["p_des"]
    s = ex.step(q, p_des)
    J, v, raw, sat, fac = s["J"], s["v"], s["raw"], s["sat"], s["fac"]
    p_tip = s["p_tip"]
    errv = p_des - p_tip
    errn = np.linalg.norm(errv)
    u_err = errv / errn
    v_pred_sat = J @ sat
    v_pred_raw = J @ raw
    e.update(p_tip_fk=p_tip, errn=errn, u_err=u_err, J=J, v=v, raw=raw, sat=sat, fac=fac,
             v_pred_sat=v_pred_sat, v_pred_raw=v_pred_raw)
    eid = f"E{k:02d}_{e['run']}_t{e['trial']}_{int(e['t_start'])}s"
    e["id"] = eid
    Jw = W @ J
    Uw, Sw, Vtw = np.linalg.svd(Jw)
    Ur, Sr, Vtr = np.linalg.svd(J)
    np.savez(os.path.join(MATDIR, eid + ".npz"), q=q, p_des=p_des, p_tip=p_tip,
             J=J, Jw=Jw, U=Uw, S=Sw, Vt=Vtw, S_raw=Sr, v_task=v, qdot_raw=raw,
             qdot_sat=sat, sat_factor=fac)
    ev_rows.append([e["run"], e["trial"], eid, e["t_start"], e["t_end"],
                    e["t_end"] - e["t_start"], *p_des, *p_tip, errn,
                    e["err_start"] - e["err_end"], *q, e["radius"],
                    *raw, *sat, fac, *v_pred_sat[:3],
                    float(v_pred_sat[:3] @ u_err), float(v_pred_raw[:3] @ u_err),
                    Sw[-1], Sw[0] / Sw[-1]])
    # SVD projection of weighted task
    vw = W @ v
    des_c = Uw.T @ vw                       # desired component per singular dir
    filt = Sw**2 / (Sw**2 + LAM**2)         # damping gain per dir
    ach_c = des_c * filt
    lost_damp = des_c - ach_c
    # loss to saturation per weighted-U direction
    dv_sat = Uw.T @ (W @ (v_pred_raw - v_pred_sat))
    # where does the translation error direction live? project W*[u_err;0;0] onto U
    ue_w = W @ np.concatenate([u_err, [0, 0]])
    ue_c = Uw.T @ ue_w
    small = np.abs(ue_c[-2:]).sum() / np.abs(ue_c).sum()
    for i in range(5):
        svd_rows.append([eid, i + 1, Sw[i], Sr[i], des_c[i], ach_c[i], lost_damp[i],
                         dv_sat[i], ue_c[i]])
    e["sigma_min"] = Sw[-1]
    e["err_in_small_frac"] = small
    svd_rows.append([eid, "err_dir_frac_in_sigma45", "", "", "", "", "", "", small])

hdr = (["run", "trial", "event_id", "t_start", "t_end", "duration_s",
        "p_des_x", "p_des_y", "p_des_z", "p_tip_x", "p_tip_y", "p_tip_z",
        "error_norm", "err_decrease_over_interval",
        "q1", "q2", "q3", "q4", "q5", "planar_radius"]
       + [f"qdot_raw_{i}" for i in range(1, 6)] + [f"qdot_sat_{i}" for i in range(1, 6)]
       + ["sat_factor", "v_pred_x", "v_pred_y", "v_pred_z",
          "v_toward_target_sat", "v_toward_target_raw", "sigma_min_Jw", "cond_Jw"])
with open(os.path.join(OUT, "kinematic_trap_events.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(hdr); w.writerows(ev_rows)
with open(os.path.join(OUT, "trap_svd_decomposition.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["event_id", "sing_index", "sigma_Jw", "sigma_Jraw", "desired_comp_weighted",
                "achievable_after_damping", "lost_to_damping", "lost_to_saturation",
                "err_dir_weighted_comp"])
    w.writerows(svd_rows)

# ---------------- Section 4: reachability / IK classification ----------------
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.solvers import Solve

Q_INIT = np.array([0.888666, 0.258649, -1.158786, 0.00013, 0.900238])


def ik_solve(p_des, seed, tilt):
    ik = InverseKinematics(ex.plant)
    tipf = ex.tip.body_frame()
    ik.AddPositionConstraint(tipf, [0, 0, 0], ex.plant.world_frame(),
                             p_des - 0.002, p_des + 0.002)
    if tilt is not None:
        # tool z-axis within 0.1 rad of vertical (sign given by tilt = +-1)
        ik.AddAngleBetweenVectorsConstraint(ex.plant.world_frame(),
                                            [0, 0, tilt], tipf, [0, 0, 1], 0.0, 0.1)
    prog = ik.prog()
    prog.SetInitialGuess(ik.q(), seed)
    res = Solve(prog)
    return (res.is_success(), res.GetSolution(ik.q()) if res.is_success() else None)


reach_rows = []
for e in events:
    a = e["R"] if "R" in e else None
    _, R = ex.fk(e["q"])
    tilt_sign = 1.0 if (R @ [0, 0, 1])[2] >= 0 else -1.0
    seeds = [e["q"], Q_INIT]
    for jidx in range(3):
        for dp in (0.5, -0.5):
            s = e["q"].copy(); s[jidx] += dp; seeds.append(s)
    exact_ok, exact_seeds = False, 0
    for sd in seeds:
        ok, _ = ik_solve(e["p_des"], sd, tilt_sign)
        exact_seeds += ok
        exact_ok = exact_ok or ok
    pos_ok = any(ik_solve(e["p_des"], sd, None)[0] for sd in seeds)
    exact_from_current = ik_solve(e["p_des"], e["q"], tilt_sign)[0]
    sig_min = e["sigma_min"]
    maxraw = np.max(np.abs(e["raw"]))
    if not pos_ok:
        cls = "K1_infeasible"
    elif not exact_ok:
        cls = "K4_tilt_infeasible"
    elif exact_ok and sig_min < 0.03 and not exact_from_current:
        cls = "K2_posture_trap"
    elif exact_ok and sig_min < 0.03:
        cls = "K2_posture_trap"  # exact IK OK somewhere but local map singular
    elif maxraw > QDOT_MAX and float(e["v_pred_sat"][:3] @ e["u_err"]) > 5e-4:
        cls = "K5_velocity_limited"
    else:
        cls = "K3_controller_stall"
    e["cls"] = cls
    reach_rows.append([e["id"], exact_ok, exact_seeds, pos_ok, exact_from_current,
                       sig_min, maxraw, cls])
with open(os.path.join(OUT, "trap_reachability_classification.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["event_id", "exact_ik_any_seed", "exact_ik_n_seeds_of_8", "position_only_ik",
                "exact_ik_from_current_q", "sigma_min_Jw", "max_abs_qdot_raw", "classification"])
    w.writerows(reach_rows)

# ---------------- Section 5: damping counterfactual ----------------
damp_rows = []
for e in events:
    for lam in (0.005, 0.01, 0.02, 0.05, 0.10, 0.20):
        raw, sat, fac = ex.dls(e["J"], e["v"], lam)
        vp = e["J"] @ sat
        vt = vp[:3]
        speed = float(vt @ e["u_err"])
        cosd = float(vt @ e["u_err"] / (np.linalg.norm(vt) + 1e-12))
        damp_rows.append([e["id"], lam, np.linalg.norm(raw), speed, cosd,
                          np.max(np.abs(raw)), fac])
with open(os.path.join(OUT, "damping_counterfactual.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["event_id", "lambda", "qdot_raw_norm", "speed_toward_target_after_sat",
                "direction_cosine", "max_abs_qdot_raw", "sat_factor"])
    w.writerows(damp_rows)

# ---------------- Section 6: task-row conflict ----------------
def variant_speed(e, rows_idx, Wm):
    J = e["J"][rows_idx, :]
    v = e["v"][rows_idx]
    raw, sat, fac = ex.dls(J, v, LAM, Wm)
    vt = (e["J"][:3, :] @ sat)
    return float(vt @ e["u_err"]), fac

conf_rows, verdicts = [], []
for e in events:
    sA, _ = variant_speed(e, [0, 1, 2, 3, 4], W)
    sB, _ = variant_speed(e, [0, 1, 2], np.eye(3))
    sC, _ = variant_speed(e, [0, 1, 2, 3], np.diag([1, 1, 1, 0.2]))
    sD, _ = variant_speed(e, [0, 1, 2, 3, 4], np.eye(5))
    verdict = (sB > 0.002 and sA < 0.0005)
    verdicts.append(verdict)
    e["speed_A"], e["speed_B"] = sA, sB
    conf_rows.append([e["id"], sA, sB, sC, sD, verdict])
with open(os.path.join(OUT, "task_row_conflict_analysis.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["event_id", "speed_A_5row_W", "speed_B_translation_only",
                "speed_C_trans_plus_wx", "speed_D_unweighted_5row",
                "verdict_B_converges_A_stalls"])
    w.writerows(conf_rows)

# ---------------- Section 7: saturation fidelity ----------------
sat_rows = []
for e in events:
    vt_des = e["v"][:3]
    for tag, qd in (("sat", e["sat"]), ("raw", e["raw"])):
        vt = e["J"][:3, :] @ qd
        cosang = float(vt @ vt_des / (np.linalg.norm(vt) * np.linalg.norm(vt_des) + 1e-12))
        ratio = float(np.linalg.norm(vt) / (np.linalg.norm(vt_des) + 1e-12))
        if tag == "sat":
            row = [e["id"], cosang, ratio]
        else:
            row += [cosang, ratio]
    sat_rows.append(row)
with open(os.path.join(OUT, "saturation_taskspace_effect.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["event_id", "cos_vtask_vs_Jqdot_sat", "mag_ratio_sat",
                "cos_vtask_vs_Jqdot_raw", "mag_ratio_raw"])
    w.writerows(sat_rows)

# ---------------- Kinematic continuation (3 representative events) ----------------
cont_rows = []
reps = ([e for e in events if e["run"] == "r4"][:1]
        + [e for e in events if e["run"] == "r5"])[:3] if events else []
for e in reps:
    for tag, rows_idx, Wm in (("actual_5row_W", [0, 1, 2, 3, 4], W),
                              ("unweighted_5row_D", [0, 1, 2, 3, 4], np.eye(5)),
                              ("translation_only_B", [0, 1, 2], np.eye(3))):
        q = e["q"].copy()
        dt = 0.02
        for _ in range(int(30 / dt)):
            v, p_tip, _ = ex.v_task(q, e["p_des"])
            J = ex.jac(q)
            _, sat, _ = ex.dls(J[rows_idx, :], v[rows_idx], LAM, Wm)
            q = q + dt * sat
        p_end, _ = ex.fk(q)
        fin = np.linalg.norm(e["p_des"] - p_end)
        cont_rows.append([e["id"], tag, e["errn"], fin, fin < 0.002])
with open(os.path.join(OUT, "kinematic_continuation.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["event_id", "variant", "err_initial_m", "err_after_30s_virtual_m",
                "converged_lt_2mm"])
    w.writerows(cont_rows)

# ---------------- Section 8: figures ----------------
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# fig 1: sigma_min vs stall (+ control samples)
ctrl_sig, ctrl_prog = [], []
rng = np.random.default_rng(0)
for (run, tr), (rows, st, sq, t, err, rad) in trial_series.items():
    ev_iv = [(e["t_start"], e["t_end"]) for e in events if (e["run"], e["trial"]) == (run, tr)]
    idxs = rng.choice(len(rows) - 25, size=min(30, len(rows) - 25), replace=False)
    for i in idxs:
        if err[i] < 0.005 or any(a <= t[i] <= b for a, b in ev_iv):
            continue
        q, _ = tc.q_at(st, sq, t[i])
        Jw = W @ ex.jac(q)
        ctrl_sig.append(np.linalg.svd(Jw, compute_uv=False)[-1])
        ctrl_prog.append((err[i] - err[min(i + 20, len(err) - 1)]) / 20.0)
fig, axa = plt.subplots(figsize=(7, 5))
axa.scatter(ctrl_sig, np.array(ctrl_prog) * 1000, c="lightgray", s=18, label="non-trap samples")
if events:
    es = [e["sigma_min"] for e in events]
    ep = [(e["err_start"] - e["err_end"]) / (e["t_end"] - e["t_start"]) * 1000 for e in events]
    sc = axa.scatter(es, ep, c=[e["t_end"] - e["t_start"] for e in events], cmap="viridis", s=60,
                     edgecolor="k", label="trap events (color=duration s)")
    plt.colorbar(sc, label="stall duration [s]")
axa.set_xlabel("sigma_min(Jw)"); axa.set_ylabel("error progress rate [mm/s]")
axa.axvline(0.03, ls="--", c="r", lw=0.8); axa.legend(); axa.set_title("sigma_min vs stall progress")
fig.tight_layout(); fig.savefig(os.path.join(FIGDIR, "sigma_min_vs_stall.png"), dpi=140); plt.close(fig)

# fig 2: desired vs realized speed
fig, axa = plt.subplots(figsize=(7, 5))
cmap = {"K1_infeasible": "tab:red", "K2_posture_trap": "tab:orange",
        "K3_controller_stall": "tab:blue", "K4_tilt_infeasible": "tab:purple",
        "K5_velocity_limited": "tab:green"}
for e in events:
    des = KC * e["errn"]
    real = float(e["v_pred_sat"][:3] @ e["u_err"])
    axa.scatter(des * 1000, real * 1000, c=cmap[e["cls"]], s=50, edgecolor="k")
    axa.annotate(e["id"].split("_", 1)[0], (des * 1000, real * 1000), fontsize=7)
for k, c in cmap.items():
    axa.scatter([], [], c=c, label=k)
lim = axa.get_xlim(); axa.plot([0, lim[1]], [0, lim[1]], "k--", lw=0.7)
axa.set_xlabel("desired speed toward target [mm/s]"); axa.set_ylabel("predicted realized [mm/s]")
axa.legend(fontsize=8); axa.set_title("desired vs realized translation speed at trap events")
fig.tight_layout(); fig.savefig(os.path.join(FIGDIR, "desired_vs_realized_cartesian_velocity.png"), dpi=140); plt.close(fig)

# fig 3: error vs time for r5 trials with events
r5_with = sorted({(e["run"], e["trial"]) for e in events if e["run"] == "r5"})[:3]
if not r5_with:
    r5_with = sorted({(e["run"], e["trial"]) for e in events})[:3]
fig, axs = plt.subplots(len(r5_with) or 1, 1, figsize=(9, 3 * max(1, len(r5_with))), squeeze=False)
for axk, key in zip(axs[:, 0], r5_with):
    rows, st, sq, t, err, rad = trial_series[key]
    axk.plot(t, err * 1000, lw=0.8)
    for e in events:
        if (e["run"], e["trial"]) == key:
            axk.axvspan(e["t_start"], e["t_end"], color="orange", alpha=0.3)
    axk.set_ylabel("|p_des-p_tip| [mm]"); axk.set_title(f"{key[0]} trial {key[1]}")
axs[-1, 0].set_xlabel("t [s]")
fig.tight_layout(); fig.savefig(os.path.join(FIGDIR, "task_error_vs_time.png"), dpi=140); plt.close(fig)

# ---------------- summary ----------------
from collections import Counter
print("\nSUMMARY")
print("events:", len(events), Counter(e["cls"] for e in events))
print("err_dir frac in sigma4/5 subspace:", [round(e["err_in_small_frac"], 3) for e in events])
print("task-row-conflict verdict True on", sum(verdicts), "/", len(verdicts))
print("continuation:")
for r in cont_rows:
    print(" ", r)
