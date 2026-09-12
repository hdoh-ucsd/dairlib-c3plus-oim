#!/usr/bin/env python3
"""§10: align trials around the first reposition request after substantial
rotation; plot yaw, gap, EE tracking, roll/pitch on a common relative timeline.

Usage: first_reposition_alignment.py OUT_PNG DRAW_DIR[:label] [DRAW_DIR:label ...]
Works on any draw dir with state_trace.jsonl + controller_cycle_costs.csv.
FK model selectable: env AGENTD_FK=panda (default) — xarm6 FK needs the xarm6
plant model; for xArm6 runs the recorder's q has 6/7 entries and the EE column
in controller_cycle_costs (sel_ee_x/y) is used for desired, actual from FK if
available else skipped.
"""
import sys, os, json, math, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

out_png = sys.argv[1]
runs = []
for a in sys.argv[2:]:
    if ":" in a:
        d, lab = a.rsplit(":", 1)
    else:
        d, lab = a, os.path.basename(a)
    runs.append((d, lab))

ROT_THRESH = 0.4  # rad of net rotation before the aligned reposition
REL = np.arange(-1.0, 1.01, 0.1)

fig, axes = plt.subplots(4, 1, figsize=(9, 12), sharex=True)
for d, lab in runs:
    recs = []
    for line in open(f"{d}/state_trace.jsonl"):
        try:
            r = json.loads(line)
            if r.get("q"):
                recs.append(r)
        except Exception:
            pass
    ts = np.array([r["t"] for r in recs])
    q = np.array([r["obj"] for r in recs])
    yaw = np.arctan2(2*(q[:, 0]*q[:, 3]+q[:, 1]*q[:, 2]), 1-2*(q[:, 2]**2+q[:, 3]**2))
    yawu = np.unwrap(yaw)
    roll = np.arctan2(2*(q[:, 0]*q[:, 1]+q[:, 2]*q[:, 3]), 1-2*(q[:, 1]**2+q[:, 2]**2))
    pitch = np.arcsin(np.clip(2*(q[:, 0]*q[:, 2]-q[:, 3]*q[:, 1]), -1, 1))
    tilt = np.maximum(np.abs(roll), np.abs(pitch))
    # mode from cycle costs; fallback: FK EE z lift onsets when the CSV is absent
    ccc = f"{d}/controller_cycle_costs.csv"
    if os.path.exists(ccc):
        cyc = list(csv.DictReader(open(ccc)))
    else:
        cyc = []
        import numpy as _np
        from pydrake.multibody.plant import MultibodyPlant
        from pydrake.multibody.parsing import Parser
        from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
        WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
        pl = MultibodyPlant(0.0); pa = Parser(pl); pa.SetAutoRenaming(True)
        if os.environ.get("AGENTD_FK") == "xarm6":
            pa.AddModels("/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant/examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml")
            pa.AddModels(f"{WT}/examples/sampling_c3/urdf/end_effector_full.urdf")
            pl.WeldFrames(pl.GetFrameByName("xarm6_link6" if pl.HasBodyNamed("xarm6_link6") else "link6"),
                          pl.GetFrameByName("end_effector_flange"),
                          RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)), [0, 0, 0.107]))
        else:
            pa.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm.urdf")
            pl.WeldFrames(pl.world_frame(), pl.GetFrameByName("panda_link0"))
            pa.AddModels(f"{WT}/examples/sampling_c3/urdf/end_effector_full.urdf")
            pl.WeldFrames(pl.GetFrameByName("panda_link7"), pl.GetFrameByName("end_effector_flange"),
                          RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)), [0, 0, 0.107]))
        pl.Finalize()
        fctx = pl.CreateDefaultContext()
        tipf = pl.GetFrameByName("end_effector_tip")
        ee = []
        for r in recs:
            qv = _np.array(r["q"], dtype=float)[:pl.num_positions()]
            pl.SetPositions(fctx, qv)
            ee.append(pl.CalcRelativeTransform(fctx, pl.world_frame(), tipf).translation())
        ee = _np.array(ee)
        z0 = _np.median(ee[:, 2])
        lift = ee[:, 2] > z0 + 0.03
        # synthesize cyc rows: mode=repos while lifted, C3 otherwise; sel_ee = actual FK EE
        for i in range(len(ts)):
            cyc.append({"time": str(ts[i]), "is_c3_mode": "0" if lift[i] else "1",
                        "sel_ee_x": str(ee[i, 0]), "sel_ee_y": str(ee[i, 1])})
    tref = None
    for i in range(1, len(cyc)):
        if cyc[i]["is_c3_mode"] == "0" and cyc[i-1]["is_c3_mode"] == "1":
            t = float(cyc[i]["time"])
            j = np.searchsorted(ts, t)
            if j > 0 and abs(yawu[min(j, len(yawu)-1)] - yawu[0]) > ROT_THRESH:
                tref = t
                break
    if tref is None:
        print(f"{lab}: no reposition after rotation > {ROT_THRESH} rad; using first reposition")
        for i in range(1, len(cyc)):
            if cyc[i]["is_c3_mode"] == "0" and cyc[i-1]["is_c3_mode"] == "1":
                tref = float(cyc[i]["time"]); break
    if tref is None:
        print(f"{lab}: no reposition at all; skipped")
        continue
    def at(v, trel):
        return np.interp(tref + trel, ts, v)
    sel = np.array([[float(c["time"]), float(c["sel_ee_x"]), float(c["sel_ee_y"])]
                    for c in cyc])
    axes[0].plot(REL, [at(yawu, r) - at(yawu, 0.0) for r in REL], label=lab)
    # pusher-object planar distance (desired EE vs object)
    dist = []
    for r in REL:
        ex = np.interp(tref + r, sel[:, 0], sel[:, 1])
        ey = np.interp(tref + r, sel[:, 0], sel[:, 2])
        ox = at(q[:, 4], r); oy = at(q[:, 5], r)
        dist.append(math.hypot(ex - ox, ey - oy))
    axes[1].plot(REL, dist, label=lab)
    axes[2].plot(REL, [at(tilt, r) for r in REL], label=lab)
    axes[3].plot(REL, [at(q[:, 6], r) for r in REL], label=lab)
    print(f"{lab}: aligned at t={tref:.1f}s")
for ax, ttl in zip(axes, ["object yaw (rel to t=0)", "desired-EE to object-center distance (m)",
                          "object max(|roll|,|pitch|) (rad)", "object z (m)"]):
    ax.set_title(ttl, fontsize=9); ax.grid(alpha=0.3); ax.legend(fontsize=7)
axes[-1].set_xlabel("t - first reposition request after rotation (s)")
fig.tight_layout()
fig.savefig(out_png, dpi=130)
print("wrote", out_png)
