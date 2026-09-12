#!/usr/bin/env python3
"""Agent D: prediction honesty (section 11).

For every tick where the SELECTED candidate changes (fresh commitment),
compare the ranking rollout's predicted terminal object displacement with the
actual object displacement over the next HORIZON_S seconds, conditioned on
whether physical contact was acquired in that window (gap from state trace FK).
Appends rows to predicted_vs_actual.csv.
Usage: prediction_honesty.py DRAW_DIR SCENE OUT_CSV
"""
import sys, os, json, math, csv
import numpy as np

draw, scene, out_csv = sys.argv[1], sys.argv[2], sys.argv[3]
HORIZON_S = 2.0
WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
XARM6_MODEL = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant/examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml"

def build_fk():
    from pydrake.multibody.plant import MultibodyPlant
    from pydrake.multibody.parsing import Parser
    from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
    plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    if os.environ.get("AGENTD_FK") == "xarm6":
        parser.AddModels(XARM6_MODEL)
        base = "link_base" if plant.HasBodyNamed("link_base") else None
        if base is None:
            for cand in ("xarm6_link_base", "world_link", "base"):
                if plant.HasBodyNamed(cand):
                    base = cand
                    break
        if base is not None:
            try:
                plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(base))
            except Exception:
                pass  # already anchored by the MJCF
        parser.AddModels(f"{WT}/examples/sampling_c3/urdf/end_effector_full.urdf")
        link6 = "xarm6_link6" if plant.HasBodyNamed("xarm6_link6") else "link6"
        plant.WeldFrames(plant.GetFrameByName(link6), plant.GetFrameByName("end_effector_flange"),
                         RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)), [0, 0, 0.107]))
    else:
        parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm.urdf")
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"))
        parser.AddModels(f"{WT}/examples/sampling_c3/urdf/end_effector_full.urdf")
        plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_flange"),
                         RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)), [0, 0, 0.107]))
    plant.Finalize()
    return plant

plant = build_fk()
ctx = plant.CreateDefaultContext()
tip = plant.GetFrameByName("end_effector_tip")

def ee_pos(q7):
    qv = np.array(q7, dtype=float)
    if len(qv) != plant.num_positions():
        qv = qv[:plant.num_positions()]
    plant.SetPositions(ctx, qv)
    return plant.CalcRelativeTransform(ctx, plant.world_frame(), tip).translation()

FOOT = []
for (cx, cy, w, h) in [(0, 0.0099, 0.089, 0.0198), (0, -0.0397, 0.0198, 0.0794)]:
    for i in range(11):
        a = i / 10.0
        FOOT += [(cx-w/2+a*w, cy-h/2), (cx-w/2+a*w, cy+h/2), (cx-w/2, cy-h/2+a*h), (cx+w/2, cy-h/2+a*h)]

recs = []
for line in open(f"{draw}/state_trace.jsonl"):
    try:
        r = json.loads(line)
        if r.get("q"):
            recs.append(r)
    except json.JSONDecodeError:
        pass
ts = [r["t"] for r in recs]
def obj_at(t):
    i = min(range(len(ts)), key=lambda i: abs(ts[i]-t))
    q = recs[i]["obj"]
    yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
    return q[4], q[5], yaw
def min_gap(t0, t1):
    g = 9e9
    for r in recs:
        if t0 <= r["t"] <= t1:
            e = ee_pos(r["q"])
            q = r["obj"]
            yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            c, s = math.cos(yaw), math.sin(yaw)
            d = min(math.hypot(e[0]-(q[4]+c*a-s*b), e[1]-(q[5]+s*a+c*b)) for a, b in FOOT)
            g = min(g, d-0.0195)
    return g

sel = {}
rows_out = []
prev_key = None
for row in csv.DictReader(open(f"{draw}/candidate_ranking_costs.csv")):
    if row["selected"] != "1":
        continue
    t = float(row["time"])
    key = row["candidate_id"]
    if key == prev_key:
        continue
    prev_key = key
    if t + HORIZON_S > (ts[-1] if ts else 0):
        continue
    ox, oy, oyaw = obj_at(t)
    px, py, pyaw = float(row["pred_term_obj_x"]), float(row["pred_term_obj_y"]), float(row["pred_term_obj_yaw"])
    ax, ay, ayaw = obj_at(t + HORIZON_S)
    pred_dxy = math.hypot(px-ox, py-oy)
    act_dxy = math.hypot(ax-ox, ay-oy)
    pred_dyaw = abs(math.remainder(pyaw-oyaw, 2*math.pi))
    act_dyaw = abs(math.remainder(ayaw-oyaw, 2*math.pi))
    g = min_gap(t, t+HORIZON_S)
    rows_out.append(dict(scene=scene, draw=os.path.basename(draw), t=round(t, 1),
                         cand=key, pred_dxy=round(pred_dxy, 4), act_dxy=round(act_dxy, 4),
                         pred_dyaw=round(pred_dyaw, 3), act_dyaw=round(act_dyaw, 3),
                         rho_xy=round(act_dxy/(pred_dxy+1e-4), 3),
                         min_gap=round(g, 4), contact=g < 0.002))
new = not os.path.exists(out_csv)
with open(out_csv, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()) if rows_out else ["scene"])
    if new and rows_out:
        w.writeheader()
    w.writerows(rows_out)
acq = [r for r in rows_out if r["contact"]]
non = [r for r in rows_out if not r["contact"]]
import statistics as st
def med(v):
    return round(st.median(v), 3) if v else None
print(json.dumps(dict(n=len(rows_out), n_contact=len(acq),
    rho_xy_contact=med([r["rho_xy"] for r in acq]),
    rho_xy_noncontact=med([r["rho_xy"] for r in non]),
    pred_dxy_med=med([r["pred_dxy"] for r in rows_out]))))
