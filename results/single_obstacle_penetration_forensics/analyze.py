#!/usr/bin/env python3
"""Penetration forensics: physical vs planner signed distance, single_obstacle pair01."""
import json, os, csv, math
import numpy as np

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics"
RUN = f"{WT}/results/xarm6_c3plus_scene_smoke/runs/single_obstacle/pair01"
OUT = f"{WT}/results/single_obstacle_penetration_forensics"
T_SDF = f"{WT}/examples/sampling_c3/urdf/push_t_oimscale_m01.sdf"
OBS_SDF = f"{WT}/examples/sampling_c3/urdf/single_obstacle_box_oimframe.sdf"

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion

# ---------- plant ----------
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
parser = Parser(plant)
tmod = parser.AddModels(T_SDF)[0]
omod = parser.AddModels(OBS_SDF)[0]
plant.Finalize()
diagram = builder.Build()
ctx = diagram.CreateDefaultContext()
pctx = plant.GetMyContextFromRoot(ctx)
sg_ctx = scene_graph.GetMyContextFromRoot(ctx)
tbody = plant.GetBodyByName("vertical_link", tmod)
insp = scene_graph.model_inspector()

def geom_name(gid):
    return insp.GetName(gid)

def d_phys_at(quat, xyz):
    q = np.array(quat, float); q /= np.linalg.norm(q)
    plant.SetFreeBodyPose(pctx, tbody, RigidTransform(Quaternion(q), xyz))
    qo = scene_graph.get_query_output_port().Eval(sg_ctx)
    best = None
    for p in qo.ComputeSignedDistancePairwiseClosestPoints(1.0):
        na, nb = geom_name(p.id_A), geom_name(p.id_B)
        cross = ("obstacle" in na) != ("obstacle" in nb)
        if not cross:
            continue
        if best is None or p.distance < best[0]:
            # witness points in world
            XA = insp.GetPoseInFrame(p.id_A)
            pa, pb = p.p_ACa, p.p_BCb
            best = (p.distance, na, nb, pa, pb, p.id_A, p.id_B)
    return best

# witness points come in geometry frames; convert to world for the min pair
def d_phys_world(quat, xyz):
    q = np.array(quat, float); q /= np.linalg.norm(q)
    X = RigidTransform(Quaternion(q), xyz)
    plant.SetFreeBodyPose(pctx, tbody, X)
    qo = scene_graph.get_query_output_port().Eval(sg_ctx)
    best = None
    for p in qo.ComputeSignedDistancePairwiseClosestPoints(1.0):
        na, nb = geom_name(p.id_A), geom_name(p.id_B)
        if ("obstacle" in na) == ("obstacle" in nb):
            continue
        if best is None or p.distance < best["d"]:
            fa = insp.GetFrameId(p.id_A); fb = insp.GetFrameId(p.id_B)
            qov = qo
            Xwa = qov.GetPoseInWorld(fa) @ insp.GetPoseInFrame(p.id_A)
            Xwb = qov.GetPoseInWorld(fb) @ insp.GetPoseInFrame(p.id_B)
            wa = Xwa @ p.p_ACa; wb = Xwb @ p.p_BCb
            best = dict(d=p.distance, gA=na, gB=nb, wa=wa, wb=wb)
    return best

# ---------- planner SDF ----------
BOX = (0.35, 0.0, 0.05, 0.05)
BASE_DISC = (0.0, 0.0, 0.09)
FOOT = [(-0.0445, 0.0198), (-0.0445, 0.0), (-0.0099, 0.0), (-0.0099, -0.0794),
        (0.0099, -0.0794), (0.0099, 0.0), (0.0445, 0.0), (0.0445, 0.0198)]
# edge samples every 2mm like boundary_sample_spacing
FOOT_SAMP = []
for i in range(len(FOOT)):
    a = np.array(FOOT[i]); b = np.array(FOOT[(i + 1) % len(FOOT)])
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / 0.002)))
    for k in range(n):
        FOOT_SAMP.append(a + (b - a) * k / n)
FOOT_SAMP = np.array(FOOT_SAMP)

def box_sdf(wx, wy):
    qx = abs(wx - BOX[0]) - BOX[2]; qy = abs(wy - BOX[1]) - BOX[3]
    if qx > 0 or qy > 0:
        return math.hypot(max(qx, 0.0), max(qy, 0.0))
    return max(qx, qy)

def obs_sdf(wx, wy):
    return min(box_sdf(wx, wy),
               math.hypot(wx - BASE_DISC[0], wy - BASE_DISC[1]) - BASE_DISC[2])

def box_sdf_vec(P):  # P (n,2), box only
    qx = np.abs(P[:, 0] - BOX[0]) - BOX[2]
    qy = np.abs(P[:, 1] - BOX[1]) - BOX[3]
    out = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    inside = (qx <= 0) & (qy <= 0)
    out[inside] = np.maximum(qx, qy)[inside]
    return out

def phi_footprint(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    P = FOOT_SAMP @ R.T + np.array([x, y])
    return float(box_sdf_vec(P).min())

def yaw_of(quat):
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

# ---------- replay trace ----------
rows = [json.loads(l) for l in open(f"{RUN}/state_trace.jsonl")]
recs = []
for r in rows:
    quat = r["obj"][:4]; xyz = r["obj"][4:7]
    b = d_phys_world(quat, xyz)
    yaw = yaw_of(quat)
    recs.append(dict(t=r["t"], x=xyz[0], y=xyz[1], z=xyz[2], yaw=yaw,
                     quat=quat, d_phys=b["d"], gA=b["gA"], gB=b["gB"],
                     wa=b["wa"], wb=b["wb"],
                     phi_c=box_sdf(xyz[0], xyz[1]),
                     phi_f=phi_footprint(xyz[0], xyz[1], yaw)))
with open(f"{OUT}/geometry/physical_signed_distance.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "obj_x", "obj_y", "obj_z", "obj_yaw", "qw", "qx", "qy", "qz",
                "d_phys", "geomA", "geomB",
                "witnessA_x", "witnessA_y", "witnessA_z",
                "witnessB_x", "witnessB_y", "witnessB_z"])
    for r in recs:
        w.writerow([r["t"], r["x"], r["y"], r["z"], r["yaw"], *r["quat"],
                    r["d_phys"], r["gA"], r["gB"], *r["wa"], *r["wb"]])

d = np.array([r["d_phys"] for r in recs]); t = np.array([r["t"] for r in recs])
imin = int(d.argmin())
neg = d < 0
dt_med = float(np.median(np.diff(t)))
bands = dict(b0=(int(((d < 0) & (d >= -0.001)).sum())),
             b1=int(((d < -0.001) & (d >= -0.003)).sum()),
             b3=int(((d < -0.003) & (d >= -0.005)).sum()),
             b5=int((d < -0.005).sum()))
print("TRACE dt_med", dt_med, "n", len(d))
print("min d_phys", d.min(), "at t", t[imin], "pair", recs[imin]["gA"], recs[imin]["gB"])
print("frac<0", neg.mean(), "n<0", neg.sum(), "duration<0 approx s", neg.sum() * dt_med)
print("bands", bands)

# events
def first_idx(cond):
    idx = np.nonzero(cond)[0]
    return int(idx[0]) if len(idx) else None
i0 = first_idx(d < 0.05)
i1 = first_idx(d < 0.002)
i2 = first_idx(d < 0.0)
i3 = imin
# settled: last time object moved > 1mm between samples
xy = np.array([[r["x"], r["y"]] for r in recs])
mv = np.linalg.norm(np.diff(xy, axis=0), axis=1)
moving = np.nonzero(mv > 0.0005)[0]
i4 = int(moving[-1] + 1) if len(moving) else len(recs) - 1
events = dict(T0=i0, T1=i1, T2=i2, T3=i3, T4=i4)
print("events(idx)", events, {k: (t[v] if v is not None else None) for k, v in events.items()})

# metrics for nearest control step
class M0:
    pass
M = M0()
with open(f"{RUN}/xarm6_c3plus_single_obstacle_s01_to_g01_metrics.csv") as f:
    rd = csv.DictReader(f)
    cols = {k: [] for k in ["control_step", "sim_time", "object_x", "object_y",
                            "object_yaw", "min_obstacle_clearance",
                            "physical_contact_active", "pusher_object_gap"]}
    for row in rd:
        for k in cols:
            cols[k].append(float(row[k]))
for k, v in cols.items():
    setattr(M, k, np.array(v))
mst = M.sim_time

with open(f"{OUT}/event_time_alignment.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["event", "trace_row", "trace_t", "video_frame_idx", "video_time_s",
                "nearest_control_step", "metrics_sim_time", "d_phys",
                "phi_planner_center", "phi_planner_footprint"])
    for name, i in events.items():
        if i is None:
            w.writerow([name, "NONE", "", "", "", "", "", "", "", ""]); continue
        fr = i // 2  # render stride 2
        vts = fr / 10.0
        j = int(np.abs(mst - t[i]).argmin())
        w.writerow([name, i, t[i], fr, vts, int(M.control_step[j]), mst[j],
                    d[i], recs[i]["phi_c"], recs[i]["phi_f"]])

# ---------- planner vs physical over metrics rows ----------
# interp d_phys onto metrics sim_time (planner phi computed exactly from metrics x/y/yaw)
phi_c_m = np.array([box_sdf(x, y) for x, y in zip(M.object_x, M.object_y)])
phi_f_m = np.array([phi_footprint(x, y, yw) for x, y, yw in
                    zip(M.object_x, M.object_y, M.object_yaw)])
d_m = np.interp(mst, t, d)
diff_m = phi_f_m - d_m
mvf = M.min_obstacle_clearance - phi_f_m
with open(f"{OUT}/geometry/planner_vs_physical_geometry.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["control_step", "sim_time", "object_x", "object_y", "object_yaw",
                "d_phys_interp", "phi_planner_center", "phi_planner_footprint",
                "metrics_min_obstacle_clearance", "physical_contact_active",
                "pusher_object_gap", "planner_penetration", "difference_m",
                "metrics_minus_phi_f"])
    for i in range(len(mst)):
        w.writerow([int(M.control_step[i]), mst[i], M.object_x[i], M.object_y[i],
                    M.object_yaw[i], d_m[i], phi_c_m[i], phi_f_m[i],
                    M.min_obstacle_clearance[i], M.physical_contact_active[i],
                    M.pusher_object_gap[i], bool(phi_f_m[i] < 0), diff_m[i],
                    mvf[i]])
print("phi_f min", phi_f_m.min(), "at sim_time", mst[phi_f_m.argmin()],
      "phi_c min", phi_c_m.min())
print("planner_penetration count", int((phi_f_m < 0).sum()))
print("metrics clearance min", M.min_obstacle_clearance.min())
print("max |metrics - phi_f|", np.abs(mvf).max(), "median", np.median(np.abs(mvf)))
print("max |phi_f - d_phys|", np.abs(diff_m).max(),
      "median", np.median(np.abs(diff_m)))
np.savez(f"{OUT}/geometry/_cache_metrics.npz", mst=mst, d_m=d_m,
         phi_c_m=phi_c_m, phi_f_m=phi_f_m, mclr=M.min_obstacle_clearance)

# save events + trace arrays for figure stage
np.savez(f"{OUT}/geometry/_cache.npz", t=t, d=d,
         phi_c=np.array([r["phi_c"] for r in recs]),
         phi_f=np.array([r["phi_f"] for r in recs]),
         xy=xy, yaw=np.array([r["yaw"] for r in recs]),
         z=np.array([r["z"] for r in recs]),
         quat=np.array([r["quat"] for r in recs]),
         events=np.array([events[k] if events[k] is not None else -1
                          for k in ["T0", "T1", "T2", "T3", "T4"]]))
print("DONE")
