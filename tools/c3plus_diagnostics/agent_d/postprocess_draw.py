#!/usr/bin/env python3
"""Agent D: per-draw postprocessor.

Reads a draw dir (planner.log, state_trace.jsonl, instrumentation CSVs),
computes run metrics + contact-acquisition transactions, appends rows to
runs CSV and contacts CSV.
Usage: postprocess_draw.py DRAW_DIR SCENE GX GY GYAW RUNS_CSV CONTACTS_CSV REPOS_CSV
"""
import sys, os, json, math, csv, re
import numpy as np

draw, scene = sys.argv[1], sys.argv[2]
gx, gy, gyaw = map(float, sys.argv[3:6])
runs_csv, contacts_csv, repos_csv = sys.argv[6:9]

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."

# ---- FK for EE tip via pydrake (franka + EE weld)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
plant = MultibodyPlant(0.0)
parser = Parser(plant)
parser.SetAutoRenaming(True)
parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"))
parser.AddModels(f"{WT}/examples/sampling_c3/urdf/end_effector_full.urdf")
plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_flange"),
                 RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)), [0, 0, 0.107]))
plant.Finalize()
fk_ctx = plant.CreateDefaultContext()
tip_frame = plant.GetFrameByName("end_effector_tip")

def ee_pos(q7):
    plant.SetPositions(fk_ctx, np.array(q7))
    return plant.CalcRelativeTransform(fk_ctx, plant.world_frame(), tip_frame).translation()

# ---- T footprint gap (same sample-point method as the controller)
FOOT = []
for (cx, cy, w, h) in [(0, 0.0099, 0.089, 0.0198), (0, -0.0397, 0.0198, 0.0794)]:
    for i in range(11):
        a = i / 10.0
        FOOT += [(cx - w/2 + a*w, cy - h/2), (cx - w/2 + a*w, cy + h/2),
                 (cx - w/2, cy - h/2 + a*h), (cx + w/2, cy - h/2 + a*h)]

def foot_gap(ex, ey, ox, oy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    d = min(math.hypot(ex - (ox + c*a - s*b), ey - (oy + s*a + c*b)) for a, b in FOOT)
    return d - 0.0195

# scene obstacles (planner discs) for clearance metrics
# true box footprints (cx, cy, hx, hy) for physical clearance
OBS = {"ycb_clutter": [(0.5, 0.0, 0.05, 0.05), (0.37, 0.2, 0.0875, 0.0475)],
       "single_obstacle": [(0.45, 0.0, 0.05, 0.05)], "icra_sign": [], "open_table": []}
obs = OBS.get(scene, [])
POLYS = []
if scene.startswith("icra_faithful"):
    envf = WT + "/results_icra_port/obstacles/obs_polys_env.txt"
    for entry in open(envf).read().strip().split(";"):
        parts = entry.split("|")
        POLYS.append([tuple(map(float, q.split(","))) for q in parts[1:]])
    # C footprint replaces the T footprint for gap/clearance math
    FOOT_C = []
    for (cx, cy, hx, hy) in [(-0.0323, 0.0, 0.016, 0.0515), (0.0, 0.0355, 0.0483, 0.016), (0.0, -0.0355, 0.0483, 0.016)]:
        for i in range(11):
            a = i / 10.0
            FOOT_C += [(cx-hx+2*hx*a, cy-hy), (cx-hx+2*hx*a, cy+hy), (cx-hx, cy-hy+2*hy*a), (cx+hx, cy-hy+2*hy*a)]

def poly_sdf(px, py, verts):
    n = len(verts); best = 1e18; inside = True
    a2 = sum(verts[i][0]*verts[(i+1)%n][1]-verts[(i+1)%n][0]*verts[i][1] for i in range(n))
    vv = verts if a2 > 0 else verts[::-1]
    for i in range(n):
        ax, ay = vv[i]; bx, by = vv[(i+1)%n]
        ex, ey = bx-ax, by-ay
        if ex*(py-ay)-ey*(px-ax) < 0: inside = False
        L2 = ex*ex+ey*ey
        t = max(0.0, min(1.0, ((px-ax)*ex+(py-ay)*ey)/L2)) if L2 > 0 else 0
        best = min(best, math.hypot(px-(ax+t*ex), py-(ay+t*ey)))
    return -best if inside else best

def obs_clear(x, y):
    if POLYS:
        return min(poly_sdf(x, y, v) for v in POLYS)
    best = float("nan")
    for (cx, cy, hx, hy) in obs:
        qx, qy = abs(x-cx)-hx, abs(y-cy)-hy
        d = math.hypot(max(qx,0.0), max(qy,0.0)) if (qx>0 or qy>0) else max(qx,qy)
        best = d if best != best else min(best, d)
    return best

# ---- state trace
recs = []
for line in open(f"{draw}/state_trace.jsonl"):
    try:
        r = json.loads(line)
        if r.get("q"):
            recs.append(r)
    except json.JSONDecodeError:
        pass
if scene.startswith("icra_faithful"):
    FOOT = FOOT_C
    obs = POLYS  # truthy so clearances are computed
ts, gaps, objs, tilts = [], [], [], []
min_pusher_obs, min_obj_obs = float("nan"), float("nan")
for r in recs:
    e = ee_pos(r["q"])
    q = r["obj"]
    yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
    ts.append(r["t"])
    gaps.append(foot_gap(e[0], e[1], q[4], q[5], yaw))
    objs.append((q[4], q[5], yaw))
    tilts.append(r.get("tilt", 0.0))
    if obs:
        pc = obs_clear(e[0], e[1])
        oc = min(obs_clear(q[4] + math.cos(yaw)*a - math.sin(yaw)*b,
                           q[5] + math.sin(yaw)*a + math.cos(yaw)*b) for a, b in FOOT)
        min_pusher_obs = min(min_pusher_obs, pc) if min_pusher_obs == min_pusher_obs else pc
        min_obj_obs = min(min_obj_obs, oc) if min_obj_obs == min_obj_obs else oc

# contact + productivity windows (1 s windows)
in_contact = [g < 0.002 for g in gaps]
contact_frac = sum(in_contact) / max(len(in_contact), 1)
prod_windows = tot_windows = 0
W = 10  # ~1 s at 10 Hz
for i in range(0, len(objs) - W, W):
    tot_windows += 1
    d = math.hypot(objs[i+W][0]-objs[i][0], objs[i+W][1]-objs[i][1])
    if d > 0.005:
        prod_windows += 1

# ---- planner log events
log = open(f"{draw}/planner.log", errors="ignore").read()
n_unprod = log.count("Repositioning after not making progress")
n_reached = log.count("Switching to C3 because reached repositioning target")
n_timeout = len(re.findall(r"timeout", log, re.I))
n_xbox = log.count("Xbox")
abort = ("abort" in log.lower() or "DRAKE_DEMAND" in log or
         "outside" in log.lower() and "workspace" in log.lower())
# success from recorder
suc = None
fin = {}
for line in open(f"{draw}/success.log", errors="ignore"):
    if line.startswith("FINAL"):
        fin = json.loads(line[6:])
suc = fin.get("first_success_t")

# transactions: segment by controller_cycle_costs is_c3_mode
trans_rows = []
ccc = f"{draw}/controller_cycle_costs.csv"
if os.path.exists(ccc):
    import csv as _csv
    cyc = list(_csv.DictReader(open(ccc)))
    seg_start = None
    prev = None
    for row in cyc + [None]:
        mode = row and row["is_c3_mode"] == "1"
        if row and (prev is None or mode != prev):
            if prev is not None and prev and seg_start is not None:
                t0, t1 = seg_start, float(row["time"])
                i0 = next((i for i, t in enumerate(ts) if t >= t0), None)
                i1 = next((i for i, t in enumerate(ts) if t >= t1), len(ts)-1)
                if i0 is not None and i1 > i0:
                    g = min(gaps[i0:i1+1])
                    dobj = math.hypot(objs[i1][0]-objs[i0][0], objs[i1][1]-objs[i0][1])
                    dyaw = abs(math.remainder(objs[i1][2]-objs[i0][2], 2*math.pi))
                    trans_rows.append(dict(
                        scene=scene, draw=os.path.basename(draw), t_start=round(t0, 1),
                        t_end=round(t1, 1), min_gap=round(g, 4),
                        contact_acquired=g < 0.002,
                        obj_dxy=round(dobj, 4), obj_dyaw=round(dyaw, 3),
                        productive=dobj > 0.005))
            seg_start = float(row["time"]) if row else None
            prev = mode
final_pos = fin.get("pos_err"); final_ang = fin.get("ang_err")
run_row = dict(
    scene=scene, draw=os.path.basename(draw),
    success=suc is not None, first_success_t=suc,
    end_t=round(fin.get("t", float("nan")), 1),
    final_xy_err=final_pos and round(final_pos, 4), final_yaw_err=final_ang and round(final_ang, 4),
    best_xy_err=round(fin.get("best_pos_err", float("nan")), 4),
    best_yaw_err=round(fin.get("best_ang_err", float("nan")), 4),
    repositions=n_reached, no_progress_transitions=n_unprod,
    timeouts=n_timeout, workspace_abort=abort,
    contact_frac=round(contact_frac, 3),
    productive_window_frac=round(prod_windows / max(tot_windows, 1), 3),
    max_tilt=round(fin.get("max_tilt", float("nan")), 3),
    min_obj_obstacle=round(min_obj_obs, 4) if min_obj_obs == min_obj_obs else "",
    min_pusher_obstacle=round(min_pusher_obs, 4) if min_pusher_obs == min_pusher_obs else "",
    n_push_transactions=len(trans_rows),
    acq_rate=round(sum(1 for r in trans_rows if r["contact_acquired"]) / max(len(trans_rows), 1), 3),
    prod_rate=round(sum(1 for r in trans_rows if r["productive"]) / max(len(trans_rows), 1), 3),
    prod_given_contact=round(
        sum(1 for r in trans_rows if r["productive"] and r["contact_acquired"]) /
        max(sum(1 for r in trans_rows if r["contact_acquired"]), 1), 3))

def append(path, row_or_rows):
    rows = row_or_rows if isinstance(row_or_rows, list) else [row_or_rows]
    if not rows:
        return
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        if new:
            w.writeheader()
        w.writerows(rows)

append(runs_csv, run_row)
append(contacts_csv, trans_rows)
# reposition events from swept CSV
sw = f"{draw}/reposition_swept_collision.csv"
if os.path.exists(sw):
    rows = list(csv.DictReader(open(sw)))
    ev = [dict(scene=scene, draw=os.path.basename(draw), **{k: r.get(k, "") for k in
          ("time", "event_id", "candidate_id", "ee_x", "ee_y", "rejection_reason")})
          for r in rows]
    append(repos_csv, ev)
print("RUN_ROW", json.dumps(run_row))
