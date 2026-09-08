#!/usr/bin/env python3
"""Postprocess an xArm6 C3+ scene-smoke run into OIM-comparable metrics.

Inputs (in --run-dir):
  steps_raw.jsonl  one JSON line per C3+ control step (record_metrics.py):
                   {control_step, sim_time, robot_q (5 xArm6 joints), robot_v,
                    robot_u, objects: {channel: [qw,qx,qy,qz,x,y,z,...]}}
  recorder.log     contains a "FINAL {json}" line with first_success_t,
                   best_pos_err, best_ang_err, pos_tol, ang_tol.

Scene config YAML schema (--scene-config):
  goal: [x, y, yaw]                       # world frame, radians
  object_channel_substring: str           # picks the manipulated object channel
  footprint: [[x,y], ...]                 # object-frame outline polygon (m),
                                          # CCW, closed implicitly
  pusher_radius: float                    # tip sphere radius (m)
  block_half_height: float                # object half height (m)
  tip_target_z: float                     # OIM tip target z = block mid-height
  obstacles:                              # optional; both lists optional
    polygons: [ [[x,y],...], ... ]        # world-frame convex polygons
    discs: [ [x,y,r], ... ]
  table:                                  # optional; enables the support block
    center: [x, y]
    half_extents: [hx, hy]
  tip_floor_branch: real                  # "real" (tip_floor_z=0.012) or "sim"
  control_dt: 0.1                         # fallback dt if sim_time is flat
  boundary_sample_spacing: 0.002          # footprint boundary sampling (m)

Outputs into --run-dir:
  RUNID_metrics.csv, RUNID_eval_metrics.png, RUNID_result.json,
  RUNID_manifest.yaml

Formulas follow results/xarm6_c3plus_scene_smoke/metrics/
task_diagnostics_definition.md and c3plus_robot_block_cost_mapping.csv.
NOT_COMPARABLE blocks (pusher_obstacle, robot_contact, admm_penalty) are NaN,
never zero; evaluation_total is a SUBSET sum over the non-NaN mapped blocks.

Usage:
  postprocess_run.py --run-dir DIR --scene SCENE --run-id RUNID \
      --scene-config CONFIG.yaml [--demo NAME]
"""
import argparse
import csv
import datetime
import json
import math
import os
import subprocess
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics"

# OIM xarm6 weights (task_diagnostics_definition.md)
Q_POS, Q_THETA = 200.0, 16.0
Q_RAMP_PER_STEP, Q_RAMP_MAX = 0.005, 30.0
W_OBSTACLE, OBSTACLE_DECAY = 50.0, 0.10
W_SUPPORT, SUPPORT_MARGIN = 500.0, 0.2
W_EFFORT = 0.05
W_APPROACH, APPROACH_R0 = 60.0, 0.075
W_ALIGN, ALIGN_GAMMA0 = 50.0, math.radians(60.0)
W_TILT = 80.0
W_Z_TIP, W_Z_TIP_EXP = 1.0, 1.0
TIP_FLOOR_Z_REAL, TIP_FLOOR_SCALE = 0.012, 0.004
W_CONTACT_Z_EXP, CONTACT_Z_SLAB, CONTACT_Z_MARGIN = 200.0, 0.015, 0.008
SHAPING_FADE_DIST = 0.25
EXP_ARG_MAX = 10.0

BLOCKS = ["goal_pos", "goal_theta", "obstacle", "support", "approach",
          "align", "tilt", "tip_z_cost", "contact_z", "pusher_obstacle",
          "robot_contact", "effort", "admm_penalty"]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(q):  # [w,x,y,z]
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# ---------- 2D geometry ----------
def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-16), 0.0, 1.0)
    return np.linalg.norm(p - (a + t * ab))


def point_in_poly(p, verts):
    x, y = p
    inside = False
    n = len(verts)
    for i in range(n):
        x1, y1 = verts[i]
        x2, y2 = verts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xin:
                inside = not inside
    return inside


def poly_signed_dist(p, verts):
    """Signed distance to polygon boundary: negative inside."""
    p = np.asarray(p, float)
    v = np.asarray(verts, float)
    d = min(seg_dist(p, v[i], v[(i + 1) % len(v)]) for i in range(len(v)))
    return -d if point_in_poly(p, verts) else d


def sample_boundary(verts, spacing):
    v = np.asarray(verts, float)
    pts = []
    for i in range(len(v)):
        a, b = v[i], v[(i + 1) % len(v)]
        L = np.linalg.norm(b - a)
        n = max(int(math.ceil(L / spacing)), 1)
        for k in range(n):
            pts.append(a + (b - a) * (k / n))
    return np.asarray(pts)


class Obstacles:
    def __init__(self, cfg):
        obs = cfg.get("obstacles") or {}
        self.polys = [np.asarray(p, float) for p in (obs.get("polygons") or [])]
        self.discs = [tuple(d) for d in (obs.get("discs") or [])]

    def empty(self):
        return not self.polys and not self.discs

    def sdf(self, p):
        ds = [poly_signed_dist(p, poly) for poly in self.polys]
        ds += [math.hypot(p[0] - cx, p[1] - cy) - r for cx, cy, r in self.discs]
        return min(ds) if ds else float("inf")


def table_sdf(p, center, half):
    """Signed distance to table box (negative inside)."""
    q = np.abs(np.asarray(p) - np.asarray(center)) - np.asarray(half)
    outside = np.linalg.norm(np.maximum(q, 0.0))
    return outside + min(max(q[0], q[1]), 0.0)


# ---------- FK ----------
def build_fk():
    from pydrake.multibody.plant import MultibodyPlant
    from pydrake.multibody.parsing import Parser
    from pydrake.math import RigidTransform, RollPitchYaw
    plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    arm, = parser.AddModels(os.path.join(
        WT, "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml"))
    ee, = parser.AddModels(os.path.join(
        WT, "examples/sampling_c3/urdf/end_effector_xarm6_stick.urdf"))
    plant.WeldFrames(
        plant.GetFrameByName("xarm6_link6", arm),
        plant.GetFrameByName("end_effector_flange", ee),
        RigidTransform())  # OIM stick: flush at link6, matches the sim weld
    plant.Finalize()
    ctx = plant.CreateDefaultContext()
    tip = plant.GetBodyByName("end_effector_tip", ee)
    nq = plant.num_positions()

    def fk(q_joints):
        q = np.zeros(nq)
        q[:len(q_joints)] = q_joints
        plant.SetPositions(ctx, q)
        X = plant.EvalBodyPoseInWorld(ctx, tip)
        from pydrake.math import RollPitchYaw as RPY
        rpy = RPY(X.rotation()).vector()
        return X.translation(), rpy, X.rotation().matrix()
    return fk


# ---------- per-step block costs ----------
def compute_row(k, x, y, yaw, ex, ey, ez, R_tip, goal, cfg, geo, q, q_prev, dt):
    gx, gy, gyaw = goal
    pos_err = math.hypot(x - gx, y - gy)
    theta_err = abs(wrap(yaw - gyaw))
    ramp = min(1.0 + Q_RAMP_PER_STEP * k, Q_RAMP_MAX)
    fade = min(max(pos_err / SHAPING_FADE_DIST, 0.0), 1.0)
    nan = float("nan")
    b = {}
    b["goal_pos"] = ramp * Q_POS * ((x - gx) ** 2 + (y - gy) ** 2)
    b["goal_theta"] = ramp * Q_THETA * wrap(yaw - gyaw) ** 2

    # world-frame footprint boundary samples
    c, s = math.cos(yaw), math.sin(yaw)
    Rm = np.array([[c, -s], [s, c]])
    world_pts = geo["boundary"] @ Rm.T + np.array([x, y])

    obstacles = geo["obstacles"]
    if obstacles.empty():
        b["obstacle"] = 0.0
        min_clear = nan
    else:
        ds = np.array([obstacles.sdf(p) for p in world_pts])
        b["obstacle"] = W_OBSTACLE * float(np.sum(np.exp(-ds / OBSTACLE_DECAY)))
        min_clear = float(np.min(ds))

    if geo["table"] is None:
        b["support"] = nan
    else:
        ctr, half = geo["table"]
        vals = [max(table_sdf(p, ctr, half) + SUPPORT_MARGIN, 0.0) ** 2
                for p in world_pts]
        b["support"] = W_SUPPORT * float(np.sum(vals))

    d2 = (ex - x) ** 2 + (ey - y) ** 2
    b["approach"] = fade * W_APPROACH * max(d2 - APPROACH_R0 ** 2, 0.0)

    v1 = np.array([x - ex, y - ey])
    v2 = np.array([gx - x, gy - y])
    cosang = float(np.dot(v1, v2)) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
    align = fade * W_ALIGN * max(math.cos(ALIGN_GAMMA0) - cosang, 0.0)
    local = Rm.T @ np.array([ex - x, ey - y])
    sdf_fp = poly_signed_dist(local, geo["footprint"])
    top_z = cfg["tip_target_z"] + cfg["block_half_height"]
    if sdf_fp <= 0.0 and (top_z - 0.005) <= ez <= (top_z + 0.05):
        align = 0.0
    b["align"] = align

    # tilt: tool "down" axis = tip frame +z in world (flange welded with
    # roll=pi, so the nominal tool z points to world -z); psi vs world -z.
    tool_down = R_tip @ np.array([0.0, 0.0, 1.0])
    cospsi = float(np.clip(np.dot(tool_down, [0, 0, -1.0]), -1.0, 1.0))
    b["tilt"] = fade * W_TILT * (1.0 - cospsi)

    tz = cfg["tip_target_z"]
    if cfg.get("tip_floor_branch", "real") == "real":
        quad = W_Z_TIP * (100.0 * (max(ez, TIP_FLOOR_Z_REAL) - tz)) ** 2
        gap = max(TIP_FLOOR_Z_REAL - ez, 0.0) / TIP_FLOOR_SCALE
        b["tip_z_cost"] = quad + W_Z_TIP_EXP * (math.exp(min(gap ** 2, EXP_ARG_MAX)) - 1.0)
    else:
        if ez >= tz:
            b["tip_z_cost"] = fade * W_Z_TIP * (100.0 * (ez - tz)) ** 2
        else:
            b["tip_z_cost"] = math.exp(min((100.0 * (tz - ez)) ** 2, EXP_ARG_MAX))

    dz = ez - top_z
    if sdf_fp <= CONTACT_Z_MARGIN and -CONTACT_Z_SLAB <= dz <= CONTACT_Z_SLAB:
        g = 1.0 - min(abs(dz) / CONTACT_Z_SLAB, 1.0)
        b["contact_z"] = W_CONTACT_Z_EXP * math.exp((2.0 * g) ** 2)
    else:
        b["contact_z"] = 0.0

    if q_prev is None or dt <= 0:
        b["effort"] = nan
    else:
        qdot = (np.asarray(q) - np.asarray(q_prev)) / dt
        b["effort"] = fade * W_EFFORT * float(np.sum(qdot ** 2))

    b["pusher_obstacle"] = nan
    b["robot_contact"] = nan
    b["admm_penalty"] = nan

    # pusher-object gap (signed, before radius clamp for the flag)
    gap2d = poly_signed_dist([ex, ey], world_pts_polygon(geo["footprint"], Rm, x, y))
    signed_gap = gap2d - cfg["pusher_radius"]
    contact = max(signed_gap, 0.0) < 0.002

    total = sum(v for v in b.values() if not math.isnan(v))
    return pos_err, theta_err, b, total, signed_gap, contact, min_clear


def world_pts_polygon(footprint, Rm, x, y):
    return (np.asarray(footprint, float) @ Rm.T + np.array([x, y])).tolist()


def parse_final(log_path):
    final = None
    with open(log_path) as f:
        for line in f:
            if line.startswith("FINAL "):
                final = json.loads(line[6:])
    return final or {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--scene-config", required=True)
    ap.add_argument("--demo", default="")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.scene_config))
    goal = [float(v) for v in cfg["goal"]]
    sub = cfg["object_channel_substring"]
    spacing = float(cfg.get("boundary_sample_spacing", 0.002))
    footprint = np.asarray(cfg["footprint"], float)
    geo = {
        "footprint": footprint,
        "boundary": sample_boundary(footprint, spacing),
        "obstacles": Obstacles(cfg),
        "table": ((cfg["table"]["center"], cfg["table"]["half_extents"])
                  if cfg.get("table") else None),
    }

    steps = [json.loads(l) for l in
             open(os.path.join(args.run_dir, "steps_raw.jsonl")) if l.strip()]
    if not steps:
        sys.exit("no steps in steps_raw.jsonl")
    final = parse_final(os.path.join(args.run_dir, "recorder.log"))
    pos_tol = float(final.get("pos_tol", 0.05))
    ang_tol = float(final.get("ang_tol", 0.1))

    fk = build_fk()
    fallback_dt = float(cfg.get("control_dt", 0.1))

    rows = []
    q_prev, t_prev = None, None
    for st in steps:
        ch = next((c for c in st["objects"] if sub in c), None)
        if ch is None:
            continue
        pv = st["objects"][ch]
        quat, x, y = pv[:4], pv[4], pv[5]
        yaw = quat_yaw(quat)
        tip_p, tip_rpy, R_tip = fk(st["robot_q"])
        t = float(st["sim_time"])
        dt = (t - t_prev) if (t_prev is not None and t > t_prev) else fallback_dt
        k = int(st["control_step"])
        pe, te, b, total, sgap, contact, mclear = compute_row(
            k, x, y, yaw, tip_p[0], tip_p[1], tip_p[2], R_tip, goal, cfg, geo,
            st["robot_q"], q_prev, dt)
        rows.append(dict(
            run_id=args.run_id, scenario=args.scene, control_step=k,
            sim_time=t, object_x=x, object_y=y, object_yaw=yaw,
            goal_x=goal[0], goal_y=goal[1], goal_yaw=goal[2],
            position_error_m=pe, orientation_error_rad=te,
            tip_x=tip_p[0], tip_y=tip_p[1], tip_z=tip_p[2],
            tip_roll=tip_rpy[0], tip_pitch=tip_rpy[1], tip_yaw=tip_rpy[2],
            physical_contact_active=int(contact), pusher_object_gap=sgap,
            min_obstacle_clearance=mclear,
            **b, evaluation_total=total))
        q_prev, t_prev = st["robot_q"], t

    if not rows:
        sys.exit("no rows matched object_channel_substring=%s" % sub)

    cols = ["run_id", "scenario", "control_step", "sim_time", "object_x",
            "object_y", "object_yaw", "goal_x", "goal_y", "goal_yaw",
            "position_error_m", "orientation_error_rad", "tip_x", "tip_y",
            "tip_z", "tip_roll", "tip_pitch", "tip_yaw",
            "physical_contact_active", "pusher_object_gap",
            "min_obstacle_clearance"] + BLOCKS + ["evaluation_total"]
    csv_path = os.path.join(args.run_dir, f"{args.run_id}_metrics.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # ---------- plot ----------
    ks = [r["control_step"] for r in rows]
    fig, (axl, axr) = plt.subplots(1, 2, figsize=(14, 5))
    axl.plot(ks, [r["position_error_m"] for r in rows], label="position error (m)")
    axl.plot(ks, [r["orientation_error_rad"] for r in rows],
             label="orientation error (rad)")
    axl.set_title("Task diagnostics")
    axl.set_xlabel("control step")
    axl.legend()
    axl.grid(alpha=0.3)

    active = [blk for blk in BLOCKS
              if any(not math.isnan(r[blk]) for r in rows)
              and np.nansum([r[blk] for r in rows]) != 0.0]
    for blk in active:
        vals = [r[blk] for r in rows]
        axr.plot(ks, vals, lw=1, alpha=0.8,
                 label=f"{blk} (Σ {np.nansum(vals):.3g})")
    axr.plot(ks, [r["evaluation_total"] for r in rows], "k-", lw=2.5,
             label="total (subset: %s)" % "+".join(
                 blk for blk in BLOCKS
                 if any(not math.isnan(r[blk]) for r in rows)))
    axr.set_yscale("symlog", linthresh=1e-3)
    axr.set_title("Robot block costs")
    axr.set_xlabel("control step")
    axr.legend(fontsize=6, loc="upper right")
    axr.grid(alpha=0.3)
    fig.suptitle(f"{args.run_id} ({args.scene})")
    fig.tight_layout()
    png_path = os.path.join(args.run_dir, f"{args.run_id}_eval_metrics.png")
    fig.savefig(png_path, dpi=140)
    plt.close(fig)

    # ---------- result json ----------
    succ_rows = [r for r in rows if r["position_error_m"] < pos_tol
                 and r["orientation_error_rad"] < ang_tol]
    last = rows[-1]
    result = {
        "run_id": args.run_id, "scenario": args.scene,
        "success": bool(succ_rows),
        "t_success": (succ_rows[0]["sim_time"] if succ_rows else None),
        "first_success_t": final.get("first_success_t"),
        "final_position_error": last["position_error_m"],
        "final_orientation_error": last["orientation_error_rad"],
        "best_pos_err": final.get("best_pos_err"),
        "best_ang_err": final.get("best_ang_err"),
        "n_control_steps": len(rows),
        "sim_time_end": last["sim_time"],
    }
    json_path = os.path.join(args.run_dir, f"{args.run_id}_result.json")
    json.dump(result, open(json_path, "w"), indent=2)

    # ---------- manifest ----------
    try:
        commit = subprocess.check_output(
            ["git", "-C", WT, "rev-parse", "HEAD"]).decode().strip()
    except Exception:
        commit = "unknown"
    manifest = {
        "run_id": args.run_id, "scenario": args.scene, "demo": args.demo,
        "goal": goal,
        "files": [os.path.basename(p) for p in (csv_path, png_path, json_path)],
        "git_commit": commit,
        "tolerances": {"pos_tol": pos_tol, "ang_tol": ang_tol},
        "generated": datetime.datetime.now().isoformat(),
    }
    man_path = os.path.join(args.run_dir, f"{args.run_id}_manifest.yaml")
    yaml.safe_dump(manifest, open(man_path, "w"), sort_keys=False)

    # ---------- validation ----------
    mono = all(b["control_step"] > a["control_step"]
               for a, b in zip(rows, rows[1:]))
    print(f"[VALIDATE] control_step strictly monotonic: "
          f"{'PASS' if mono else 'FAIL'}")
    pe_i = math.hypot(last["object_x"] - goal[0], last["object_y"] - goal[1])
    te_i = abs(wrap(last["object_yaw"] - goal[2]))
    ok_err = (abs(pe_i - last["position_error_m"]) < 1e-9
              and abs(te_i - last["orientation_error_rad"]) < 1e-9)
    print(f"[VALIDATE] final-row errors reproduce independently: "
          f"{'PASS' if ok_err else 'FAIL'}")
    ok_json = (result["final_position_error"] == last["position_error_m"]
               and result["final_orientation_error"] ==
               last["orientation_error_rad"])
    print(f"[VALIDATE] result JSON final errors equal CSV final row: "
          f"{'PASS' if ok_json else 'FAIL'}")
    print("WROTE", csv_path)
    print("WROTE", png_path)
    print("WROTE", json_path)
    print("WROTE", man_path)
    if not (mono and ok_err and ok_json):
        sys.exit(1)


if __name__ == "__main__":
    main()
