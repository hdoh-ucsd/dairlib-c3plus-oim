"""Derive unchanged offline FK/diagnostic quantities and coordinate saved exports."""
import argparse
import csv
import datetime
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import yaml

from c3plus.configs.paths import REPO
from .exporter import wrap, quat_yaw, project_result, add_result_semantics, write_result_json
from .schema import BLOCKS
from .validation import _validate

WT = str(REPO)

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
        tip_floor = float(cfg.get("tip_floor_z_real", TIP_FLOOR_Z_REAL))
        quad = W_Z_TIP * (100.0 * (max(ez, tip_floor) - tz)) ** 2
        gap = max(tip_floor - ez, 0.0) / TIP_FLOOR_SCALE
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


def diagnostic_cost_parameters(cfg):
    """Weights of this offline evaluator, separate from the native objective."""
    return dict(q_pos=Q_POS, q_theta=Q_THETA, q_ramp_per_step=Q_RAMP_PER_STEP,
                q_ramp_max=Q_RAMP_MAX, w_obstacle=W_OBSTACLE, obstacle_decay=OBSTACLE_DECAY,
                w_support=W_SUPPORT, support_margin=SUPPORT_MARGIN, w_robot_effort=W_EFFORT,
                w_approach=W_APPROACH, r0=APPROACH_R0, w_align=W_ALIGN,
                gamma0_deg=math.degrees(ALIGN_GAMMA0), w_tilt=W_TILT,
                w_z_tip=W_Z_TIP, w_z_tip_exp=W_Z_TIP_EXP, w_contact_z_exp=W_CONTACT_Z_EXP,
                contact_z_slab=CONTACT_Z_SLAB, contact_z_margin=CONTACT_Z_MARGIN,
                shaping_fade_dist=SHAPING_FADE_DIST,
                tip_floor_z=cfg.get("tip_floor_z_real", TIP_FLOOR_Z_REAL),
                tip_floor_scale=TIP_FLOOR_SCALE, exp_arg_max=EXP_ARG_MAX)


def export_existing_result(args, cfg):
    """Enrich only the saved result JSON; never recompute metrics or run dynamics."""
    directory = Path(args.run_dir)
    result_path = directory / f"{args.run_id}_result.json"
    summary = json.loads(result_path.read_text())
    if summary.get("run_id") != args.run_id or summary.get("scenario") != args.scene:
        raise ValueError("Saved result identity does not match the requested run")
    if {"steps_raw", "state_trace", "metrics_csv"} <= (summary.get("recording") or {}).keys():
        # A compact bundle already contains the original dynamic arrays and native
        # settings. Rebuilding metadata from its deleted sidecars would lose them.
        recorded_cfg = summary["provenance"]["evaluation_scene_config"]
        if cfg is not None and cfg != recorded_cfg:
            raise ValueError("Scene configuration differs from the saved compacted run")
        _validate(summary, summary["recording"], recorded_cfg, directory)
        result = (summary if (summary.get("execution") or {}).get("alignment") == "physical_policy_boundaries_v1"
                  else add_result_semantics(summary, directory))
        write_result_json(result_path, result)
        print("WROTE", result_path)
        return
    if cfg is None:
        config_path = directory / "evaluation_scene_config.yaml"
        if not config_path.is_file():
            raise ValueError("Supply --scene-config for a run without saved evaluation geometry")
        cfg = yaml.safe_load(config_path.read_text())
    with (directory / "steps_raw.jsonl").open() as stream:
        steps = [json.loads(line) for line in stream if line.strip()]
    with (directory / f"{args.run_id}_metrics.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    manifest_path = directory / f"{args.run_id}_manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text()) if manifest_path.exists() else {}
    tolerances = manifest.get("tolerances") or {}
    final_path = directory / "recorder.log"
    final = parse_final(final_path) if final_path.exists() else {}
    pos_tol = tolerances.get("pos_tol", final.get("pos_tol"))
    ang_tol = tolerances.get("ang_tol", final.get("ang_tol"))
    evaluation = manifest.get("evaluation") or {}
    costs = evaluation.get("weights", evaluation.get("costs"))
    result = project_result(directory, args.scene, args.run_id, cfg, summary, steps, rows,
                            pos_tol=pos_tol, ang_tol=ang_tol, evaluation_costs=costs)
    write_result_json(result_path, result)
    print("WROTE", result_path)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--scene-config", help="Saved evaluation configuration; required for new postprocessing")
    ap.add_argument("--demo", default="")
    ap.add_argument("--export-only", action="store_true",
                    help="Enrich existing JSON from its embedded data or saved CSV/configs; leave other artifacts untouched")
    args = ap.parse_args(argv)

    cfg = yaml.safe_load(Path(args.scene_config).read_text()) if args.scene_config else None
    if args.export_only:
        export_existing_result(args, cfg)
        return
    if cfg is None:
        ap.error("--scene-config is required unless --export-only is used")
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

    with open(os.path.join(args.run_dir, "steps_raw.jsonl")) as stream:
        steps = [json.loads(line) for line in stream if line.strip()]
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
    object_identity = ({key: cfg[key] for key in ("object_name", "simulation_model", "controller_model",
                                                "object_body_name", "object_channel_substring")}
                       if "object_name" in cfg else {})
    result.update(object_identity)
    result = project_result(args.run_dir, args.scene, args.run_id, cfg, result, steps, rows,
                            pos_tol, ang_tol, diagnostic_cost_parameters(cfg))
    json_path = os.path.join(args.run_dir, f"{args.run_id}_result.json")
    write_result_json(json_path, result)

    # ---------- manifest ----------
    commit = result.get("provenance", {}).get("recorded_runtime", {}).get("commit") or "unknown"
    manifest = {
        "run_id": args.run_id, "scenario": args.scene, "demo": args.demo,
        "goal": goal,
        "files": [os.path.basename(p) for p in (csv_path, json_path)],
        "git_commit": commit,
        "tolerances": {"pos_tol": pos_tol, "ang_tol": ang_tol},
        "evaluation": {"weights": result["evaluation"]["weights"],
                       "costs": result["evaluation"]["weights"],
                       "legacy_fields": {"costs": "Legacy manifest alias of evaluation.weights."},
                       "source": "offline diagnostic evaluator",
                       **{key: result["evaluation"][key] for key in
                          ("thresholds", "ever_success", "final_success", "first_success_t")}},
        "native_controller": result["native_controller"],
        "n_snapshots": result["n_snapshots"], "n_recorded_intervals": result["n_recorded_intervals"],
        "generated": datetime.datetime.now().isoformat(),
    }
    manifest.update(object_identity)
    man_path = os.path.join(args.run_dir, f"{args.run_id}_manifest.yaml")
    with open(man_path, "w") as stream:
        yaml.safe_dump(manifest, stream, sort_keys=False)

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
    print("WROTE", json_path)
    print("WROTE", man_path)
    if not (mono and ok_err and ok_json):
        sys.exit(1)


if __name__ == "__main__":
    main()
