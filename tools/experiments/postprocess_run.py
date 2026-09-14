#!/usr/bin/env python3
"""Postprocess an xArm6 C3+ experiment into OIM-comparable metrics.

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
  tip_floor_z_real: float                 # optional world-frame floor for tip centre (m)
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
  RUNID_metrics.csv, RUNID_result.json, RUNID_manifest.yaml
  The result retains summary fields and adds schema/run/hyperparameters/static/
  dynamic sections projected from recorded data. --export-only enriches only
  an existing result using its CSV, raw samples and saved configuration.

Formulas follow the local historical task_diagnostics_definition.md reference,
preserved in results/archive/index.json under source_files at
_provenance/xarm6_c3plus_scene_smoke/metrics/task_diagnostics_definition.md.
NOT_COMPARABLE blocks (pusher_obstacle, robot_contact, admm_penalty) are NaN,
never zero; evaluation_total is a SUBSET sum over the non-NaN mapped blocks.

Usage:
  postprocess_run.py --run-dir DIR --scene SCENE --run-id RUNID \
      --scene-config CONFIG.yaml [--demo NAME]
"""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import numpy as np
import yaml

if __package__:
    from .result_metadata import add_recorded_semantics, build_metadata
else:
    from result_metadata import add_recorded_semantics, build_metadata

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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


def _number(value):
    return None if value is None or value == "" else float(value)


def _differences(values, times, angular=()):
    """Backward estimates at recorded times; never invent an initial velocity."""
    width = len(values[0])
    result = [[None] * width]
    for previous, current, t0, t1 in zip(values, values[1:], times, times[1:]):
        dt = t1 - t0
        row = []
        for j, (a, b) in enumerate(zip(previous, current)):
            if dt <= 0 or a is None or b is None or not math.isfinite(a + b):
                row.append(None)
            else:
                row.append((wrap(b - a) if j in angular else b - a) / dt)
        result.append(row)
    return result


def project_result(run_dir, scene, run_id, cfg, summary, steps, rows,
                   pos_tol=0.05, ang_tol=0.1, evaluation_costs=None, include_semantics=True):
    """Project recorded snapshots into the reference's state/interval layout.

    M observed states define M-1 observed intervals. There is no synthetic t=0
    state and no claim that a recorded effort caused the next sampled state.
    Legacy summary fields retain their meanings; steps_run counts intervals.
    """
    if not rows:
        raise ValueError("Cannot export an empty trajectory")
    if summary.get("run_id") != run_id or summary.get("scenario") != scene:
        raise ValueError("Result run_id/scenario does not match the requested run")
    raw = {}
    for step in steps:
        channels = [ch for ch in step["objects"] if cfg["object_channel_substring"] in ch]
        if not channels:
            continue
        if len(channels) != 1:
            raise ValueError("Ambiguous manipulated object channel")
        k = int(step["control_step"])
        if k in raw:
            raise ValueError("Duplicate raw control_step")
        raw[k] = (step, step["objects"][channels[0]])
    indices = [int(row["control_step"]) for row in rows]
    if len(raw) != len(rows) or set(indices) != set(raw) or any(
            b <= a for a, b in zip(indices, indices[1:])):
        raise ValueError("CSV and raw control steps do not match in increasing order")
    if summary.get("n_control_steps") != len(rows):
        raise ValueError("Summary n_control_steps does not match recorded rows")
    times, poses, poses_3d, robot_q, robot_v, efforts, tips, tilts = [], [], [], [], [], [], [], []
    nq = len(raw[indices[0]][0]["robot_q"])
    for k, row in zip(indices, rows):
        step, pose = raw[k]
        if row.get("run_id", run_id) != run_id or row.get("scenario", scene) != scene:
            raise ValueError("CSV run_id/scenario does not match the requested run")
        t = float(row["sim_time"])
        if not math.isfinite(t) or not math.isclose(t, float(step["sim_time"]), abs_tol=1e-9, rel_tol=0):
            raise ValueError("CSV and raw sim_time do not match")
        planar = [float(row[key]) for key in ("object_x", "object_y", "object_yaw")]
        if len(pose) < 7 or len(step["robot_q"]) != nq:
            raise ValueError("Inconsistent raw pose or robot joint dimensions")
        if not all(math.isfinite(v) for v in [*planar, *pose[:7], *cfg["goal"]]):
            raise ValueError("Nonfinite object pose or goal")
        if (not math.isclose(planar[0], pose[4], abs_tol=1e-9, rel_tol=0)
                or not math.isclose(planar[1], pose[5], abs_tol=1e-9, rel_tol=0)
                or abs(wrap(planar[2] - quat_yaw(pose[:4]))) > 1e-9):
            raise ValueError("CSV and raw object poses do not match")
        for j, key in enumerate(("goal_x", "goal_y", "goal_yaw")):
            if key in row:
                delta = float(row[key]) - cfg["goal"][j]
                if not math.isfinite(delta) or abs(wrap(delta) if j == 2 else delta) > 1e-9:
                    raise ValueError("CSV goal does not match the scene configuration")
        times.append(t)
        poses.append(planar)
        poses_3d.append(list(pose[:7]))
        robot_q.append(list(step["robot_q"]))
        for key, target in (("robot_v", robot_v), ("robot_u", efforts)):
            vector = step.get(key) or [None] * nq
            if len(vector) != nq:
                raise ValueError(f"Inconsistent {key} dimensions")
            target.append(list(vector))
        tips.append([_number(row.get(key)) for key in ("tip_x", "tip_y", "tip_z")])
        roll, pitch = (_number(row.get(key)) for key in ("tip_roll", "tip_pitch"))
        tilts.append(None if roll is None or pitch is None or not math.isfinite(roll + pitch)
                     else math.acos(max(-1.0, min(1.0, -math.cos(roll) * math.cos(pitch)))))

    n = len(rows) - 1
    dt = [b - a for a, b in zip(times, times[1:])]
    observed_span = times[-1] - times[0]
    observed_dt = observed_span / n if n > 0 and math.isfinite(observed_span) and all(d > 0 for d in dt) else None
    object_v = _differences(poses, times, angular=(2,))
    robot_xy = [p[:2] for p in tips]
    robot_xy_v = _differences(robot_xy, times)
    object_z_v = _differences([[p[6]] for p in poses_3d], times)
    metadata = build_metadata(run_dir, scene, run_id, cfg,
                              {**summary, "pos_tol": pos_tol, "ang_tol": ang_tol}, n, observed_dt)
    metadata["provenance"]["trajectory_sources"] = [
        {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        for name in ("steps_raw.jsonl", f"{run_id}_metrics.csv")
        if (path := Path(run_dir) / name).is_file()]
    metadata["provenance"]["evaluation_scene_config"] = cfg
    metadata["evaluation"] = {"weights": evaluation_costs}
    metadata["provenance"]["metadata_semantics"]["diagnostic_weights"] = (
        "Saved offline diagnostic weights; native planner weights are under provenance."
        if evaluation_costs is not None else
        "Historical diagnostic weights were not recorded; no current defaults substituted.")
    metadata["static"].update(
        qpos_size=nq + 4, qvel_size=nq + 4,
        block_qpos_adr=[nq, nq + 1, nq + 2], block_dof_adr=[nq, nq + 1, nq + 2],
        block_z_qpos_adr=nq + 3, block_z_dof_adr=nq + 3,
        state_layout=[f"robot_joint_{i + 1}" for i in range(nq)] + ["object_x", "object_y", "object_yaw", "object_z"])
    schema = {
        "version": "c3plus-reference-projection-v1",
        "indexing": "State arrays have steps_run+1 entries. steps_run is the number of adjacent "
                    "recorded intervals, not executed control actions; n_control_steps counts snapshots. "
                    "Entry 0 is the first observed state, not the initial condition. Interval arrays have "
                    "steps_run entries; tip_z and tip_tilt use state[i+1]. No applied transition control is recorded.",
        "frames": "object_pose=[x,y,theta] and robot_pos=tip [x,y] in world coordinates; "
                  "object_footprint_body is in object coordinates. object_pose_3d=[qw,qx,qy,qz,x,y,z].",
        "units": {"time": "s (simulation)", "compute_time": "s (wall)", "position": "m",
                  "orientation": "rad", "linear_velocity": "m/s", "angular_velocity": "rad/s",
                  "robot_joint_effort": "N m", "contact_force": "N"},
        "sampling": "Asynchronous latest-message snapshots on C3_DEBUG_CURR. Object/robot timestamps "
                    "were not retained individually. Times are preserved, without resampling or a synthetic t=0. "
                    "control_dt is the mean observed state interval when all intervals are positive; use dynamic.time for execution time.",
        "velocities": "object_velocity and robot_vel are backward finite-difference estimates at actual "
                      "sample times (yaw differences wrapped to [-pi,pi)). First estimates and nonpositive-dt "
                      "intervals are null. Repeated cached poses may produce zero despite physical motion.",
        "qpos": "Projected layout in static.state_layout: recorded robot joints, then object [x,y,yaw,z]. "
                "qvel combines recorded robot joint velocities with estimated object [vx,vy,omega,vz]. "
                "These are not Drake's complete generalized coordinates; object_pose_3d preserves quaternion pose.",
        "controls": "robot_control contains null vectors: applied transition controls were not recorded. "
                    "robot_joint_effort preserves published efforts at each snapshot, without action alignment.",
        "plans": "Predicted trajectories and wrench/consensus series were not recorded and are omitted.",
        "missing": {"robot_control": "No aligned control input recording",
                    "compute_time": "No per-step optimization timing recording",
                    "contact_normal_force_z": "No measured contact force recording",
                    "robot_contact_force": "No measured contact force recording",
                    "object_limit_surface_d": "No equivalent native C3+ limit surface parameters",
                    "object_wrench_limit": "No equivalent native C3+ planar wrench limit"},
        "evaluation": "evaluation_costs and diagnostic arrays are state-aligned CSV values; evaluation_total "
                      "sums available terms only. Null denotes unavailable data, including CSV NaN. "
                      "physical_contact_active is a planar proximity flag, not measured contact. "
                      "Legacy success means ever meeting both tolerances, not final-state success.",
    }
    dynamic = dict(time=times, object_pose=poses, object_velocity=object_v,
                   robot_pos=robot_xy, robot_vel=robot_xy_v,
                   robot_control=[[None] * nq for _ in range(n)],
                   qpos=[q + p + [full[6]] for q, p, full in zip(robot_q, poses, poses_3d)],
                   qvel=[q + v + z for q, v, z in zip(robot_v, object_v, object_z_v)],
                   tip_z=[p[2] for p in tips[1:]], tip_tilt=tilts[1:],
                   contact_normal_force_z=[None] * n, robot_contact_force=[None] * n,
                   control_step=indices, object_pose_3d=poses_3d, robot_joint_effort=efforts,
                   tip_z_state=[p[2] for p in tips], tip_tilt_state=tilts,
                   evaluation_costs={key: [_number(row.get(key)) for row in rows] for key in BLOCKS})
    for key in ("position_error_m", "orientation_error_rad", "physical_contact_active",
                "pusher_object_gap", "min_obstacle_clearance", "evaluation_total"):
        dynamic[key] = [_number(row.get(key)) for row in rows]
    result = {**summary, "steps_run": n, "schema": schema, **metadata, "dynamic": dynamic}
    return add_result_semantics(result, run_dir) if include_semantics else result


def add_result_semantics(result, run_dir=None):
    """Upgrade evaluation metadata while preserving recorded states and legacy summary values."""
    dynamic = result["dynamic"]
    times = dynamic["time"]
    if not isinstance(times, list) or not times or not all(
            isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t) for t in times):
        raise ValueError("Evaluation requires nonempty, finite recorded snapshot times")
    count = len(times)
    states = ("time", "object_pose", "object_velocity", "robot_pos", "robot_vel", "qpos", "qvel",
              "control_step", "object_pose_3d", "robot_joint_effort", "tip_z_state", "tip_tilt_state",
              "position_error_m", "orientation_error_rad", "physical_contact_active",
              "pusher_object_gap", "min_obstacle_clearance", "evaluation_total")
    intervals = ("robot_control", "tip_z", "tip_tilt",
                 "contact_normal_force_z", "robot_contact_force")
    if "compute_time" in dynamic:
        timing = dynamic["compute_time"]
        if not isinstance(timing, list) or len(timing) != count - 1:
            raise ValueError(f"dynamic.compute_time must contain {count - 1} recorded entries")
        if all(value is None for value in timing):
            dynamic.pop("compute_time")
        else:
            intervals += ("compute_time",)
    for keys, length in ((states, count), (intervals, count - 1)):
        for key in keys:
            if not isinstance(dynamic.get(key), list) or len(dynamic[key]) != length:
                raise ValueError(f"dynamic.{key} must contain {length} recorded entries")
    costs = dynamic.get("evaluation_costs")
    if not isinstance(costs, dict) or any(not isinstance(v, list) or len(v) != count for v in costs.values()):
        raise ValueError("Diagnostic cost components must align with recorded snapshots")
    for key, expected in (("n_control_steps", count), ("steps_run", count - 1)):
        if key in result and result[key] != expected:
            raise ValueError(f"Legacy {key} does not match the recorded trajectory")

    parameters = result["hyperparameters"]
    observed_span = times[-1] - times[0]
    parameters["control_dt"] = (observed_span / (count - 1) if count > 1 and math.isfinite(observed_span)
                                and all(b > a for a, b in zip(times, times[1:])) else None)
    parameters["control_dt_source"] = "mean_observed_state_interval"
    saved_evaluation = result.get("evaluation") or {}
    weights = saved_evaluation.get("weights", parameters.get("costs"))
    if "weights" in saved_evaluation and "costs" in parameters and parameters["costs"] != weights:
        raise ValueError("Conflicting saved diagnostic weights")
    parameters.pop("costs", None)
    weights_source = parameters.pop("costs_source", None)
    if weights_source is not None:
        result.setdefault("provenance", {}).setdefault("metadata_semantics", {})["diagnostic_weights"] = weights_source
    position, orientation = (parameters.get(key) for key in ("goal_pos_tol", "goal_theta_tol"))
    for value in (position, orientation):
        if value is not None and (not isinstance(value, (int, float)) or isinstance(value, bool)
                                  or not math.isfinite(value) or value < 0):
            raise ValueError("Recorded evaluation thresholds must be finite and nonnegative")
    checks = []
    for pos_error, ang_error in zip(dynamic["position_error_m"], dynamic["orientation_error_rad"]):
        known = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in (position, orientation, pos_error, ang_error))
        checks.append(pos_error < position and ang_error < orientation if known else None)
    ever = True if True in checks else None if None in checks else False
    result.update(n_snapshots=count, n_recorded_intervals=count - 1,
                  evaluation={"ever_success": ever, "final_success": checks[-1],
                              "first_success_t": next((t for t, ok in zip(times, checks) if ok), None),
                              "thresholds": {"position_m": position, "orientation_rad": orientation},
                              "weights": weights,
                              "costs": {"components": costs, "total": dynamic["evaluation_total"]},
                              "success_semantics": "Simultaneous strict < comparisons on retained snapshots; unknown thresholds/errors remain null.",
                              "cost_semantics": "Offline diagnostic evaluation, not the native C3+ optimization objective; total sums available components only."})
    schema = result.setdefault("schema", {})
    schema.update(semantics_version=3,
                  indexing="n_snapshots counts retained asynchronous states; n_recorded_intervals is n_snapshots-1. "
                           "Neither counts executed controls. Entry 0 is the first observed state; no initial state is invented. "
                           "Interval tip_z/tip_tilt use state[i+1]. Keep the actual dynamic.time.",
                  state_arrays=list(states), interval_arrays=list(intervals),
                  legacy_fields={
                      "success": "Legacy common-evaluation ever-success, not native C3+ completion; use evaluation.ever_success.",
                      "t_success": "Legacy first successful retained snapshot time; use evaluation.first_success_t.",
                      "first_success_t": "Legacy common-evaluation event time from recorder object messages; may differ from the retained-snapshot evaluation.first_success_t.",
                      "n_control_steps": "Legacy retained-snapshot count; use n_snapshots. Not executed control actions.",
                      "steps_run": "Legacy adjacent recorded interval count; use n_recorded_intervals.",
                      "hyperparameters.steps": "Recorded interval count, not an executed action count or requested budget.",
                      "hyperparameters.goal_pos_tol": "Common evaluation position threshold; native thresholds are under native_controller.",
                      "hyperparameters.goal_theta_tol": "Common evaluation orientation threshold; native thresholds are under native_controller.",
                      "hyperparameters.costs": "Legacy diagnostic weights were relocated to evaluation.weights; not native controller weights.",
                      "hyperparameters.c3plus": "Legacy native configuration was relocated to provenance.c3plus and verified provenance.configuration snapshots.",
                      "dynamic.evaluation_costs": "Alias of evaluation.costs.components: offline diagnostic component series.",
                      "dynamic.evaluation_total": "Alias of evaluation.costs.total: sum of available offline components."})
    schema.setdefault("missing", {}).update(
        predicted_plans="Predicted object/robot plans were not recorded.",
        wrench_consensus_trajectories="Wrench and consensus trajectories were not recorded.")
    if "compute_time" in dynamic:
        schema["missing"].pop("compute_time", None)
    else:
        schema["missing"]["compute_time"] = "No per-step optimization timing recording; unavailable array omitted."
    schema["sampling"] = ("Asynchronous latest-message snapshots on C3_DEBUG_CURR; dynamic.time is unchanged. "
                          "Individual object/robot timestamps were not retained; no resampling or initial state is invented. "
                          "Execution time is the observed last-minus-first timestamp. control_dt is its mean per "
                          "recorded interval when all intervals are positive, not a physics or planner timestep.")
    result.setdefault("provenance", {}).setdefault("metadata_semantics", {})["control_dt"] = (
        "Mean observed state interval: (dynamic.time[-1]-dynamic.time[0])/steps_run; "
        "null unless all observed intervals are positive. Not physics or planner dt.")
    return add_recorded_semantics(result, run_dir)


def _json_values(value):
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, dict):
        return {key: _json_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_values(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    return None if isinstance(value, float) and not math.isfinite(value) else value


def write_result_json(path, result):
    """Serialize before replacing the result, with strict JSON nulls for missing data."""
    path = Path(path)
    text = json.dumps(_json_values(result), indent=2, allow_nan=False) + "\n"
    expected = json.loads(text)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        if json.loads(temporary.read_text()) != expected:
            raise ValueError("Result JSON failed read-back verification")
        if path.exists():
            os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def export_existing_result(args, cfg):
    """Enrich only the saved result JSON; never recompute metrics or run dynamics."""
    directory = Path(args.run_dir)
    result_path = directory / f"{args.run_id}_result.json"
    summary = json.loads(result_path.read_text())
    if summary.get("run_id") != args.run_id or summary.get("scenario") != args.scene:
        raise ValueError("Saved result identity does not match the requested run")
    if "recording" in summary:
        # A compact bundle already contains the original dynamic arrays and native
        # settings. Rebuilding metadata from its deleted sidecars would lose them.
        recorded_cfg = summary["provenance"]["evaluation_scene_config"]
        if cfg is not None and cfg != recorded_cfg:
            raise ValueError("Scene configuration differs from the saved compacted run")
        if __package__:
            from .run_artifacts import _validate
        else:
            from run_artifacts import _validate
        _validate(summary, summary["recording"], recorded_cfg, directory)
        result = add_result_semantics(summary, directory)
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--scene-config", help="Saved evaluation configuration; required for new postprocessing")
    ap.add_argument("--demo", default="")
    ap.add_argument("--export-only", action="store_true",
                    help="Enrich existing JSON from its embedded data or saved CSV/configs; leave other artifacts untouched")
    args = ap.parse_args()

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
