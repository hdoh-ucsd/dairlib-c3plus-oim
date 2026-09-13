#!/usr/bin/env python3
"""Shared machinery for the baseline-vs-ReLU 5x5 comparison pipeline.

Offline, evaluation-only. Reads run dirs under
results/c3plus_relu_chomp_comparison/<variant>/<scene>/sMMgNN/ and scene
geometry from tools/scene_smoke/scene_configs/<scene>.yaml.
"""
import json
import math
import os
import re

import numpy as np
import yaml

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT = os.environ.get("C3PLUS_RESULTS_ROOT", os.path.join(WT, "results/c3plus_relu_chomp_comparison"))
SCFG = os.path.join(WT, "tools/scene_smoke/scene_configs")
SCENES = ["open_task", "single_obstacle", "shelf_gap", "ycb_clutter",
          "icra_sign", "slalom"]
VARIANTS = ["baseline", "relu"]

FAM = {
    "open_task": "matched_open_table_xarm6_",
    "single_obstacle": "matched_single_obstacle_xarm6_",
    "shelf_gap": "matched_shelf_gap_xarm6_",
    "ycb_clutter": "matched_ycb_clutter_xarm6_",
    "slalom": "matched_slalom_xarm6_",
    "icra_sign": "anything_icra_c_matched_xarm6_",
}

# CHOMP body-point spacing along the footprint boundary (object frame).
BODY_POINT_SPACING = 0.005  # 5 mm
CHOMP_EPS = 0.01            # m
TILT_THRESH = 0.1           # rad, |roll| or |pitch| above this = non-planar row


def load_scene(scene):
    with open(os.path.join(SCFG, scene + ".yaml")) as f:
        cfg = yaml.safe_load(f)
    obs = cfg.get("obstacles") or {}
    return {
        "footprint": [tuple(map(float, v)) for v in cfg["footprint"]],
        "discs": [tuple(map(float, d)) for d in (obs.get("discs") or [])],
        "polys": [[tuple(map(float, v)) for v in p]
                  for p in (obs.get("polygons") or [])],
        "goal": [float(v) for v in cfg.get("goal", [0, 0, 0])],
        "pusher_radius": float(cfg.get("pusher_radius", 0.0)),
    }


def goal_of(scene, n):
    """Per-goal-index goal pose from the demo's goal_params.yaml (same rule as
    run_grid_campaign.py)."""
    gp = os.path.join(WT, "examples/sampling_c3", FAM[scene] + f"t{n}",
                      "parameters/goal_params.yaml")
    with open(gp) as stream:
        txt = stream.read()
    pos = re.search(r"^fixed_target_position: \[([^\]]+)\]", txt, re.M).group(1)
    quat = re.search(r"^fixed_target_orientation: \[([^\]]+)\]", txt,
                     re.M).group(1)
    x, y, _ = [float(v) for v in pos.split(",")]
    w, _, _, z = [float(v) for v in quat.split(",")]
    return x, y, 2.0 * math.atan2(z, w)


def demo_name(scene, m, n):
    return FAM[scene] + (f"t{m}" if m == n else f"s{m}g{n}")


def poly_signed_dist(px, py, poly):
    """Signed distance point->polygon (negative inside; even-odd, winding
    agnostic)."""
    n = len(poly)
    dmin = math.inf
    inside = False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        rx, ry = px - x1, py - y1
        den = ex * ex + ey * ey
        t = 0.0 if den == 0.0 else max(0.0, min(1.0, (rx * ex + ry * ey) / den))
        dx, dy = rx - t * ex, ry - t * ey
        d2 = dx * dx + dy * dy
        if d2 < dmin:
            dmin = d2
        if ey != 0.0 and (y1 > py) != (y2 > py) and \
                px < x1 + ex * (py - y1) / ey:
            inside = not inside
    dmin = math.sqrt(dmin)
    return -dmin if inside else dmin


def min_obstacle_sdf(px, py, discs, polys):
    """Min signed distance from point to all scene obstacles (m).
    +inf if the scene has no obstacles."""
    d = math.inf
    for ox, oy, r in discs:
        d = min(d, math.hypot(px - ox, py - oy) - r)
    for p in polys:
        d = min(d, poly_signed_dist(px, py, p))
    return d


def sample_boundary(footprint, spacing=BODY_POINT_SPACING):
    """Fixed body-point set: footprint polygon boundary sampled every
    `spacing` m in the OBJECT frame. Each edge gets ceil(len/spacing) equal
    subdivisions; vertices included once. Deterministic."""
    pts = []
    n = len(footprint)
    for i in range(n):
        x1, y1 = footprint[i]
        x2, y2 = footprint[(i + 1) % n]
        L = math.hypot(x2 - x1, y2 - y1)
        k = max(1, int(math.ceil(L / spacing)))
        for j in range(k):  # includes start vertex, excludes end (next edge's)
            t = j / k
            pts.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    return np.asarray(pts)  # (U, 2)


def quat_to_rpy(qw, qx, qy, qz):
    """Roll/pitch/yaw (XYZ extrinsic / ZYX intrinsic) from wxyz quaternion."""
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    s = 2 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, s)))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def read_state_trace(run_dir):
    """Returns dict of numpy arrays: t, x, y, yaw, roll, pitch, pos_err,
    ang_err. Skips malformed trailing lines (in-progress writes)."""
    rows = []
    path = os.path.join(run_dir, "state_trace.jsonl")
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue  # partial trailing write
            obj = r["obj"]
            roll, pitch, yaw = quat_to_rpy(*obj[0:4])
            rows.append((r["t"], obj[4], obj[5], yaw, roll, pitch,
                         r.get("pos_err", float("nan")),
                         r.get("ang_err", float("nan"))))
    if not rows:
        return None
    a = np.asarray(rows)
    return {"t": a[:, 0], "x": a[:, 1], "y": a[:, 2], "yaw": a[:, 3],
            "roll": a[:, 4], "pitch": a[:, 5], "pos_err": a[:, 6],
            "ang_err": a[:, 7]}


def parse_launcher(run_dir):
    """Returns (success:bool, success_t:float|nan, final:dict|None, done:bool).
    done = the run finished (RUN DONE line present)."""
    path = os.path.join(run_dir, "launcher.log")
    success, success_t, final, done = False, float("nan"), None, False
    if not os.path.isfile(path):
        return success, success_t, final, done
    with open(path) as f:
        for line in f:
            if line.startswith("SUCCESS t="):
                success = True
                try:
                    success_t = float(line.split("t=")[1].split()[0])
                except (ValueError, IndexError):
                    pass
            elif line.startswith("FINAL "):
                try:
                    final = json.loads(line[len("FINAL "):])
                except json.JSONDecodeError:
                    pass
            elif line.startswith("RUN DONE"):
                done = True
    return success, success_t, final, done


def run_complete(run_dir):
    """A run dir is usable when its launcher.log has both FINAL and RUN DONE
    and state_trace.jsonl exists non-empty."""
    st = os.path.join(run_dir, "state_trace.jsonl")
    if not (os.path.isfile(st) and os.path.getsize(st) > 0):
        return False
    _, _, final, done = parse_launcher(run_dir)
    return done and final is not None


def iter_runs(variants=VARIANTS, scenes=SCENES, require_complete=True):
    """Yields (variant, scene, start:int, goal:int, run_id, run_dir)."""
    for variant in variants:
        for scene in scenes:
            sdir = os.path.join(ROOT, variant, scene)
            if not os.path.isdir(sdir):
                continue
            for name in sorted(os.listdir(sdir)):
                mm = re.fullmatch(r"s(\d{2})g(\d{2})", name)
                if not mm:
                    continue
                rd = os.path.join(sdir, name)
                if not os.path.isdir(rd):
                    continue
                if require_complete and not run_complete(rd):
                    continue
                yield (variant, scene, int(mm.group(1)), int(mm.group(2)),
                       f"{variant}/{scene}/{name}", rd)


def write_csv(path, fieldnames, rows):
    import csv
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)


def upsert_csv(path, fieldnames, new_rows, key_fields):
    """Idempotent append/update: rows keyed by key_fields are replaced."""
    import csv
    rows = {}
    if os.path.isfile(path):
        with open(path) as f:
            for r in csv.DictReader(f):
                rows[tuple(r.get(k, "") for k in key_fields)] = r
    for r in new_rows:
        rows[tuple(str(r[k]) for k in key_fields)] = {k: str(v) for k, v in
                                                      r.items()}
    ordered = sorted(rows.values(),
                     key=lambda r: tuple(r.get(k, "") for k in key_fields))
    write_csv(path, fieldnames, ordered)
