#!/usr/bin/env python3
"""Generate pose-matched C3+ trial variants for the matched OIM comparison.

For scene in {open_table, single_obstacle, shelf_gap} and trial k in {1,2,3},
creates examples/sampling_c3/matched_<scene>_xarm6_t<k>/ as a copy of
push_t_bt010_<scene>_xarm6 with:
  - sim_params q_init_object(s) = OIM start k  ([x,y,yaw] -> [w,0,0,z, x,y,0.0008])
  - goal_params fixed_target_position(s)/orientation(s) = OIM goal k
    (quaternion is W-FIRST: goal_generator.cc builds Eigen::Quaterniond(v[0],
     v[1], v[2], v[3]) = (w,x,y,z); the existing [0,0,0,1] == yaw=pi confirms)
  - all internal paths rewritten to the new demo dir
  - obstacle scene pointed at the OIM-native-frame SDFs
    (single_obstacle_box_oimframe.sdf / scene_shelf_gap_oimframe.sdf) with the
    scenario_params obstacle disc x's rigidly shifted by the same delta
    (-0.15 single_obstacle, -0.11 shelf_gap).

Poses come verbatim (RIGID IDENTITY, benchmark_protocol.yaml) from
external/Object-Informed-Manipulation-MJX/examples/poses/<scene>.yaml.
Workspace fit (r <= 0.70, |y| <= 0.6) is checked and reported, never fixed.
"""
import math
import os
import re
import shutil
import sys

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
POSES_DIR = "/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX/examples/poses"
SCENES = ["open_table", "single_obstacle", "shelf_gap"]
TRIALS = [1, 2, 3]
Z_OBJ = 0.0008
R_MAX, Y_MAX = 0.70, 0.6

# Obstacle-frame corrections: (new obstacle_model, x-shift for scenario discs)
OBSTACLE_FIX = {
    "single_obstacle": ("examples/sampling_c3/urdf/single_obstacle_box_oimframe.sdf", -0.15),
    "shelf_gap": ("examples/sampling_c3/urdf/scene_shelf_gap_oimframe.sdf", -0.11),
}


def fmt(v):
    return f"{v:.6g}"


def quat_wxyz(yaw):
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def replace_line(path, key, new_line):
    with open(path) as f:
        lines = f.readlines()
    pat = re.compile(rf"^{re.escape(key)}\s*:")
    hits = [i for i, l in enumerate(lines) if pat.match(l)]
    if len(hits) != 1:
        raise RuntimeError(f"{path}: key '{key}' matched {len(hits)} lines")
    lines[hits[0]] = new_line + "\n"
    with open(path, "w") as f:
        f.writelines(lines)


def workspace_check(scene, kind, k, x, y):
    r = math.hypot(x, y)
    ok = r <= R_MAX and abs(y) <= Y_MAX
    verdict = "OK" if ok else "VIOLATION"
    print(f"  [workspace] {scene} {kind} {k}: r={r:.3f} (<= {R_MAX}), |y|={abs(y):.3f} (<= {Y_MAX}) -> {verdict}")
    return ok


def main():
    violations = []
    made = []
    for scene in SCENES:
        with open(os.path.join(POSES_DIR, f"{scene}.yaml")) as f:
            poses = yaml.safe_load(f)
        src = os.path.join(REPO, "examples/sampling_c3", f"push_t_bt010_{scene}_xarm6")
        src_name = os.path.basename(src)
        for k in TRIALS:
            sx, sy, syaw = poses["starts"][str(k)]
            gx, gy, gyaw = poses["goals"][str(k)]
            if not workspace_check(scene, "start", k, sx, sy):
                violations.append((scene, "start", k))
            if not workspace_check(scene, "goal", k, gx, gy):
                violations.append((scene, "goal", k))

            dst_name = f"matched_{scene}_xarm6_t{k}"
            dst = os.path.join(REPO, "examples/sampling_c3", dst_name)
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.copytree(src, dst)

            # Rewrite all internal path references.
            for root, _, files in os.walk(dst):
                for fn in files:
                    p = os.path.join(root, fn)
                    with open(p) as f:
                        txt = f.read()
                    if src_name in txt:
                        with open(p, "w") as f:
                            f.write(txt.replace(src_name, dst_name))

            # sim_params: initial object pose.
            sq = quat_wxyz(syaw)
            q7 = ", ".join(fmt(v) for v in sq + [sx, sy, Z_OBJ])
            simp = os.path.join(dst, "parameters/sim_params.yaml")
            replace_line(simp, "q_init_object", f"q_init_object: [{q7}]")
            replace_line(simp, "q_init_objects", f"q_init_objects: [[{q7}]]")

            # goal_params: fixed goal.
            gq = quat_wxyz(gyaw)
            pos = ", ".join(fmt(v) for v in [gx, gy, Z_OBJ])
            quat = ", ".join(fmt(v) for v in gq)
            gp = os.path.join(dst, "parameters/goal_params.yaml")
            replace_line(gp, "fixed_target_position", f"fixed_target_position: [{pos}]")
            replace_line(gp, "fixed_target_orientation", f"fixed_target_orientation: [{quat}]")
            replace_line(gp, "fixed_target_positions", f"fixed_target_positions: [[{pos}]]")
            replace_line(gp, "fixed_target_orientations", f"fixed_target_orientations: [[{quat}]]")

            # scenario_params: OIM-native obstacle frame.
            if scene in OBSTACLE_FIX:
                model, dx = OBSTACLE_FIX[scene]
                scen = os.path.join(dst, "parameters/scenario_params.yaml")
                sp = yaml.safe_load(open(scen))
                assert sp["obstacle_model"].endswith(".sdf")
                sp["obstacle_model"] = model
                sp["obstacles"] = [[round(o[0] + dx, 6), o[1], o[2]] for o in sp["obstacles"]]
                with open(scen, "w") as f:
                    yaml.safe_dump(sp, f, sort_keys=False, default_flow_style=None)

            made.append(dst_name)
            print(f"  [gen] {dst_name}: start=({sx},{sy},yaw={syaw}) goal=({gx},{gy},yaw={gyaw})")

    print(f"\nGenerated {len(made)} demo dirs.")
    if violations:
        print("WORKSPACE VIOLATIONS (reported, not fixed):")
        for v in violations:
            print(" ", v)
        sys.exit(1)
    print("All starts/goals inside workspace (r <= 0.70, |y| <= 0.6).")


if __name__ == "__main__":
    main()
