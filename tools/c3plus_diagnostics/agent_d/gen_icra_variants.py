#!/usr/bin/env python3
"""Create anything_icra_c_v2..v5 task dirs from the upstream pose variants."""
import os, math, re, shutil, yaml

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
OIM = "/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX"
pv = yaml.safe_load(open(f"{OIM}/examples/poses/icra_sign.yaml"))
base = f"{WT}/examples/sampling_c3/anything_icra_c"

for k in "2345":
    s, g = pv["starts"][k], pv["goals"][k]
    d = f"{WT}/examples/sampling_c3/anything_icra_c_v{k}"
    if os.path.exists(d):
        shutil.rmtree(d)
    shutil.copytree(base, d)
    for fn in os.listdir(f"{d}/parameters"):
        p = f"{d}/parameters/{fn}"
        t = open(p).read().replace("anything_icra_c/", f"anything_icra_c_v{k}/")
        open(p, "w").write(t)
    # start pose
    sp = f"{d}/parameters/sim_params.yaml"
    t = open(sp).read()
    qw, qz = math.cos(s[2]/2), math.sin(s[2]/2)
    t = t.replace("q_init_object: [1.0, 0, 0, 0, 0.3, 0.4, -0.016]",
                  f"q_init_object: [{qw:.6f}, 0, 0, {qz:.6f}, {s[0]}, {s[1]}, -0.016]")
    t = t.replace("q_init_objects: [[1.0, 0, 0, 0, 0.3, 0.4, -0.016]]",
                  f"q_init_objects: [[{qw:.6f}, 0, 0, {qz:.6f}, {s[0]}, {s[1]}, -0.016]]")
    open(sp, "w").write(t)
    # goal pose
    gp = f"{d}/parameters/goal_params.yaml"
    t = open(gp).read()
    gw, gz = math.cos(g[2]/2), math.sin(g[2]/2)
    t = t.replace("fixed_target_position: [0.5, -0.4, -0.0165]",
                  f"fixed_target_position: [{g[0]}, {g[1]}, -0.0165]")
    t = t.replace("fixed_target_orientation: [0.70710678, 0.0, 0.0, 0.70710678]",
                  f"fixed_target_orientation: [{gw:.8f}, 0.0, 0.0, {gz:.8f}]")
    t = t.replace("fixed_target_positions: [[0.5, -0.4, -0.0165]]",
                  f"fixed_target_positions: [[{g[0]}, {g[1]}, -0.0165]]")
    t = t.replace("fixed_target_orientations: [[0.70710678, 0, 0, 0.70710678]]",
                  f"fixed_target_orientations: [[{gw:.8f}, 0, 0, {gz:.8f}]]")
    open(gp, "w").write(t)
    print(f"v{k}: start {s} goal {g}")
print("variant dirs written")
