#!/usr/bin/env python3
"""Generate the missing pair-2..5 task dirs for ycb_clutter / icra_sign /
slalom from their pair-1 templates, and append the upstream robot-base disc
to the existing single_obstacle / shelf_gap t2-t5 scenario files (t1 already
has it). Poses are the upstream examples/poses values (d6d80a6), start/goal
quats derived as [cos(yaw/2), 0, 0, sin(yaw/2)].
"""
import math, os, re, shutil

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SC3 = os.path.join(WT, "examples/sampling_c3")

# upstream examples/poses/*.yaml at d6d80a6 (x, y, yaw)
POSES = {
    "ycb": {  # == open_table poses
        "starts": {2: (0.3660, 0.4310, 0.0579), 3: (0.3910, 0.4370, -0.4082),
                   4: (0.3610, 0.3570, 0.3282), 5: (0.4060, 0.3830, 0.2806)},
        "goals":  {2: (0.3970, -0.4310, 3.2400), 3: (0.4040, -0.3930, 2.7924),
                   4: (0.3860, -0.4240, 2.7704), 5: (0.3590, -0.4240, 2.7338)},
    },
    "slalom": {
        "starts": {2: (0.3660, 0.4310, 0.0579), 3: (0.3910, 0.4370, -0.4082),
                   4: (0.3610, 0.3700, 0.3282), 5: (0.4060, 0.3930, 0.2806)},
        "goals":  {2: (0.4040, -0.3930, 2.7924), 3: (0.3630, -0.4100, 3.5684),
                   4: (0.3670, -0.3800, 2.7338), 5: (0.4040, -0.4180, 3.3892)},
    },
    "icra": {
        "starts": {2: (0.2779, 0.4264, 0.0579), 3: (0.3007, 0.4383, -0.4082),
                   4: (0.2910, 0.3534, 0.3282), 5: (0.3284, 0.3896, 0.2806)},
        "goals":  {2: (0.5230, -0.4262, 1.6692), 3: (0.5247, -0.4041, 1.2708),
                   4: (0.5107, -0.4221, 1.1996), 5: (0.4845, -0.4286, 1.1630)},
    },
}
TEMPLATES = {"ycb": "matched_ycb_clutter_xarm6_t1",
             "slalom": "matched_slalom_xarm6_t1",
             "icra": "anything_icra_c_matched_xarm6_t1"}
DIRFMT = {"ycb": "matched_ycb_clutter_xarm6_t{}",
          "slalom": "matched_slalom_xarm6_t{}",
          "icra": "anything_icra_c_matched_xarm6_t{}"}
OBJ_Z = {"ycb": 0.0008, "slalom": 0.0008, "icra": -0.016}


def quat(yaw):
    return round(math.cos(yaw / 2), 6), round(math.sin(yaw / 2), 6)


def sub_line(path, key, newline):
    txt = open(path).read()
    out, n = re.subn(rf"(?m)^{re.escape(key)}: .*$", newline, txt)
    assert n >= 1, (path, key)
    open(path, "w").write(out)


for fam, poses in POSES.items():
    tmpl = TEMPLATES[fam]
    for i in range(2, 6):
        name = DIRFMT[fam].format(i)
        dst = os.path.join(SC3, name)
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(os.path.join(SC3, tmpl), dst)
        # repoint the parameter-file paths
        cp = os.path.join(dst, "parameters/sampling_c3_controller_params.yaml")
        cp_txt = open(cp).read()
        open(cp, "w").write(cp_txt.replace(tmpl, name))
        bz = os.path.join(dst, "BUILD.bazel")
        if os.path.exists(bz):
            bz_txt = open(bz).read()
            open(bz, "w").write(bz_txt.replace(tmpl, name))
        # start pose
        sx, sy, syaw = poses["starts"][i]
        w, z = quat(syaw)
        oz = OBJ_Z[fam]
        sp = os.path.join(dst, "parameters/sim_params.yaml")
        sub_line(sp, "q_init_object",
                 f"q_init_object: [{w}, 0, 0, {z}, {sx}, {sy}, {oz}]")
        sub_line(sp, "q_init_objects",
                 f"q_init_objects: [[{w}, 0, 0, {z}, {sx}, {sy}, {oz}]]")
        # goal
        gx, gy, gyaw = poses["goals"][i]
        gw, gz = quat(gyaw)
        gp = os.path.join(dst, "parameters/goal_params.yaml")
        sub_line(gp, "fixed_target_position",
                 f"fixed_target_position: [{gx}, {gy}, {oz}]")
        sub_line(gp, "fixed_target_orientation",
                 f"fixed_target_orientation: [{gw}, 0, 0, {gz}]")
        sub_line(gp, "fixed_target_positions",
                 f"fixed_target_positions: [[{gx}, {gy}, {oz}]]")
        sub_line(gp, "fixed_target_orientations",
                 f"fixed_target_orientations: [[{gw}, 0, 0, {gz}]]")
        print(f"created {name}: start ({sx},{sy},{syaw}) goal ({gx},{gy},{gyaw})")

# base-disc parity for existing single_obstacle / shelf_gap t2-t5
DISC = "- [0.0, 0.0, 0.09]        # robot base disc (planner-only, upstream scenes.py Circle r=0.09)\n"
for scene in ("matched_single_obstacle_xarm6_t", "matched_shelf_gap_xarm6_t"):
    for i in range(2, 6):
        p = os.path.join(SC3, f"{scene}{i}", "parameters/scenario_params.yaml")
        txt = open(p).read()
        if "robot base disc" in txt:
            print(f"skip (has disc): {scene}{i}")
            continue
        lines = txt.splitlines(True)
        # insert after the last obstacle list entry
        last = max(j for j, l in enumerate(lines) if l.lstrip().startswith("- ["))
        lines.insert(last + 1, DISC)
        open(p, "w").writelines(lines)
        print(f"disc added: {scene}{i}")
print("DONE")
