#!/usr/bin/env python3
"""Generate the 120 off-diagonal (start M, goal N) task dirs for the 5x5 grid:
copy the diagonal dir t<M> (carries start M's q_init_*) and overwrite the goal
block with pair N's goal (taken verbatim from the diagonal dir t<N>). Names:
<family>_s<M>g<N>. Idempotent (rmtree + recreate)."""
import os, re, shutil

WT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SC3 = os.path.join(WT, "examples/sampling_c3")
FAM = {
    "open_task": "matched_open_table_xarm6_t",
    "single_obstacle": "matched_single_obstacle_xarm6_t",
    "shelf_gap": "matched_shelf_gap_xarm6_t",
    "ycb_clutter": "matched_ycb_clutter_xarm6_t",
    "slalom": "matched_slalom_xarm6_t",
    "icra_sign": "anything_icra_c_matched_xarm6_t",
}
GOAL_KEYS = ["fixed_target_position", "fixed_target_orientation",
             "fixed_target_positions", "fixed_target_orientations"]


def get_goal_lines(gp_path):
    out = {}
    for line in open(gp_path):
        m = re.match(r"^(\w+): (.*)$", line)
        if m and m.group(1) in GOAL_KEYS:
            out[m.group(1)] = line
    assert len(out) == 4, gp_path
    return out


made = 0
for scene, fam in FAM.items():
    goals = {n: get_goal_lines(os.path.join(
        SC3, f"{fam}{n}", "parameters/goal_params.yaml")) for n in range(1, 6)}
    for m in range(1, 6):
        for n in range(1, 6):
            if m == n:
                continue
            src = os.path.join(SC3, f"{fam}{m}")
            name = f"{fam[:-2]}_s{m}g{n}" if fam.endswith("_t") else None
            name = fam[:-1] + f"s{m}g{n}"  # e.g. matched_open_table_xarm6_s2g4
            dst = os.path.join(SC3, name)
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.copytree(src, dst)
            cp = os.path.join(dst, "parameters/sampling_c3_controller_params.yaml")
            txt = open(cp).read()
            open(cp, "w").write(txt.replace(f"{fam}{m}", name))
            gp = os.path.join(dst, "parameters/goal_params.yaml")
            lines = open(gp).readlines()
            out = []
            for line in lines:
                key = line.split(":")[0].strip()
                out.append(goals[n][key] if key in GOAL_KEYS else line)
            open(gp, "w").writelines(out)
            made += 1
print("created", made, "grid dirs")
