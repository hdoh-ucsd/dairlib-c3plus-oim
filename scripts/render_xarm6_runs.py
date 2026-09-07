#!/usr/bin/env python3
"""Render top-down MP4 videos of xArm6 open_table runs from state traces."""
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation, patches, transforms

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RollPitchYaw

WT = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant"
OUT = "/root/push_anything_ADMM/results/final_oim_c3plus_comparison/figures/trajectories/videos"
RES = "/root/push_anything_ADMM/results"

# ---------- FK plant ----------
def build_plant():
    plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    arm, = parser.AddModels(os.path.join(
        WT, "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml"))
    ee, = parser.AddModels(os.path.join(
        WT, "examples/sampling_c3/urdf/end_effector_full.urdf"))
    plant.WeldFrames(
        plant.GetFrameByName("xarm6_link6", arm),
        plant.GetFrameByName("end_effector_flange", ee),
        RigidTransform(RollPitchYaw(3.1415, 0, 0), [0, 0, 0.107]))
    # weld base to world if floating
    base = plant.GetBodyByName("xarm6_link_base", arm) if plant.HasBodyNamed("xarm6_link_base", arm) else None
    plant.Finalize()
    ctx = plant.CreateDefaultContext()
    tip = plant.GetBodyByName("end_effector_tip", ee)
    return plant, ctx, tip

def make_fk(plant, ctx, tip):
    nq = plant.num_positions()
    def fk(q5):
        q = np.zeros(nq)
        q[:len(q5)] = q5
        plant.SetPositions(ctx, q)
        return plant.EvalBodyPoseInWorld(ctx, tip).translation()
    return fk

# ---------- geometry ----------
CROSS = (0.089, 0.0198, 0.0, 0.0099)   # w, h, cx, cy (local)
STEM = (0.0198, 0.0794, 0.0, -0.0397)

def t_patches(x, y, yaw, **kw):
    ps = []
    for (w, h, cx, cy) in (CROSS, STEM):
        p = patches.Rectangle((cx - w/2, cy - h/2), w, h, **kw)
        ps.append(p)
    tr = transforms.Affine2D().rotate(yaw).translate(x, y)
    return ps, tr

def quat_yaw(q):  # wxyz
    w, x, y, z = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

# ---------- run enumeration ----------
def load_yaml_pairs():
    import re
    starts, goals = {}, {}
    sec = None
    for line in open("/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX/examples/poses/open_table.yaml"):
        s = line.split("#")[0].strip()
        if s.startswith("starts:"): sec = starts; continue
        if s.startswith("goals:"): sec = goals; continue
        m = re.match(r'"(\d)":\s*\[([^\]]+)\]', s)
        if m and sec is not None:
            sec[m.group(1)] = [float(v) for v in m.group(2).split(",")]
    return starts, goals

GOAL_AB = (0.5, -0.3, math.pi)

def runs():
    _, goals = load_yaml_pairs()
    out = []
    for i in range(1, 6):
        out.append(("A", f"xarm6_trial{i}",
                    f"{RES}/panda_to_xarm6_port_final_r6/xarm6_trial{i}", GOAL_AB, "legacy"))
    for i in range(1, 4):
        out.append(("B", f"trial{i}",
                    f"{RES}/final_oim_c3plus_comparison_runs/c3plus/open_table/trial{i}", GOAL_AB, "legacy"))
    for i in range(1, 4):
        out.append(("C", f"matched_trial{i}",
                    f"{RES}/final_oim_c3plus_comparison_runs/c3plus_matched/open_table/trial{i}",
                    tuple(goals[str(i)]), "protocol"))
    for name in ("t4_a", "t5_a", "t1_b", "t2_b", "t3_b", "t4_b", "t5_b"):
        k = name[1]
        out.append(("C", f"matched_b_{name}",
                    f"{RES}/final_oim_c3plus_comparison_runs/c3plus_matched_b/open_table/{name}",
                    tuple(goals[k]), "protocol"))
    return out

# ---------- rendering ----------
def first_cross(data, pos_th, ang_th):
    for d in data:
        if d["pos_err"] < pos_th and d["ang_err"] < ang_th:
            return d["t"]
    return None

def render(setn, runname, dirpath, goal, gate, plant_stuff):
    trace = os.path.join(dirpath, "state_trace.jsonl")
    data = [json.loads(l) for l in open(trace) if l.strip()]
    if not data:
        raise RuntimeError("empty trace")
    fk = plant_stuff
    step = max(1, math.ceil(len(data) / 1200))
    frames = data[::step]

    t_legacy = first_cross(data, 0.02, 0.1)
    t_proto = first_cross(data, 0.05, 0.1)
    gate_t = t_legacy if gate == "legacy" else t_proto
    success = gate_t is not None
    label = "SUCCESS" if success else "FAIL"

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlim(0, 0.9); ax.set_ylim(-0.7, 0.7)
    ax.add_patch(patches.Circle((0, 0), 0.70, fill=False, color="0.85", lw=1))
    ax.plot(0, 0, marker="s", ms=10, color="0.4")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

    # goal outline
    gps, gtr = t_patches(goal[0], goal[1], goal[2], fill=False,
                         ls="--", ec="green", lw=1.5)
    for p in gps:
        p.set_transform(gtr + ax.transData); ax.add_patch(p)

    # object patches
    ops, _ = t_patches(0, 0, 0, fc="tab:blue", ec="navy", alpha=0.85)
    for p in ops: ax.add_patch(p)
    ee_c = patches.Circle((0, 0), 0.0195, fc="tab:red", ec="darkred", zorder=5)
    ax.add_patch(ee_c)

    obj_trail, = ax.plot([], [], "-", color="tab:blue", alpha=0.35, lw=1)
    ee_trail, = ax.plot([], [], "-", color="tab:red", alpha=0.35, lw=1)

    hud = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                  fontsize=9, family="monospace")
    stat = ax.text(0.02, 0.90, "", transform=ax.transAxes, va="top",
                   fontsize=10, family="monospace", fontweight="bold")

    ee_xy = [fk(f["q"])[:2] for f in frames]
    TRAIL = 80

    def update(i):
        f = frames[i]
        yaw = quat_yaw(f["obj"][:4])
        x, y = f["obj"][4], f["obj"][5]
        tr = transforms.Affine2D().rotate(yaw).translate(x, y) + ax.transData
        for p in ops: p.set_transform(tr)
        ee_c.center = tuple(ee_xy[i])
        j0 = max(0, i - TRAIL)
        obj_trail.set_data([fr["obj"][4] for fr in frames[j0:i+1]],
                           [fr["obj"][5] for fr in frames[j0:i+1]])
        ee_trail.set_data([p[0] for p in ee_xy[j0:i+1]],
                          [p[1] for p in ee_xy[j0:i+1]])
        hud.set_text(f"{setn}_{runname}  gate={gate} "
                     f"({'0.02/0.1' if gate=='legacy' else '0.05/0.1'})\n"
                     f"t={f['t']:7.1f}s  pos_err={f['pos_err']:.4f}  "
                     f"ang_err={f['ang_err']:.4f}")
        if success and f["t"] >= gate_t:
            extra = ""
            if t_legacy is not None and t_proto is not None and t_legacy != t_proto:
                extra = f"  (0.05 gate @ {t_proto:.1f}s, 0.02 gate @ {t_legacy:.1f}s)"
            elif t_proto is not None and t_legacy is None:
                extra = f"  (0.05 gate @ {t_proto:.1f}s; 0.02 gate never)"
            stat.set_text(f"SUCCESS @ t={gate_t:.1f}s{extra}")
            stat.set_color("green")
        else:
            alt = ""
            if not success and t_proto is not None and gate == "legacy":
                alt = f" (0.05 gate crossed @ {t_proto:.1f}s)"
            stat.set_text(("running..." if (success or i < len(frames)-1)
                           else "FAIL" + alt))
            stat.set_color("0.3" if success or i < len(frames)-1 else "red")
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    outfile = os.path.join(OUT, f"{setn}_{runname}_{label}.mp4")
    anim.save(outfile, writer=animation.FFMpegWriter(fps=10, bitrate=1800))
    plt.close(fig)
    return outfile, label


def main():
    os.makedirs(OUT, exist_ok=True)
    plant, ctx, tip = build_plant()
    fk = make_fk(plant, ctx, tip)
    only = sys.argv[1] if len(sys.argv) > 1 else None
    done, failed = [], []
    for setn, runname, dirpath, goal, gate in runs():
        if only and only not in f"{setn}_{runname}":
            continue
        try:
            out, label = render(setn, runname, dirpath, goal, gate, fk)
            sz = os.path.getsize(out)
            print(f"OK  {out}  {sz/1e6:.1f}MB  {label}", flush=True)
            done.append(out)
        except Exception as e:
            print(f"ERR {setn}_{runname}: {e}", flush=True)
            failed.append((runname, str(e)))
    print(f"done={len(done)} failed={len(failed)}")

if __name__ == "__main__":
    main()
