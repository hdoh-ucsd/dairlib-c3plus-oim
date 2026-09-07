#!/usr/bin/env python3
"""Render top-down MP4s for the xArm6 PORT development campaigns (P6 + P5 arcs)."""
import csv
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
RES = "/root/push_anything_ADMM/results"
OUT = os.path.join(RES, "final_oim_c3plus_comparison/figures/trajectories/port_arc_videos")
XML_6DOF = os.path.join(WT, "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport_6dof.xml")
XML_5J = os.path.join(WT, "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml")
EE_URDF = os.path.join(WT, "examples/sampling_c3/urdf/end_effector_full.urdf")

GOAL = (0.5, -0.3, math.pi)
TILT_TH = 0.3  # rad


def build_plant(arm_xml):
    plant = MultibodyPlant(0.0)
    parser = Parser(plant)
    arm, = parser.AddModels(arm_xml)
    ee, = parser.AddModels(EE_URDF)
    plant.WeldFrames(
        plant.GetFrameByName("xarm6_link6", arm),
        plant.GetFrameByName("end_effector_flange", ee),
        RigidTransform(RollPitchYaw(3.1415, 0, 0), [0, 0, 0.107]))
    plant.Finalize()
    ctx = plant.CreateDefaultContext()
    tip = plant.GetBodyByName("end_effector_tip", ee)
    nq = plant.num_positions()

    def fk(qj):
        q = np.zeros(nq)
        q[:len(qj)] = qj
        plant.SetPositions(ctx, q)
        return plant.EvalBodyPoseInWorld(ctx, tip).translation()
    return fk


# ---------- geometry ----------
CROSS = (0.089, 0.0198, 0.0, 0.0099)
STEM = (0.0198, 0.0794, 0.0, -0.0397)


def t_patches(**kw):
    return [patches.Rectangle((cx - w / 2, cy - h / 2), w, h, **kw)
            for (w, h, cx, cy) in (CROSS, STEM)]


def quat_yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_tilt(q):
    """Angle between body z-axis and world z (max of roll/pitch effect)."""
    w, x, y, z = q
    r33 = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, r33)))


# ---------- manifest ----------
def M(setn, rnd, base, labels, slugs):
    out = []
    for i in range(1, 6):
        out.append(dict(setn=setn, rnd=rnd, trial=f"t{i}",
                        path=f"{RES}/{base}/xarm6_trial{i}",
                        label=labels.get(i, labels.get(0)),
                        slug=slugs.get(i, slugs.get(0))))
    return out


RUNS = []
# SET P6 (6-DOF OSC port arc)
RUNS += M("P6", "r1", "panda_to_xarm6_port_runs",
          {1: "FAIL planner workspace crash t~75s then OSC hold",
           0: "FAIL unreachable reposition target (r5 shell)"},
          {1: "CRASH", 0: "STALL"})
RUNS += M("P6", "r2", "panda_to_xarm6_port_runs_r2",
          {3: "FAIL wedged-lift topple",
           0: "FAIL topple at first reposition lift (t~8s)"},
          {0: "TOPPLE"})
RUNS += M("P6", "r3", "panda_to_xarm6_port_runs_r3",
          {0: "FAIL topple (tilt-weight ablation)"}, {0: "TOPPLE"})
RUNS += M("P6", "r4", "panda_to_xarm6_port_runs_r4",
          {0: "FAIL topple (lift-height ablation)"}, {0: "TOPPLE"})
RUNS += M("P6", "r5", "panda_to_xarm6_port_runs_r5",
          {0: "FAIL startup workspace crash (Kd20) - zero motion"}, {0: "ZERO"})
# SET P5 (5-joint final arc)
RUNS += M("P5", "r1", "panda_to_xarm6_port_final_r1",
          {0: "FAIL descent stall (per-joint qdot clamp)"}, {0: "STALL"})
RUNS += M("P5", "r2", "panda_to_xarm6_port_final_r2",
          {4: "NEAR-MISS best pos 0.009 m (wall-clock truncated)",
           2: "FAIL radius-assert crash + stale chase",
           3: "FAIL topple",
           0: "FAIL acquisition stall"},
          {4: "NEARMISS", 2: "CRASH", 3: "TOPPLE", 0: "STALL"})
RUNS += M("P5", "r3", "panda_to_xarm6_port_final_r3",
          {0: "FAIL sampler starvation (reach 0.66 ablation) - zero pushes"},
          {0: "ZERO"})
RUNS += M("P5", "r4", "panda_to_xarm6_port_final_r4",
          {0: "FAIL v_des-cancellation trap stall (pre-fix)"}, {0: "STALL"})
RUNS += M("P5", "r5", "panda_to_xarm6_port_final_r5",
          {0: "FAIL trap stall / progressing at cap (pre-fix)"}, {0: "STALL"})


# ---------- rendering ----------
def render(run, fks):
    trace = os.path.join(run["path"], "state_trace.jsonl")
    data = [json.loads(l) for l in open(trace) if l.strip()]
    if not data:
        raise RuntimeError("empty trace")
    nq = len(data[0]["q"])
    fk = fks[6] if nq == 6 else fks[5]

    step = max(1, math.ceil(len(data) / 1200))
    frames = data[::step]

    tilts = [quat_tilt(f["obj"][:4]) for f in frames]
    topple_t = None
    for f, tl in zip(frames, tilts):
        if tl > TILT_TH:
            topple_t = f["t"]
            break

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlim(0, 0.9); ax.set_ylim(-0.7, 0.7)
    ax.add_patch(patches.Circle((0, 0), 0.70, fill=False, color="0.85", lw=1))
    ax.plot(0, 0, marker="s", ms=10, color="0.4")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

    gtr = transforms.Affine2D().rotate(GOAL[2]).translate(GOAL[0], GOAL[1])
    for p in t_patches(fill=False, ls="--", ec="green", lw=1.5):
        p.set_transform(gtr + ax.transData); ax.add_patch(p)

    ops = t_patches(fc="tab:blue", ec="navy", alpha=0.85)
    for p in ops: ax.add_patch(p)
    ee_c = patches.Circle((0, 0), 0.0195, fc="tab:red", ec="darkred", zorder=5)
    ax.add_patch(ee_c)
    obj_trail, = ax.plot([], [], "-", color="tab:blue", alpha=0.35, lw=1)
    ee_trail, = ax.plot([], [], "-", color="tab:red", alpha=0.35, lw=1)

    hud = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                  fontsize=9, family="monospace")
    lab = ax.text(0.02, 0.905, run["label"], transform=ax.transAxes, va="top",
                  fontsize=10, family="monospace", fontweight="bold",
                  color=("darkorange" if run["slug"] == "NEARMISS" else "red"))

    ee_xy = [fk(f["q"])[:2] for f in frames]
    TRAIL = 80
    name = f"{run['setn']}_{run['rnd']}_{run['trial']}"

    def update(i):
        f = frames[i]
        yaw = quat_yaw(f["obj"][:4])
        x, y = f["obj"][4], f["obj"][5]
        tr = transforms.Affine2D().rotate(yaw).translate(x, y) + ax.transData
        toppled = topple_t is not None and f["t"] >= topple_t
        for p in ops:
            p.set_transform(tr)
            p.set_facecolor("tab:red" if toppled else "tab:blue")
            p.set_edgecolor("darkred" if toppled else "navy")
        ee_c.center = tuple(ee_xy[i])
        j0 = max(0, i - TRAIL)
        obj_trail.set_data([fr["obj"][4] for fr in frames[j0:i + 1]],
                           [fr["obj"][5] for fr in frames[j0:i + 1]])
        ee_trail.set_data([p[0] for p in ee_xy[j0:i + 1]],
                          [p[1] for p in ee_xy[j0:i + 1]])
        hud.set_text(f"{name}  ({nq}-joint FK)  goal 0.5,-0.3,pi  gate 0.02/0.1\n"
                     f"t={f['t']:7.1f}s  pos_err={f['pos_err']:.4f}  "
                     f"ang_err={f['ang_err']:.4f}  tilt={tilts[i]:.3f}rad"
                     + ("  TOPPLED" if toppled else ""))
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(frames), blit=False)
    outfile = os.path.join(OUT, f"{name}_{run['slug']}.mp4")
    anim.save(outfile, writer=animation.FFMpegWriter(fps=10, bitrate=1800))
    plt.close(fig)
    return outfile, frames[-1]["t"], topple_t


def main():
    os.makedirs(OUT, exist_ok=True)
    fks = {6: build_plant(XML_6DOF), 5: build_plant(XML_5J)}
    only = sys.argv[1] if len(sys.argv) > 1 else None
    rows, skipped, failed = [], [], []
    for run in RUNS:
        name = f"{run['setn']}_{run['rnd']}_{run['trial']}"
        if only and only not in name:
            continue
        if not os.path.isfile(os.path.join(run["path"], "state_trace.jsonl")):
            print(f"SKIP {name}: no state_trace.jsonl", flush=True)
            skipped.append(name)
            continue
        try:
            out, dur, tt = render(run, fks)
            print(f"OK  {out}  {os.path.getsize(out)/1e6:.1f}MB  dur={dur:.1f}s"
                  f"  topple_t={tt}", flush=True)
            rows.append([os.path.basename(out), f"{run['setn']}_{run['rnd']}",
                         run["trial"], run["label"], f"{dur:.1f}",
                         "" if tt is None else f"{tt:.1f}"])
        except Exception as e:
            print(f"ERR {name}: {e}", flush=True)
            failed.append((name, str(e)))
    with open(os.path.join(OUT, "manifest.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "round", "trial", "label", "duration_s",
                    "topple_detected_t"])
        w.writerows(rows)
    print(f"done={len(rows)} skipped={len(skipped)} failed={len(failed)}")


if __name__ == "__main__":
    main()
