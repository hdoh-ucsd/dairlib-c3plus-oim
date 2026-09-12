#!/usr/bin/env python3
"""Agent D: dense perimeter contact bank replay (section 10).

From a given object state (x, y, yaw) in a scene, place the pusher at N
approach sectors on a ring, push through the object center for PUSH_LEN, and
measure realized motion + goal progress with the standalone rig physics.
Usage: dense_bank.py TASK_DIR OX OY OYAW GX GY N OUT_CSV TAG
"""
import sys, os, math, csv, subprocess, json, tempfile

task = sys.argv[1]
ox, oy, oyaw, gx, gy = map(float, sys.argv[2:7])
n = int(sys.argv[7]); out_csv = sys.argv[8]; tag = sys.argv[9]
HERE = os.path.dirname(os.path.abspath(__file__))
PY = "/root/miniconda3/envs/push_anything_ADMM/bin/python3"

# Patch: forced_contact_sanity places object at sim q_init; we need arbitrary
# state, so we pass offsets relative to q_init and override via env.
rows = []
d0 = math.hypot(ox - gx, oy - gy)
for k in range(n):
    ang = 2 * math.pi * k / n
    ring = 0.085
    sxo, syo = ring * math.cos(ang), ring * math.sin(ang)
    # push through the object center, 0.09 m
    dx, dy = -0.09 * math.cos(ang), -0.09 * math.sin(ang)
    env = dict(os.environ, AGENTD_OBJ_POSE=f"{ox},{oy},{oyaw}")
    r = subprocess.run(
        [PY, f"{HERE}/forced_contact_sanity.py", task, f"{sxo}", f"{syo}",
         f"{dx}", f"{dy}", f"{tag}_s{k}"],
        capture_output=True, text=True, env=env)
    line = [l for l in r.stdout.splitlines() if l.startswith("{")]
    if not line:
        rows.append(dict(tag=tag, sector_deg=round(math.degrees(ang), 1), error=r.stderr[-200:]))
        continue
    d = eval(line[-1])
    # realized goal progress: parse final object pose printed by the rig
    fin = [l for l in r.stdout.splitlines() if l.startswith("OBJ_FINAL")]
    prog = ""
    if fin:
        fx, fy, fyaw = map(float, fin[-1].split()[1:4])
        prog = round(d0 - math.hypot(fx - gx, fy - gy), 4)
    rows.append(dict(tag=tag, sector_deg=round(math.degrees(ang), 1),
                     contact=d["contact_acquired"], obj_dxy=d["obj_dxy"],
                     obj_dyaw=d["obj_dyaw"], goal_progress=prog,
                     min_obj_obstacle=d["min_obj_obstacle"],
                     productive=d["obj_dxy"] > 0.005))
new = not os.path.exists(out_csv)
with open(out_csv, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["tag", "sector_deg", "contact", "obj_dxy",
                                      "obj_dyaw", "goal_progress",
                                      "min_obj_obstacle", "productive", "error"])
    if new:
        w.writeheader()
    for r in rows:
        w.writerow(r)
print(json.dumps(rows))
