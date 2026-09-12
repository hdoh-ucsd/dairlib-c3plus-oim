import json, math, os, sys
os.environ["AGENTD_FK"] = "xarm6"
sys.argv = ['x', 'r', 'open_table', '0.5', '-0.3', '0.0', 'a', 'b', 'c']
src = open('tools/c3plus_diagnostics/agent_d/postprocess_draw.py').read()
exec(src.split('# ---- state trace')[0])
import numpy as np
R = "/root/push_anything_ADMM/results/panda_to_xarm6_port_final_r2/xarm6_trial4/state_trace.jsonl"
recs = [json.loads(l) for l in open(R) if l.strip()]
zs_contact, zs_all = [], []
for r in recs:
    e = ee_pos(r['q']); q = r['obj']
    yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
    g = foot_gap(e[0], e[1], q[4], q[5], yaw)
    zs_all.append(e[2])
    if g < 0.002:
        zs_contact.append(e[2])
print("EE z overall p10/p50/p90:", np.percentile(zs_all, [10, 50, 90]).round(3))
print("EE z during 2D-contact p10/p50/p90:", np.percentile(zs_contact, [10, 50, 90]).round(3), "n=", len(zs_contact))
