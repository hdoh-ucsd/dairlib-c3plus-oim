#!/usr/bin/env python3
"""r2 trial1/trial4 forensics: gap distribution, EE radius, topple-onset state."""
import json, math, os, sys
import numpy as np
os.environ.setdefault("AGENTD_FK", "xarm6")
sys.argv = ['x', 'unused', 'open_table', '0.5', '-0.3', '0.0', 'a', 'b', 'c']
src = open('tools/c3plus_diagnostics/agent_d/postprocess_draw.py').read()
exec(src.split('# ---- state trace')[0])  # gives ee_pos, FOOT, foot_gap

R = "/root/push_anything_ADMM/results/panda_to_xarm6_port_final_r2"
for trial in ("xarm6_trial1", "xarm6_trial4"):
    recs = [json.loads(l) for l in open(f"{R}/{trial}/state_trace.jsonl") if l.strip()]
    recs = [r for r in recs if r.get("q")]
    out = []
    for r in recs:
        e = ee_pos(r["q"])
        q = r["obj"]
        yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        sinp = max(-1, min(1, 2*(q[0]*q[2]-q[3]*q[1])))
        tilt = max(abs(math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2))), abs(math.asin(sinp)))
        g = foot_gap(e[0], e[1], q[4], q[5], yaw)
        out.append((r["t"], g, math.hypot(e[0], e[1]), e[2], tilt, r["pos_err"], r["ang_err"], q[4], q[5]))
    a = np.array(out)
    print(f"== {trial}: n={len(a)} t_end={a[-1,0]:.0f}")
    print("gap percentiles (m): p5=%.3f p25=%.3f p50=%.3f p75=%.3f" % tuple(np.percentile(a[:,1],[5,25,50,75])))
    print("EE radius: min=%.3f max=%.3f; EE z: p50=%.3f max=%.3f" % (a[:,2].min(), a[:,2].max(), np.median(a[:,3]), a[:,3].max()))
    ti = np.argmax(a[:,4] > 0.3) if (a[:,4] > 0.3).any() else None
    if ti:
        w = a[max(0,ti-30):ti+5]
        print(f"topple onset t={a[ti,0]:.1f}: pos_err={a[ti,5]:.3f} ang_err={a[ti,6]:.3f}")
        print("  3s before: gap=%.4f ee_z=%.3f ang_err=%.3f pos_err=%.3f" % (w[0,1], w[0,3], w[0,6], w[0,5]))
        print("  gap min in last 3s: %.4f" % w[:,1].min())
    # time in standoff band 0.02-0.06 vs far
    frac_far = (a[:,1] > 0.10).mean(); frac_stand = ((a[:,1] > 0.005) & (a[:,1] < 0.06)).mean()
    print("frac gap>10cm: %.2f; frac 0.5-6cm standoff band: %.2f; frac contact<2mm: %.3f" % (frac_far, frac_stand, (a[:,1]<0.002).mean()))
