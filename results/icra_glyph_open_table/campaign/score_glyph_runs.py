#!/usr/bin/env python3
"""Score ICRA glyph open-table trials from state_trace.jsonl.

Success = pos_err<=0.05 AND ang_err<=0.1 (protocol tolerance), reported both
as first-hit (latched) and final-state. Also min errors, topple flag
(roll/pitch>0.3 rad), trace duration.
"""
import json, math, sys, glob, os, csv

def score(path):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    if not rows: return None
    t_hit = None; min_pos = 1e9; min_ang = 1e9; min_joint = (1e9, None); topple = False
    for r in rows:
        p, a = r["pos_err"], r["ang_err"]
        min_pos = min(min_pos, p); min_ang = min(min_ang, a)
        if p <= 0.05 and a <= 0.1 and t_hit is None: t_hit = r["t"]
        if p + a < min_joint[0]: min_joint = (p + a, r)
        w, x, y, z = r["obj"][:4]
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1, min(1, 2*(w*y-z*x))))
        if abs(roll) > 0.3 or abs(pitch) > 0.3: topple = True
    f = rows[-1]
    return dict(dur=f["t"], t_success=t_hit,
                final_pos=f["pos_err"], final_ang=f["ang_err"],
                final_pass=f["pos_err"] <= 0.05 and f["ang_err"] <= 0.1,
                min_pos=min_pos, min_ang=min_ang, topple=topple)

def main(root, out_csv):
    w = csv.writer(open(out_csv, "w"))
    w.writerow(["glyph","trial","dur_s","t_success_s","final_pos","final_ang",
                "final_pass","min_pos","min_ang","topple"])
    for tr in sorted(glob.glob(os.path.join(root, "*", "trial*", "state_trace.jsonl"))):
        g = tr.split(os.sep)[-3]; trial = tr.split(os.sep)[-2]
        s = score(tr)
        if s is None:
            w.writerow([g, trial] + ["EMPTY"]*8); continue
        w.writerow([g, trial, f'{s["dur"]:.1f}',
                    "" if s["t_success"] is None else f'{s["t_success"]:.1f}',
                    f'{s["final_pos"]:.4f}', f'{s["final_ang"]:.4f}', s["final_pass"],
                    f'{s["min_pos"]:.4f}', f'{s["min_ang"]:.4f}', s["topple"]])
    print(open(out_csv).read())

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
