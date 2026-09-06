#!/usr/bin/env python3
"""Extract the N_closest active-obstacle trace (§14) from a draw dir."""
import csv, sys, os, json

draw, out_csv = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(f"{draw}/obstacle_lcs_contacts.csv")))
recs = {}
for line in open(f"{draw}/state_trace.jsonl"):
    try:
        r = json.loads(line)
        recs[round(r["t"], 1)] = r["obj"][4:6]
    except Exception:
        pass
by_t = {}
for r in rows:
    t = float(r["time"])
    by_t.setdefault(t, {})[int(r["slot"])] = r
out = []
prev_ids = None
n_switches = 0
for t in sorted(by_t):
    slots = by_t[t]
    ids = tuple(slots.get(i, {}).get("obstacle_id", "-") for i in (0, 1))
    if prev_ids is not None and ids != prev_ids:
        n_switches += 1
    prev_ids = ids
    xy = recs.get(round(t, 1), ["", ""])
    out.append(dict(time=t, obj_x=xy[0], obj_y=xy[1],
                    active_obstacle_1=ids[0], active_obstacle_2=ids[1],
                    phi_1=slots.get(0, {}).get("phi", ""),
                    phi_2=slots.get(1, {}).get("phi", "")))
new = not os.path.exists(out_csv)
with open(out_csv, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
    if new:
        w.writeheader()
    w.writerows(out)
ids_seen = sorted(set(r["active_obstacle_1"] for r in out) | set(r["active_obstacle_2"] for r in out))
print(json.dumps(dict(ticks=len(out), identity_switches=n_switches, ids_seen=ids_seen)))
