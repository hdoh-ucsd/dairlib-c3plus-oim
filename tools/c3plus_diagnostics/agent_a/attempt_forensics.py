#!/usr/bin/env python3
"""Agent A stepwise forensics: segment early push attempts of a run, build the
synchronized cycle index, extract per-attempt C3 input/solution data, and prove
whether the repeated attempts solve the same problem / produce the same inputs.
Passive analysis of existing run logs only — no controller interaction."""
import csv, json, math, os, sys
import numpy as np

PR = 0.0125
# T footprint boxes in object frame (matches controller TFootprint()).
BOXES = [(0.0, 0.0099, 0.0445, 0.0099), (0.0, -0.0397, 0.0099, 0.0397)]

def footprint_gap(ex, ey, ox, oy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * (ex - ox) + s * (ey - oy)
    ly = -s * (ex - ox) + c * (ey - oy)
    best = 1e9
    for (cx, cy, hx, hy) in BOXES:
        dx, dy = abs(lx - cx) - hx, abs(ly - cy) - hy
        d = math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 or dy > 0) else max(dx, dy)
        best = min(best, d)
    return best - PR

def load(run):
    cyc = list(csv.DictReader(open(run + '/controller_cycle_costs.csv')))
    ee = {}
    for r in csv.DictReader(open(run + '/candidate_ranking_costs.csv')):
        if r['candidate_id'] == '0':
            ee[int(r['event_id'])] = (float(r['ee_x']), float(r['ee_y']), float(r['ee_z']))
    return cyc, ee

def segment(run, t_max, out_dir, tag):
    cyc, ee = load(run)
    n = len(cyc)
    rows = []
    for k, r in enumerate(cyc):
        t = float(r['time'])
        if t > t_max: break
        e = ee.get(k, (float('nan'),) * 3)
        ox, oy, yaw = float(r['obj_x']), float(r['obj_y']), float(r['obj_yaw'])
        gap = footprint_gap(e[0], e[1], ox, oy, yaw) if not math.isnan(e[0]) else float('nan')
        rows.append(dict(cycle=k, t=t, mode=int(r['is_c3_mode']), reason=r['mode_switch_reason'],
                         sel=r['selected_id'], sel_ee=(float(r['sel_ee_x']), float(r['sel_ee_y'])),
                         obj=(ox, oy, yaw), ee=e, gap=gap,
                         ncand=int(r['num_candidates']), fin=r['finished_reposition_flag']))
    # attempts: segment on mode transitions; a push attempt = contiguous C3 span,
    # a reposition event = contiguous repos span. Attach the preceding reposition
    # to the following push as one transaction.
    segs = []
    cur = [rows[0]]
    for r in rows[1:]:
        if r['mode'] != cur[-1]['mode']:
            segs.append(cur); cur = [r]
        else:
            cur.append(r)
    segs.append(cur)
    # synchronized index
    with open(f'{out_dir}/synchronized_cycle_index.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['planner_cycle_id', 'sim_time', 'outer_loop_mode', 'mode_switch_reason',
                    'selected_candidate_id', 'push_attempt_id', 'reposition_event_id',
                    'obj_x', 'obj_y', 'obj_yaw', 'ee_x', 'ee_y', 'ee_z', 'pusher_object_gap',
                    'num_candidates', 'finished_reposition_flag'])
        pa, re_ = 0, 0
        seg_of = {}
        for si, sg in enumerate(segs):
            if sg[0]['mode'] == 1: pa += 1
            else: re_ += 1
            for r in sg:
                seg_of[r['cycle']] = (si, pa if sg[0]['mode'] == 1 else '', re_ if sg[0]['mode'] == 0 else '')
                w.writerow([r['cycle'], r['t'], 'C3' if r['mode'] else 'REPOS', r['reason'], r['sel'],
                            seg_of[r['cycle']][1], seg_of[r['cycle']][2],
                            *r['obj'], *r['ee'], round(r['gap'], 5) if not math.isnan(r['gap']) else '',
                            r['ncand'], r['fin']])
    # attempt table: for each C3 segment measure object motion + contact fraction
    att = []
    pa = 0
    for si, sg in enumerate(segs):
        if sg[0]['mode'] != 1: continue
        pa += 1
        o0, o1 = sg[0]['obj'], sg[-1]['obj']
        dxy = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
        dyaw = math.remainder(o1[2] - o0[2], 2 * math.pi)
        gaps = [r['gap'] for r in sg if not math.isnan(r['gap'])]
        contact_frac = sum(1 for g in gaps if g < 0.004) / max(1, len(gaps))
        nxt = segs[si + 1][0]['reason'] if si + 1 < len(segs) else 'run_continues'
        # contact sector: object-frame angle of EE at segment midpoint
        m = sg[len(sg) // 2]
        c, s = math.cos(m['obj'][2]), math.sin(m['obj'][2])
        lx = c * (m['ee'][0] - m['obj'][0]) + s * (m['ee'][1] - m['obj'][1])
        ly = -s * (m['ee'][0] - m['obj'][0]) + c * (m['ee'][1] - m['obj'][1])
        att.append(dict(attempt=pa, t_start=sg[0]['t'], t_end=sg[-1]['t'],
                        cycles=len(sg), sel_at_start=sg[0]['sel'],
                        sector_deg=round(math.degrees(math.atan2(ly, lx)), 1),
                        obj_frame_ee=(round(lx, 4), round(ly, 4)),
                        contact_frac=round(contact_frac, 2),
                        min_gap=round(min(gaps), 4) if gaps else '',
                        d_xy_mm=round(1000 * dxy, 1), d_yaw_deg=round(math.degrees(dyaw), 1),
                        end_reason=nxt))
    with open(f'{out_dir}/early_attempts_{tag}.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(att[0]))
        w.writeheader(); [w.writerow(a) for a in att]
    return att, rows

def extract_solutions(run, attempts, out_dir, tag):
    """Per attempt: selected-candidate C3 solution at the attempt's first cycles."""
    want = {}
    for a in attempts:
        want[a['attempt']] = (a['t_start'], a['t_end'])
    sols = {}
    for line in open(run + '/selected_candidate_qp_variables.jsonl'):
        try: j = json.loads(line)
        except Exception: continue
        e = j['event_id']
        sols.setdefault(e, {})[j['k']] = j
    # map event->time via cycle csv row index (event_id == cycle index)
    cyc = list(csv.DictReader(open(run + '/controller_cycle_costs.csv')))
    per_attempt = {}
    for a in attempts:
        # first event of the attempt that has a full solution
        for e in range(int(a['t_start'] * 0), len(cyc)):
            if float(cyc[e]['time']) < a['t_start']: continue
            if float(cyc[e]['time']) > a['t_end']: break
            if e in sols and len(sols[e]) >= 5:
                ks = sols[e]
                per_attempt[a['attempt']] = dict(
                    event=e, t=float(cyc[e]['time']),
                    x0=ks[0]['x'], x_des=ks[0]['x_des'],
                    U=[ks[k]['u'] for k in range(5)],
                    Xee=[ks[k]['x'][0:3] for k in range(5)],
                    Xobj=[ks[k]['x'][7:10] for k in range(5)],
                    Lam=[ks[k]['lambda'] for k in range(5)])
                break
    json.dump(per_attempt, open(f'{out_dir}/c3_candidate_solutions_{tag}.json', 'w'))
    # pairwise similarity
    rows = []
    ids = sorted(per_attempt)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            A, B_ = per_attempt[ids[i]], per_attempt[ids[j]]
            U1, U2 = np.array(A['U']), np.array(B_['U'])
            x01, x02 = np.array(A['x0']), np.array(B_['x0'])
            sg1, sg2 = np.array(A['x_des']), np.array(B_['x_des'])
            l1 = np.array([v[:8] for v in A['Lam']]); l2 = np.array([v[:8] for v in B_['Lam']])
            dU_inf = float(np.max(np.abs(U1 - U2))); dU_2 = float(np.linalg.norm(U1 - U2))
            dee = float(np.max(np.abs(np.array(A['Xee']) - np.array(B_['Xee']))))
            dob = float(np.max(np.abs(np.array(A['Xobj']) - np.array(B_['Xobj']))))
            dl = float(np.max(np.abs(l1 - l2)))
            rel = dU_inf / max(1e-12, float(np.max(np.abs(U1))))
            cls = ('EXACTLY_IDENTICAL' if dU_inf == 0.0 else
                   'NUMERICALLY_IDENTICAL' if dU_inf < 1e-9 else
                   'NEAR_IDENTICAL' if rel < 1e-3 else 'BEHAVIORALLY_DISTINCT')
            rows.append(dict(a=ids[i], b=ids[j], dU_inf=dU_inf, dU_2=dU_2,
                             dXee_inf=dee, dXobj_inf=dob, dLam_inf=dl,
                             dx0_inf=float(np.max(np.abs(x01 - x02))),
                             dsubgoal_inf=float(np.max(np.abs(sg1 - sg2))),
                             classification=cls))
    with open(f'{out_dir}/input_sequence_similarity_{tag}.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ['a'])
        w.writeheader(); [w.writerow(r) for r in rows]
    return per_attempt, rows

if __name__ == '__main__':
    run, t_max, out_dir, tag = sys.argv[1], float(sys.argv[2]), sys.argv[3], sys.argv[4]
    os.makedirs(out_dir, exist_ok=True)
    att, rows = segment(run, t_max, out_dir, tag)
    for a in att: print(a)
    per, sim = extract_solutions(run, att, out_dir, tag)
    for r in sim: print({k: (round(v, 6) if isinstance(v, float) else v) for k, v in r.items()})
