#!/usr/bin/env python3
"""Derive outer_loop_decisions.jsonl and reposition_events.jsonl from the
synchronized cycle index (passive; existing logs only)."""
import csv, json, math
OUT = '/root/push_anything_ADMM/results/parallel_c3plus_audit/agent_a_stepwise'
REASON = {'0': 'kNoSwitch', '1': 'kToC3Cost', '2': 'kToC3ReachedReposTarget',
          '3': 'kToReposCost', '4': 'kToReposUnproductive', '5': 'kToC3Xbox'}
rows = list(csv.DictReader(open(OUT + '/synchronized_cycle_index.csv')))
dec = open(OUT + '/outer_loop_decisions.jsonl', 'w')
rep = open(OUT + '/reposition_events.jsonl', 'w')
prev = None
seg = None
for r in rows:
    if prev is not None and r['outer_loop_mode'] != prev['outer_loop_mode']:
        dec.write(json.dumps(dict(
            planner_cycle_id=int(r['planner_cycle_id']), sim_time=float(r['sim_time']),
            prev_mode=prev['outer_loop_mode'], new_mode=r['outer_loop_mode'],
            transition_reason=REASON.get(r['mode_switch_reason'], r['mode_switch_reason']),
            selected_candidate_id=r['selected_candidate_id'],
            num_candidates=int(r['num_candidates']),
            pusher_object_gap=r['pusher_object_gap'])) + '\n')
    if r['outer_loop_mode'] == 'REPOS':
        if seg is None:
            seg = dict(start_cycle=int(r['planner_cycle_id']), start_t=float(r['sim_time']),
                       target=(r['ee_x'], r['ee_y']))
        seg['end_cycle'] = int(r['planner_cycle_id']); seg['end_t'] = float(r['sim_time'])
        seg['end_gap'] = r['pusher_object_gap']
    elif seg is not None:
        seg['end_reason'] = REASON.get(r['mode_switch_reason'], r['mode_switch_reason'])
        seg['duration_s'] = round(seg['end_t'] - seg['start_t'], 3)
        seg['contact_acquired_at_end'] = (seg['end_gap'] != '' and float(seg['end_gap']) < 0.004)
        rep.write(json.dumps(seg) + '\n')
        seg = None
    prev = r
dec.close(); rep.close()
print('decision + reposition traces written')
