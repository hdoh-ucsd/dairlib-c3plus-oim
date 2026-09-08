#!/usr/bin/env python3
"""Scene-smoke recorder: one raw row per C3+ control step (C3_DEBUG_CURR msg).

Passive instrumentation only. Subscribes to all channels; on every planner
solve tick it snapshots the latest robot joints and every OBJECT_*_STATE
pose. Errors/costs are computed offline by postprocess_run.py.

Also writes a 10 Hz state_trace.jsonl compatible with the existing 3D
renderer, and prints SUCCESS/FINAL lines like agent_d's record_run.py.
"""
import argparse, json, math, struct, time
from pydrake.lcm import DrakeLcm

p = argparse.ArgumentParser()
p.add_argument('--goal', type=float, nargs=3, required=True)
p.add_argument('--object-name', required=True,
               help='substring identifying the manipulated OBJECT_* channel')
p.add_argument('--out-steps', required=True)
p.add_argument('--out-trace', required=True)
p.add_argument('--url', required=True)
p.add_argument('--duration', type=float, default=120.0)
p.add_argument('--pos-tol', type=float, default=0.05)
p.add_argument('--ang-tol', type=float, default=0.1)
p.add_argument('--exit-on-success', action='store_true')
p.add_argument('--settle', type=float, default=5.0,
               help='extra seconds recorded after first success')
args = p.parse_args()


class Cur:
    def __init__(self, b):
        self.b, self.i = b, 8

    def i32(self):
        v = struct.unpack_from('>i', self.b, self.i)[0]; self.i += 4; return v

    def i64(self):
        v = struct.unpack_from('>q', self.b, self.i)[0]; self.i += 8; return v

    def s(self):
        n = self.i32()
        v = self.b[self.i:self.i + n - 1].decode(); self.i += n; return v

    def dbl(self, n=1):
        v = struct.unpack_from('>%dd' % n, self.b, self.i); self.i += 8 * n
        return list(v)


def decode_object_state(b):
    c = Cur(b)
    utime = c.i64(); name = c.s()
    npos = c.i32(); nvel = c.i32()
    [c.s() for _ in range(npos)]
    pos = c.dbl(npos)
    return utime, name, pos


def decode_robot_output(b):
    c = Cur(b)
    utime = c.i64()
    npos = c.i32(); nvel = c.i32(); neff = c.i32()
    [c.s() for _ in range(npos)]
    pos = c.dbl(npos)
    [c.s() for _ in range(nvel)]
    vel = c.dbl(nvel)
    eff = []
    if neff > 0:
        [c.s() for _ in range(neff)]
        eff = c.dbl(neff)
    return utime, pos, vel, eff


def yaw_of(q):
    return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                      1 - 2 * (q[2] * q[2] + q[3] * q[3]))


S = {'robot': None, 'objects': {}, 'step': 0, 'first_success_t': None,
     'last_trace': -1.0, 'best_pos': 1e9, 'best_ang': 1e9, 'sim_t': 0.0}
steps_f = open(args.out_steps, 'w')
trace_f = open(args.out_trace, 'w')


def handle(channel, data):
    try:
        if channel == 'FRANKA_STATE_SIMULATION':
            S['robot'] = decode_robot_output(data)
        elif channel.startswith('OBJECT_') and channel.endswith('_STATE_SIMULATION'):
            utime, name, pos = decode_object_state(data)
            S['objects'][channel] = (utime, name, pos)
            if args.object_name in channel:
                on_manip(utime, pos)
        elif channel == 'C3_DEBUG_CURR':
            try:
                ut = struct.unpack_from('>q', data, 8)[0]
                S['sim_t'] = ut / 1e6
            except Exception:
                pass
            on_step()
    except Exception:
        pass


def on_manip(utime, posv):
    q = posv[:4]; x, y = posv[4], posv[5]
    t = utime / 1e6
    pos_err = math.hypot(x - args.goal[0], y - args.goal[1])
    ang_err = abs(math.remainder(yaw_of(q) - args.goal[2], 2 * math.pi))
    S['best_pos'] = min(S['best_pos'], pos_err)
    S['best_ang'] = min(S['best_ang'], ang_err)
    if pos_err < args.pos_tol and ang_err < args.ang_tol and S['first_success_t'] is None:
        S['first_success_t'] = t
        print(f"SUCCESS t={t:.2f}", flush=True)
    if S['robot'] is not None and t - S['last_trace'] >= 0.1:
        S['last_trace'] = t
        trace_f.write(json.dumps(
            {'t': t, 'q': S['robot'][1][:7], 'obj': posv[:7],
             'pos_err': pos_err, 'ang_err': ang_err}) + '\n')
        trace_f.flush()


def on_step():
    if S['robot'] is None or not S['objects']:
        return
    S['step'] += 1
    rec = {'control_step': S['step'], 'sim_time': S['sim_t'],
           'robot_q': S['robot'][1], 'robot_v': S['robot'][2],
           'robot_u': S['robot'][3],
           'objects': {ch: v[2] for ch, v in S['objects'].items()}}
    steps_f.write(json.dumps(rec) + '\n')
    if S['step'] % 100 == 0:
        steps_f.flush()


lc = DrakeLcm(args.url)
lc.SubscribeAllChannels(handle)
deadline = time.time() + args.duration
while time.time() < deadline:
    lc.HandleSubscriptions(timeout_millis=500)
    if args.exit_on_success and S['first_success_t'] is not None:
        t_end = time.time() + args.settle
        while time.time() < t_end:
            lc.HandleSubscriptions(timeout_millis=200)
        break
steps_f.flush(); trace_f.flush()
print("FINAL " + json.dumps({
    'first_success_t': S['first_success_t'], 'control_steps': S['step'],
    'best_pos_err': S['best_pos'], 'best_ang_err': S['best_ang'],
    'pos_tol': args.pos_tol, 'ang_tol': args.ang_tol}), flush=True)
