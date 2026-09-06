#!/usr/bin/env python3
"""Agent D run recorder: robot joints + object pose + success scoring.

Self-contained: decodes dairlib lcmt_object_state / lcmt_robot_output by hand
(no generated bindings needed).
"""
import argparse, json, math, struct, time
from pydrake.lcm import DrakeLcm

p = argparse.ArgumentParser()
p.add_argument('--goal', type=float, nargs=3, required=True)
p.add_argument('--out', required=True)
p.add_argument('--url', required=True)
p.add_argument('--duration', type=float, default=600.0)
p.add_argument('--object-channel', default='OBJECT_G_shape_video_STATE_SIMULATION')
p.add_argument('--exit-on-success', action='store_true')
args = p.parse_args()


class Cur:
    def __init__(self, b):
        self.b, self.i = b, 8  # skip 8-byte fingerprint

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
    [c.s() for _ in range(nvel)]
    vel = c.dbl(nvel)
    return utime, name, pos, vel


def decode_robot_output(b):
    c = Cur(b)
    utime = c.i64()
    npos = c.i32(); nvel = c.i32(); neff = c.i32()
    [c.s() for _ in range(npos)]
    pos = c.dbl(npos)
    return utime, pos


lc = DrakeLcm(args.url)
out = open(args.out, 'w')
S = {'q': None, 'first_success_t': None, 'n': 0, 'last_write': -1.0, 'last': None,
     'best_pos': 1e9, 'best_ang': 1e9, 'max_tilt': 0.0}


def on_robot(data):
    S['q'] = decode_robot_output(data)[1][:7]


def on_obj(data):
    utime, name, posv, velv = decode_object_state(data)
    q = posv[:4]; x, y, z = posv[4], posv[5], posv[6]
    yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]))
    sinp = max(-1.0, min(1.0, 2 * (q[0] * q[2] - q[3] * q[1])))
    pitch = math.asin(sinp)
    roll = math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
    tilt = max(abs(roll), abs(pitch))
    t = utime / 1e6
    pos_err = math.hypot(x - args.goal[0], y - args.goal[1])
    ang_err = abs(math.remainder(yaw - args.goal[2], 2 * math.pi))
    S['n'] += 1
    S['best_pos'] = min(S['best_pos'], pos_err)
    if pos_err == S['best_pos']:
        S['best_ang_at_best_pos'] = ang_err
    S['best_ang'] = min(S['best_ang'], ang_err)
    S['max_tilt'] = max(S['max_tilt'], tilt)
    rec = {'t': t, 'q': S['q'], 'obj': list(q) + [x, y, z],
           'pos_err': pos_err, 'ang_err': ang_err, 'tilt': tilt}
    S['last'] = rec
    if S['q'] is not None and t - S['last_write'] >= 0.1:
        out.write(json.dumps(rec) + '\n'); out.flush(); S['last_write'] = t
    if pos_err < 0.02 and ang_err < 0.1 and S['first_success_t'] is None:
        S['first_success_t'] = t
        print(f"SUCCESS t={t:.2f}", flush=True)


lc.Subscribe('FRANKA_STATE_SIMULATION', on_robot)
lc.Subscribe(args.object_channel, on_obj)
deadline = time.time() + args.duration
while time.time() < deadline:
    lc.HandleSubscriptions(timeout_millis=500)
    if args.exit_on_success and S['first_success_t'] is not None:
        t_end = time.time() + 5.0
        while time.time() < t_end:
            lc.HandleSubscriptions(timeout_millis=200)
        break
last = dict(S['last'] or {})
last.pop('q', None)
print("FINAL " + json.dumps({
    'first_success_t': S['first_success_t'], 'samples': S['n'],
    'best_pos_err': S['best_pos'], 'best_ang_err': S['best_ang'],
    'max_tilt': S['max_tilt'], **last}), flush=True)
