"""Passive asynchronous debug snapshots and replay trace bookkeeping."""
import json
import math
from pathlib import Path
import struct
import time

from c3plus.runtime.lcm import decode_object_state, decode_robot_output

def yaw_of(q):
    return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                      1 - 2 * (q[2] * q[2] + q[3] * q[3]))


def progress_label(object_name, steps_path):
    # New managed runs already record their canonical public identity. Retain
    # the model/channel fallback below for archived or direct native runs.
    try:
        runtime = json.loads((Path(steps_path).parent / "runtime_status.json").read_text())
        if isinstance(runtime.get("object_name"), str) and runtime["object_name"]:
            return runtime["object_name"]
    except (OSError, ValueError, AttributeError):
        pass
    if object_name in {f"{name}_base" for name in ("sugar_box", "power_drill", "hammer", "banana")}:
        return object_name.removesuffix("_base")
    known = {"T_shape_video": "T_block", "vertical_link": "T_block", "c_glyph_base": "Cblock"}
    if object_name in known:
        return known[object_name]
    # Both legacy scene objects share the G_shape_video channel. Only the
    # saved simulation model can distinguish them without guessing the scene.
    if object_name == "G_shape_video":
        import yaml
        try:
            saved = yaml.safe_load((Path(steps_path).parent / "config/simulation.yaml").read_text())
            model = Path(saved["object_model"]).name
            return {"push_t_oimscale_m01.sdf": "T_block", "push_c_glyph.sdf": "Cblock"}.get(model, object_name)
        except (OSError, KeyError, TypeError, yaml.YAMLError):
            pass
    return object_name


class SnapshotRecorder:
    def __init__(self, args, steps_f, trace_f, recording_started):
        self.args = args
        self.steps_f = steps_f
        self.trace_f = trace_f
        self.recording_started = recording_started
        self.simulation_time_start = None
        self.simulation_time = None
        self.label = progress_label(args.object_name, args.out_steps)
        self.state = {'robot': None, 'objects': {}, 'step': 0, 'first_success_t': None,
             'last_trace': -1.0, 'best_pos': 1e9, 'best_ang': 1e9, 'sim_t': 0.0}

    def handle(self, channel, data):
        try:
            if channel == 'FRANKA_STATE_SIMULATION':
                self.state['robot'] = decode_robot_output(data)
                # Use the simulator clock, independently of planner/debug events.
                t = self.state['robot'][0] / 1e6
                if math.isfinite(t) and t >= 0:
                    if self.simulation_time_start is None:
                        self.simulation_time_start = t
                    if self.simulation_time is None or t > self.simulation_time:
                        self.simulation_time = t
            elif channel.startswith('OBJECT_') and channel.endswith('_STATE_SIMULATION'):
                utime, name, pos = decode_object_state(data)
                self.state['objects'][channel] = (utime, name, pos)
                if self.args.object_name in channel:
                    self.on_manip(utime, pos)
            elif channel == 'C3_DEBUG_CURR':
                try:
                    ut = struct.unpack_from('>q', data, 8)[0]
                    self.state['sim_t'] = ut / 1e6
                except Exception:
                    pass
                self.on_step()
        except Exception:
            pass

    @property
    def elapsed_simulation_time(self):
        if self.simulation_time_start is None:
            return None
        return self.simulation_time - self.simulation_time_start

    def on_manip(self, utime, posv):
        q = posv[:4]; x, y = posv[4], posv[5]
        t = utime / 1e6
        pos_err = math.hypot(x - self.args.goal[0], y - self.args.goal[1])
        ang_err = abs(math.remainder(yaw_of(q) - self.args.goal[2], 2 * math.pi))
        self.state['best_pos'] = min(self.state['best_pos'], pos_err)
        self.state['best_ang'] = min(self.state['best_ang'], ang_err)
        if pos_err < self.args.pos_tol and ang_err < self.args.ang_tol and self.state['first_success_t'] is None:
            self.state['first_success_t'] = t
            print(f"SUCCESS t={t:.2f}", flush=True)
        if self.state['robot'] is not None and t - self.state['last_trace'] >= 0.1:
            self.state['last_trace'] = t
            self.trace_f.write(json.dumps(
                {'t': t, 'q': self.state['robot'][1][:7], 'obj': posv[:7],
                 'pos_err': pos_err, 'ang_err': ang_err}) + '\n')
            self.trace_f.flush()

    def on_step(self):
        if self.state['robot'] is None or not self.state['objects']:
            return
        self.state['step'] += 1
        rec = {'control_step': self.state['step'], 'sim_time': self.state['sim_t'],
               'robot_q': self.state['robot'][1], 'robot_v': self.state['robot'][2],
               'robot_u': self.state['robot'][3],
               'objects': {ch: v[2] for ch, v in self.state['objects'].items()}}
        self.steps_f.write(json.dumps(rec) + '\n')
        if self.state['step'] % 100 == 0:
            self.steps_f.flush()
        if self.state['step'] % 10 == 0:
            selected = next((state for channel, state in self.state['objects'].items()
                             if self.args.object_name in channel), None)
            if selected is not None:
                pose = selected[2]
                pos_err = math.hypot(pose[4] - self.args.goal[0], pose[5] - self.args.goal[1])
                ang_err = abs(math.remainder(yaw_of(pose[:4]) - self.args.goal[2], 2 * math.pi))
                within_goal = pos_err < self.args.pos_tol and ang_err < self.args.ang_tol
                print(f"[{self.label}] step={self.state['step']:04d} sim={self.state['sim_t']:.2f}s "
                      f"wall={time.monotonic() - self.recording_started:.1f}s pos_err={pos_err:.3f}m "
                      f"yaw_err={math.degrees(ang_err):.1f}deg within_goal={'yes' if within_goal else 'no'}",
                      flush=True)

    def on_native_goal(self, snapshot):
        """Retain the exact reached state even before the first debug message.

        control_step remains a snapshot index, not an executed-policy counter.
        Native terminal effort is unavailable and is left unrecorded.
        """
        self.state['step'] += 1
        record = dict(snapshot, control_step=self.state['step'])
        self.steps_f.write(json.dumps(record) + '\n')
        pose = next(value for channel, value in snapshot['objects'].items()
                    if self.args.object_name in channel)
        norm = math.hypot(*pose[:4])
        yaw = yaw_of([value / norm for value in pose[:4]])
        pos_err = math.hypot(pose[4]-self.args.goal[0], pose[5]-self.args.goal[1])
        ang_err = abs(math.remainder(yaw-self.args.goal[2], 2*math.pi))
        self.state['best_pos'] = min(self.state['best_pos'], pos_err)
        self.state['best_ang'] = min(self.state['best_ang'], ang_err)
        self.trace_f.write(json.dumps(dict(
            t=snapshot['sim_time'], q=snapshot['robot_q'], obj=pose,
            pos_err=pos_err, ang_err=ang_err, source='native_goal_terminal')) + '\n')
        self.steps_f.flush()
        self.trace_f.flush()
