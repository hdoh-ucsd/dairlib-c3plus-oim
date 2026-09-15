"""Coordinate passive recording; no subscriptions or files are opened on import."""
import argparse
import json
import math
from pathlib import Path
import time

from .snapshots import SnapshotRecorder

def parser():
    p = argparse.ArgumentParser()
    p.add_argument('--goal', type=float, nargs=3, required=True)
    p.add_argument('--object-name', required=True,
                   help='substring identifying the manipulated OBJECT_* channel')
    p.add_argument('--out-steps', required=True)
    p.add_argument('--out-trace', required=True)
    p.add_argument('--url', required=True)
    p.add_argument('--duration', type=float, default=120.0,
                   help='elapsed simulation seconds from the first simulator state')
    p.add_argument('--pos-tol', type=float, default=0.05)
    p.add_argument('--ang-tol', type=float, default=0.1)
    p.add_argument('--exit-on-success', action='store_true')
    p.add_argument('--settle', type=float, default=0.0,
                   help='extra wall-clock seconds after first success, bounded by --duration')
    p.add_argument('--stop-file', type=Path,
                   help='native completion marker for goal success or execution-step budget')
    return p


def main(argv=None):
    recording_started = time.monotonic()
    from pydrake.lcm import DrakeLcm
    p = parser()
    args = p.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        p.error('--duration must be finite and positive')
    if not math.isfinite(args.settle) or args.settle < 0:
        p.error('--settle must be finite and nonnegative')
    with open(args.out_steps, 'w') as steps_f, open(args.out_trace, 'w') as trace_f:
        recorder = SnapshotRecorder(args, steps_f, trace_f, recording_started)
        lc = DrakeLcm(args.url)
        lc.SubscribeAllChannels(recorder.handle)

        def native_completion():
            if args.stop_file is None or not args.stop_file.is_file():
                return None
            marker = json.loads(args.stop_file.read_text())
            if marker.get('termination_reason') not in ('step_budget', 'goal_reached'):
                raise ValueError('Unknown native completion reason')
            return marker

        settle_deadline = None
        while True:
            lc.HandleSubscriptions(timeout_millis=500)
            completed = native_completion()
            if completed is not None:
                termination_reason = completed['termination_reason']
                if termination_reason == 'goal_reached':
                    reached = completed.get('sim_time')
                    if not isinstance(reached, (int, float)) or not math.isfinite(reached) or reached < 0:
                        raise ValueError('Goal completion requires a finite simulation timestamp')
                    recorder.state['first_success_t'] = reached
                    recorder.simulation_time = reached
                    snapshot = completed.get('snapshot')
                    if snapshot is not None:
                        recorder.on_native_goal(snapshot)
                    print(f"SUCCESS t={reached:.6f} source=native_simulation", flush=True)
                else:
                    termination_reason = 'execution_step_budget'
                break
            elapsed = recorder.elapsed_simulation_time
            if elapsed is not None and elapsed >= args.duration:
                termination_reason = 'simulation_time_cap'
                break
            if args.exit_on_success and recorder.state['first_success_t'] is not None:
                if settle_deadline is None:
                    settle_deadline = time.monotonic() + args.settle
                if time.monotonic() >= settle_deadline:
                    termination_reason = 'goal_reached'
                    break
        steps_f.flush(); trace_f.flush()
        print("FINAL " + json.dumps({
            'success': recorder.state['first_success_t'] is not None,
            'first_success_t': recorder.state['first_success_t'], 'control_steps': recorder.state['step'],
            'best_pos_err': recorder.state['best_pos'], 'best_ang_err': recorder.state['best_ang'],
            'termination_reason': termination_reason,
            'simulation_time_start': recorder.simulation_time_start,
            'simulation_time_end': recorder.simulation_time,
            'elapsed_simulation_time': recorder.elapsed_simulation_time,
            'pos_tol': args.pos_tol, 'ang_tol': args.ang_tol}), flush=True)


if __name__ == '__main__':
    main()
