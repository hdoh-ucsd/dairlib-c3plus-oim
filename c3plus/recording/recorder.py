"""Coordinate passive recording; no subscriptions or files are opened on import."""
import argparse
import json
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
    p.add_argument('--duration', type=float, default=120.0)
    p.add_argument('--pos-tol', type=float, default=0.05)
    p.add_argument('--ang-tol', type=float, default=0.1)
    p.add_argument('--exit-on-success', action='store_true')
    p.add_argument('--settle', type=float, default=5.0,
                   help='extra seconds recorded after first success')
    p.add_argument('--stop-file', type=Path,
                   help='native completion marker for an explicitly configured execution-step budget')
    return p


def main(argv=None):
    recording_started = time.monotonic()
    from pydrake.lcm import DrakeLcm
    args = parser().parse_args(argv)
    with open(args.out_steps, 'w') as steps_f, open(args.out_trace, 'w') as trace_f:
        recorder = SnapshotRecorder(args, steps_f, trace_f, recording_started)
        lc = DrakeLcm(args.url)
        lc.SubscribeAllChannels(recorder.handle)

        def budget_finished():
            return args.stop_file is not None and args.stop_file.is_file()

        deadline = time.time() + args.duration
        while time.time() < deadline:
            lc.HandleSubscriptions(timeout_millis=500)
            if budget_finished():
                break
            if args.exit_on_success and recorder.state['first_success_t'] is not None:
                t_end = time.time() + args.settle
                while time.time() < t_end:
                    lc.HandleSubscriptions(timeout_millis=200)
                    if budget_finished():
                        break
                break
        steps_f.flush(); trace_f.flush()
        print("FINAL " + json.dumps({
            'first_success_t': recorder.state['first_success_t'], 'control_steps': recorder.state['step'],
            'best_pos_err': recorder.state['best_pos'], 'best_ang_err': recorder.state['best_ang'],
            'termination_reason': ('execution_step_budget' if budget_finished() else
                                   'goal_reached' if args.exit_on_success and recorder.state['first_success_t'] is not None
                                   else 'wall_time_cap'),
            'pos_tol': args.pos_tol, 'ang_tol': args.ang_tol}), flush=True)


if __name__ == '__main__':
    main()
