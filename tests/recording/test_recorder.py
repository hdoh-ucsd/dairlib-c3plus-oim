from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from c3plus.configs.paths import REPO
from c3plus.recording import recorder as Recorder


class WorkflowTests(unittest.TestCase):
    def test_native_terminal_is_saved_without_debug_messages(self):
        from c3plus.recording.snapshots import SnapshotRecorder
        args = types.SimpleNamespace(object_name='hammer_base', out_steps='/tmp/missing/steps.jsonl',
                                     goal=[.4, -.2, 0])
        steps, trace = io.StringIO(), io.StringIO()
        recorder = SnapshotRecorder(args, steps, trace, 0)
        snapshot = dict(source='native_goal_terminal', sim_time=.2,
                        robot_q=[1, 2, 3, 4, 5], robot_v=[0]*5,
                        objects={'OBJECT_hammer_base_STATE_SIMULATION': [1, 0, 0, 0, .39, -.2, .03]})
        recorder.on_native_goal(snapshot)
        recorded = json.loads(steps.getvalue())
        self.assertEqual(recorded, dict(snapshot, control_step=1))
        self.assertNotIn('robot_u', recorded)
        frame = json.loads(trace.getvalue())
        self.assertEqual(frame['t'], .2)
        self.assertEqual(frame['obj'], next(iter(snapshot['objects'].values())))
        self.assertAlmostEqual(frame['pos_err'], .01)
        self.assertEqual(frame['ang_err'], 0)

    def clock_run(self, ticks, *flags):
        """Deliver simulator states without any C3 debug/plan messages."""
        clock = [100.0]
        consumed = []
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            stop = directory / 'stop.json'

            class FakeLcm:
                def __init__(self, url):
                    self.pending = iter(ticks)

                def SubscribeAllChannels(self, callback):
                    self.callback = callback

                def HandleSubscriptions(self, timeout_millis):
                    tick = next(self.pending)  # Exhaustion exposes a missed stop.
                    consumed.append(tick)
                    clock[0], sim, success, stopped = tick
                    if sim is not None:
                        self.callback('FRANKA_STATE_SIMULATION', (sim, [], [], []))
                    if success:
                        self.callback('OBJECT_hammer_base_STATE_SIMULATION',
                                      (sim, 'hammer_base', [1, 0, 0, 0, 0, 0, 0]))
                    if stopped:
                        stop.write_text(json.dumps(stopped if isinstance(stopped, dict) else
                                                   {'termination_reason': 'step_budget'}))

            module = types.ModuleType('pydrake.lcm')
            module.DrakeLcm = FakeLcm
            output = io.StringIO()
            with patch.dict(sys.modules, {'pydrake.lcm': module}), \
                    patch('c3plus.recording.snapshots.decode_robot_output', side_effect=lambda x: x), \
                    patch('c3plus.recording.snapshots.decode_object_state', side_effect=lambda x: x), \
                    patch('time.monotonic', side_effect=lambda: clock[0]), \
                    patch('time.time', side_effect=AssertionError('Wall clock used for simulation cap')), \
                    redirect_stdout(output):
                Recorder.main(['--goal', '0', '0', '0', '--object-name', 'hammer_base',
                               '--out-steps', str(directory / 'steps.jsonl'),
                               '--out-trace', str(directory / 'trace.jsonl'),
                               '--url', 'memq://test', '--duration', '2',
                               '--stop-file', str(stop), *flags])
        final, = [json.loads(line[6:]) for line in output.getvalue().splitlines()
                  if line.startswith('FINAL ')]
        return final, len(consumed)

    def test_simulation_cap_excludes_startup_pause_and_stale_timestamps(self):
        ticks = [(10000, None, False, False), (20000, 40_000_000, False, False),
                 (30000, 40_000_000, False, False), (40000, 39_000_000, False, False),
                 (50000, 41_000_000, False, False), (60000, 42_000_000, False, False)]
        result, n = self.clock_run(ticks)
        self.assertEqual(n, 6)
        self.assertEqual(result['termination_reason'], 'simulation_time_cap')
        self.assertEqual(result['simulation_time_start'], 40)
        self.assertEqual(result['elapsed_simulation_time'], 2)
        self.assertEqual(result['control_steps'], 0)

    def test_fast_simulation_reaches_cap_before_wall_deadline(self):
        result, n = self.clock_run([(100, 0, False, False), (100.1, 2_000_000, False, False)])
        self.assertEqual(n, 2)
        self.assertEqual(result['termination_reason'], 'simulation_time_cap')

    def test_simulation_cap_applies_during_success_settling(self):
        result, n = self.clock_run([(100, 0, True, False), (101, 2_000_000, False, False)],
                                   '--exit-on-success', '--settle', '5')
        self.assertEqual(n, 2)
        self.assertEqual(result['termination_reason'], 'simulation_time_cap')
        self.assertEqual(result['first_success_t'], 0)

    def test_success_settling_still_uses_wall_seconds(self):
        result, n = self.clock_run([(100, 0, True, False), (104, 500_000, False, False),
                                   (105, 1_000_000, False, False)], '--exit-on-success', '--settle', '5')
        self.assertEqual(n, 3)
        self.assertEqual(result['termination_reason'], 'goal_reached')
        self.assertEqual(result['elapsed_simulation_time'], 1)

    def test_success_exits_immediately_without_default_settling(self):
        result, n = self.clock_run([(100, 0, True, False)], '--exit-on-success')
        self.assertEqual(n, 1)
        self.assertTrue(result['success'])
        self.assertEqual(result['termination_reason'], 'goal_reached')

    def test_native_goal_marker_marks_success_without_async_goal_observation(self):
        marker = {'termination_reason': 'goal_reached', 'sim_time': 1.01}
        result, n = self.clock_run([(101, 1_000_000, False, marker)])
        self.assertEqual(n, 1)
        self.assertTrue(result['success'])
        self.assertEqual(result['termination_reason'], 'goal_reached')
        self.assertEqual(result['first_success_t'], 1.01)
        self.assertEqual(result['simulation_time_end'], 1.01)

    def test_step_budget_can_stop_before_first_simulator_state(self):
        result, n = self.clock_run([(101, None, False, True)])
        self.assertEqual(n, 1)
        self.assertEqual(result['termination_reason'], 'execution_step_budget')
        self.assertIsNone(result['elapsed_simulation_time'])

    def test_import_does_not_parse_arguments_open_outputs_or_load_drake(self):
        code = """
import sys
sys.path.insert(0, sys.argv[1])
sys.argv = ["recorder", "--invalid-import-time-argument"]
from c3plus.recording import recorder
assert callable(recorder.main)
assert not any(name == "pydrake" or name.startswith("pydrake.") for name in sys.modules)
assert "numpy" not in sys.modules
"""
        with tempfile.TemporaryDirectory() as tmp:
            completed = subprocess.run(
                [sys.executable, "-B", "-S", "-c", code, str(REPO)],
                cwd=tmp, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual(completed.stdout, "")
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_recorder_progress_every_ten_steps_reports_current_goal_and_keeps_all_rows(self):
        goal = [.4, -.3, math.pi - .025]
        channel = "OBJECT_sugar_box_base_STATE_SIMULATION"
        samples = []
        for step in range(1, 31):
            yaw = goal[2] + .2 if 11 <= step <= 20 else -math.pi + .025
            x = goal[0] + (.2 if step > 20 else 0)
            samples.append((step * 20000,
                            [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2), x, goal[1], -.029]))

        def encode_string(value):
            encoded = value.encode() + b"\0"
            return struct.pack(">i", len(encoded)) + encoded

        def robot_message(utime):
            header = b"\0" * 8 + struct.pack(">qiii", utime, 5, 5, 5)
            names = b"".join(encode_string(f"joint_{index}") for index in range(5))
            return header + names + struct.pack(">5d", *range(5)) + names + \
                struct.pack(">5d", *([.1] * 5)) + names + struct.pack(">5d", *([.2] * 5))

        def object_message(utime, pose):
            return (b"\0" * 8 + struct.pack(">q", utime) + encode_string("sugar_box_base")
                    + struct.pack(">ii", 7, 6)
                    + b"".join(encode_string(f"q{index}") for index in range(7))
                    + struct.pack(">7d", *pose))

        class Clock:
            wall = 10000.0
            monotonic = 1000.0

        clock = Clock()

        class FakeLcm:
            def __init__(self, url):
                self.index = 0

            def SubscribeAllChannels(self, callback):
                self.callback = callback

            def HandleSubscriptions(self, timeout_millis):
                if self.index >= len(samples):
                    raise AssertionError("Recorder did not stop at its synthetic deadline")
                utime, pose = samples[self.index]
                self.index += 1
                clock.monotonic = 1000.0 + .12 * self.index
                # A wall-clock adjustment must not change the elapsed display.
                clock.wall = 10000.0 + .2 * self.index - (3600 if self.index >= 15 else 0)
                self.callback("FRANKA_STATE_SIMULATION", robot_message(utime))
                self.callback(channel, object_message(utime, pose))
                self.callback("C3_DEBUG_CURR", b"\0" * 8 + struct.pack(">q", utime))
                if self.index == len(samples):
                    clock.wall, clock.monotonic = 20000.0, 2000.0

        pydrake = types.ModuleType("pydrake")
        lcm = types.ModuleType("pydrake.lcm")
        lcm.DrakeLcm = FakeLcm
        pydrake.lcm = lcm
        recorder = REPO / "c3plus/recording/recorder.py"
        with tempfile.TemporaryDirectory() as tmp:
            steps_file, trace_file = Path(tmp) / "steps.jsonl", Path(tmp) / "trace.jsonl"
            argv = [str(recorder), "--goal", *map(str, goal), "--object-name", "sugar_box_base",
                    "--out-steps", str(steps_file), "--out-trace", str(trace_file),
                    "--url", "memq://fake", "--duration", "0.57"]
            output = io.StringIO()
            with patch.dict(sys.modules, {"pydrake": pydrake, "pydrake.lcm": lcm}), \
                    patch.object(sys, "argv", argv), patch("time.time", side_effect=lambda: clock.wall), \
                    patch("time.monotonic", side_effect=lambda: clock.monotonic), redirect_stdout(output):
                Recorder.main(argv[1:])
            rows = [json.loads(line) for line in steps_file.read_text().splitlines()]
            self.assertEqual(len(rows), 30)
            self.assertEqual([row["control_step"] for row in rows], list(range(1, 31)))
            for row, (utime, pose) in zip(rows, samples):
                self.assertEqual(set(row), {"control_step", "sim_time", "robot_q", "robot_v", "robot_u", "objects"})
                self.assertEqual(row["sim_time"], utime / 1e6)
                self.assertEqual(row["objects"], {channel: pose})
                self.assertEqual(row["robot_q"], list(range(5)))
                self.assertEqual(row["robot_v"], [.1] * 5)
                self.assertEqual(row["robot_u"], [.2] * 5)
            pattern = re.compile(r"^\[sugar_box\] step=(\d+) sim=([\d.]+)s wall=([\d.]+)s "
                                 r"pos_err=([\d.]+)m yaw_err=([\d.]+)deg within_goal=(yes|no)$")
            progress = [pattern.fullmatch(line) for line in output.getvalue().splitlines()
                        if line.startswith("[sugar_box]")]
            self.assertEqual(len(progress), 3, output.getvalue())
            self.assertTrue(all(progress), output.getvalue())
            self.assertEqual([match[1] for match in progress], ["0010", "0020", "0030"])
            self.assertEqual([float(match[2]) for match in progress], [.2, .4, .6])
            self.assertEqual([float(match[3]) for match in progress], [1.2, 2.4, 3.6])
            self.assertEqual([float(match[4]) for match in progress], [0, 0, .2])
            self.assertEqual([float(match[5]) for match in progress], [2.9, 11.5, 2.9])
            self.assertEqual([match[6] for match in progress], ["yes", "no", "no"])
            final, = [json.loads(line[len("FINAL "):]) for line in output.getvalue().splitlines()
                      if line.startswith("FINAL ")]
            self.assertEqual(final["control_steps"], 30)
            self.assertEqual(final["first_success_t"], .02)
            self.assertEqual(sum(line.startswith("SUCCESS ") for line in output.getvalue().splitlines()), 1)
