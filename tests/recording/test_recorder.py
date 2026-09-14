from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import re
import runpy
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from c3plus.configs.paths import REPO


class WorkflowTests(unittest.TestCase):
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
                    "--url", "memq://fake", "--duration", "10"]
            output = io.StringIO()
            with patch.dict(sys.modules, {"pydrake": pydrake, "pydrake.lcm": lcm}), \
                    patch.object(sys, "argv", argv), patch("time.time", side_effect=lambda: clock.wall), \
                    patch("time.monotonic", side_effect=lambda: clock.monotonic), redirect_stdout(output):
                runpy.run_module("c3plus.recording.recorder", run_name="__main__")
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
