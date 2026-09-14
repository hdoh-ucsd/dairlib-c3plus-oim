from c3plus.recording.execution import read_execution_steps, read_native_execution
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from c3plus.evaluation import exporter as P

from tests.fixtures.results import ProjectionFixtures

class ResultProjectionTests(ProjectionFixtures, unittest.TestCase):
    def test_native_boundary_reader_keeps_exact_states_and_distinct_empty_status(self):
        native, _ = self.physical_fixture()
        native["boundaries"][0]["diagnostic_extension"] = {"preserve": [None, "μ"]}
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "sim.log"
            self.assertIsNone(read_native_execution(tmp))
            log.write_text("ordinary simulator output\n")
            self.assertIsNone(read_native_execution(tmp))
            header = "[C3_EXECUTION_LOGGING] " + json.dumps(native["headers"][0]) + "\n"
            log.write_text(header)
            self.assertEqual(read_native_execution(tmp),
                             {"headers": native["headers"], "boundaries": [], "terminals": []})
            log.write_text(header + "unrelated output\n" + "".join(
                "[C3_EXECUTION_BOUNDARY] " + json.dumps(item) + "\n"
                for item in native["boundaries"]) + "".join(
                "[C3_EXECUTION_TERMINAL] " + json.dumps(item) + "\n"
                for item in native["terminals"]))
            self.assertEqual(read_native_execution(tmp), native)

    def test_execution_timing_adds_only_native_wall_clock_fields(self):
        cfg, summary, steps, rows = self.fixture()
        # Native execution has its own axis, unrelated to the five snapshots
        # with repeated/decreasing simulation timestamps in this fixture.
        events = [{"execution_step": i, "execution_wall_time": t,
                   "utime": 10_000_000 + i, "mode": "c3" if i == 1 else "reposition"}
                  for i, t in enumerate([0.0, .0213, .0428])]
        with tempfile.TemporaryDirectory() as tmp:
            before = self.project(tmp, cfg, summary, steps, rows)
            (Path(tmp) / "planner.log").write_text("unrelated log\n" + "".join(
                "[C3_EXECUTION_STEP] " + json.dumps(event) + "\n" for event in events))
            after = self.project(tmp, cfg, summary, steps, rows)
        self.assertEqual(after["dynamic"].pop("execution_step"), [0, 1, 2])
        self.assertEqual(after["dynamic"].pop("execution_wall_time"), [0.0, .0213, .0428])
        timing = after.pop("execution_timing")
        self.assertEqual(timing["n_steps"], 3)
        self.assertEqual(timing["n_intervals"], 2)
        self.assertAlmostEqual(timing["elapsed_wall_time_s"], .0428)
        self.assertAlmostEqual(timing["mean_step_wall_time_s"], .0214)
        self.assertAlmostEqual(timing["frequency_hz"], 1 / np.mean(np.diff([0.0, .0213, .0428])))
        self.assertEqual(timing["source"], "monotonic_wall_time_at_execution_step")
        self.assertEqual(after, before)


    def test_execution_frequency_requires_two_valid_native_events(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "planner.log"
            log.write_text("legacy planner output\n")
            self.assertIsNone(read_execution_steps(tmp))
            log.write_text("[C3_EXECUTION_TIMING] monotonic_wall_time_at_execution_step\n")
            self.assertEqual(read_execution_steps(tmp), [])
        for times in ([], [0.0]):
            with self.subTest(times=times):
                result = {"dynamic": {"time": [100, 200]}}
                events = [{"execution_step": i, "execution_wall_time": t} for i, t in enumerate(times)]
                P.add_execution_timing(result, events)
                self.assertIsNone(result["execution_timing"]["frequency_hz"])
                self.assertIsNone(result["execution_timing"]["mean_step_wall_time_s"])
                self.assertEqual(result["dynamic"]["time"], [100, 200])
        for times in ([1.0, 2.0], [0.0, 0.0], [0.0, -1.0], [0.0, float("nan")],
                      [0.0, float("inf")], [False, 1.0], [0.0, "0.2"]):
            with self.subTest(times=times), self.assertRaises(ValueError):
                P.add_execution_timing({"dynamic": {}}, [
                    {"execution_step": i, "execution_wall_time": t} for i, t in enumerate(times)])
        for indices in ([1, 2], [0, 2], [0, 0], [False, 1]):
            with self.subTest(indices=indices), self.assertRaises(ValueError):
                P.add_execution_timing({"dynamic": {}}, [
                    {"execution_step": i, "execution_wall_time": t} for i, t in zip(indices, [0.0, .02])])
