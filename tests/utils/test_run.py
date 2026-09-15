from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from c3plus.utils import run as R

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_seed_check_exception_requires_native_goal_and_only_reposition(self):
        goal = {'terminals': [{'reason': 'goal_reached'}]}
        self.assertTrue(R.goal_reached_without_sampling(goal, [{'mode': 'reposition'}]))
        for native, updates in ((None, [{'mode': 'reposition'}]),
                                ({'terminals': [{'reason': 'shutdown'}]}, [{'mode': 'reposition'}]),
                                (goal, []), (goal, [{'mode': 'c3'}]),
                                (goal, [{'mode': 'reposition'}, {'mode': 'c3'}])):
            self.assertFalse(R.goal_reached_without_sampling(native, updates))

    def test_run_dry_run_never_launches(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run"
            for flags, expected in (([], "exponential"), (["--obstacle_cost", "exponential"], "exponential"),
                                    (["--obstacle_cost", "relu"], "relu")):
                argv = ["run", "--scene", "open_task", "--out", str(out), "--dry-run", *flags]
                stream = io.StringIO()
                with patch("sys.argv", argv), patch.object(R, "run_one") as run, redirect_stdout(stream):
                    R.main()
                plan = json.loads(stream.getvalue())
                self.assertEqual(plan["obstacle_cost"], expected)
                self.assertEqual(plan["simulation_cap_seconds"], 300)
                self.assertEqual(plan["run_id"], f"{expected}_open_table_T_shape_s01g01_seed42")
                self.assertFalse(out.exists())
                run.assert_not_called()


    def test_explicit_simulation_cap_overrides_run_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run"
            stream = io.StringIO()
            with patch.object(R, "run_one") as run, redirect_stdout(stream):
                R.main(["--task", "open_table", "--out", str(out), "--cap", "600", "--dry-run"])
            self.assertEqual(json.loads(stream.getvalue())["simulation_cap_seconds"], 600)
            self.assertFalse(out.exists())
            run.assert_not_called()


    def test_direct_run_default_and_explicit_cap_are_forwarded_to_plan(self):
        for options, expected in (({}, 300), ({"cap": 600}, 600)):
            with self.subTest(options=options), \
                    patch.object(R, "plan_run", side_effect=RuntimeError("planning-only stop")) as plan:
                with self.assertRaisesRegex(RuntimeError, "planning-only stop"):
                    R.run_one("open_table", "exponential", 1, 1, "/unused/test", **options)
                self.assertEqual(plan.call_args.args[5], expected)


    def test_invalid_parameters(self):
        for changes in (dict(start=0), dict(port=1), dict(cap=0),
                        dict(goal_pose=[float("nan"), 0, 0])):
            values = dict(scene="open_task", obstacle_cost="exponential", start=1,
                          goal=1, out=Path("/unused/test"))
            values.update(changes)
            with self.assertRaises(ValueError):
                R.run_one(**values)


    def test_never_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(R.os, "access", return_value=True):
            out = Path(tmp)
            with self.assertRaises(FileExistsError):
                R.run_one("open_task", "exponential", 1, 1, out)


class RecordingFlagTests(WorkflowFixtures, unittest.TestCase):
    """--no-video and --no-record must reach run_one without touching planning."""

    def _forwarded(self, flags):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run"
            argv = ["run", "--scene", "open_task", "--out", str(out), *flags]
            with patch("sys.argv", argv), patch.object(R, "run_one") as run:
                R.main()
            return run.call_args.kwargs

    def test_recording_and_video_default_to_enabled(self):
        kwargs = self._forwarded([])
        self.assertTrue(kwargs["record"])
        self.assertTrue(kwargs["video"])

    def test_no_video_keeps_recording(self):
        kwargs = self._forwarded(["--no-video"])
        self.assertTrue(kwargs["record"])
        self.assertFalse(kwargs["video"])

    def test_no_record_forwards_and_implies_no_video(self):
        kwargs = self._forwarded(["--no-record"])
        self.assertFalse(kwargs["record"])
        # run_one enforces the implication; the flag itself stays independent.
        with patch.object(R, "plan_run", side_effect=RuntimeError("planning-only stop")):
            with self.assertRaisesRegex(RuntimeError, "planning-only stop"):
                R.run_one("open_table", "exponential", 1, 1, "/unused/test", record=False)

    def test_planning_never_receives_recording_options(self):
        # plan_run resolves configuration only; recording choices must not leak.
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run"
            argv = ["run", "--scene", "open_task", "--out", str(out),
                    "--no-record", "--dry-run"]
            stream = io.StringIO()
            with patch("sys.argv", argv), patch.object(R, "plan_run",
                                                       wraps=R.plan_run) as plan, \
                    redirect_stdout(stream):
                R.main()
            self.assertNotIn("record", plan.call_args.kwargs)
            self.assertNotIn("video", plan.call_args.kwargs)

    def test_oim_out_is_forwarded_and_defaults_to_none(self):
        self.assertIsNone(self._forwarded([])["oim_out"])
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "oim"
            kwargs = self._forwarded(["--oim-out", str(target)])
            self.assertEqual(Path(kwargs["oim_out"]), target)
