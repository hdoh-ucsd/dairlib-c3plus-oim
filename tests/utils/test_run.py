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
                self.assertEqual(plan["run_id"], f"{expected}_open_table_T_shape_s01g01_seed42")
                self.assertFalse(out.exists())
                run.assert_not_called()


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
