"""Read-only regression tests for the reproducible run workflow."""
import argparse
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import run_experiment as R
import run_grid_campaign as G


class WorkflowTests(unittest.TestCase):
    def args(self, **changes):
        values = dict(manifest=None, variant="both", scenes=list(R.SCENES), pairs="all")
        values.update(changes)
        return argparse.Namespace(**values)

    def test_checkout_paths(self):
        self.assertEqual(Path(R.C.WT), R.REPO)

    def test_six_scene_grid(self):
        jobs = G.jobs(self.args())
        self.assertEqual(len(jobs), 300)
        self.assertEqual([j["variant"] for j in jobs[:2]], ["baseline", "relu"])

    def test_smoke_and_diagonal(self):
        self.assertEqual(len(G.jobs(self.args(pairs="smoke"))), 12)
        self.assertEqual(len(G.jobs(self.args(pairs="diagonal"))), 60)

    def test_exact_campaign_manifest(self):
        jobs = G.jobs(self.args(manifest=Path(__file__).with_name("campaign_seed42.json")))
        self.assertEqual(len(jobs), 25)
        self.assertEqual(sum(j["variant"] == "baseline" for j in jobs), 13)
        self.assertEqual(jobs[-1]["start"], 3)
        self.assertEqual(jobs[-1]["goal"], 3)
        self.assertEqual(jobs[0]["goal_pose"], [0.381, -0.4, 3.1416])

    def test_settings_isolated(self):
        with patch.dict(R.os.environ, {"SAMPLING_C3_UNKNOWN": "old",
                                      "SAMPLING_C3_RANK_OBS_MODE": "old"}):
            baseline = R.environment("baseline")
            relu = R.environment("relu")
        self.assertNotIn("SAMPLING_C3_UNKNOWN", baseline)
        self.assertNotIn("SAMPLING_C3_RANK_OBS_MODE", baseline)
        self.assertEqual(baseline["SAMPLING_C3_SEED"], "42")
        self.assertEqual(relu["SAMPLING_C3_OBS_RELU_W"], "200")

    def test_all_configs_and_goals(self):
        for scene in R.SCENES:
            for m in range(1, 6):
                for n in range(1, 6):
                    demo = R.REPO / "examples/sampling_c3" / R.C.demo_name(scene, m, n)
                    self.assertTrue((demo / "parameters/goal_params.yaml").is_file())
                    self.assertEqual(len(R.C.goal_of(scene, n)), 3)

    def test_failure_classification(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            for name in ("planner.log", "sim.log", "osc.log", "launcher.log"):
                (out / name).touch()
            (out / "planner.log").write_text("abort: Failure at CheckForWorkspaceLimitViolations")
            self.assertEqual(R.classify_failure(out)[0]["reason"], "workspace_limit_assertion")
            (out / "planner.log").write_text("AllFinite(q_v)")
            self.assertEqual(R.classify_failure(out)[0]["reason"], "AllFinite(q_v)")
            (out / "planner.log").write_text("SAMPLING_C3_TOPPLE_GUARD tripped")
            self.assertEqual(R.classify_failure(out)[0]["reason"], "topple_guard")

    def test_invalid_parameters(self):
        for changes in (dict(start=0), dict(port=1), dict(cap=0),
                        dict(goal_pose=[float("nan"), 0, 0])):
            values = dict(scene="open_task", variant="baseline", start=1,
                          goal=1, out=Path("/unused/test"))
            values.update(changes)
            with self.assertRaises(ValueError):
                R.run_one(**values)

    def test_never_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(R.os, "access", return_value=True):
            out = Path(tmp)
            with self.assertRaises(FileExistsError):
                R.run_one("open_task", "baseline", 1, 1, out)

    def test_stop_after_current(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            def complete(*args):
                (root / "STOP_AFTER_CURRENT").touch()
                return {"failures": []}
            argv = ["run_grid_campaign.py", "--output-root", tmp, "--scenes", "open_task"]
            with patch("sys.argv", argv), patch.object(G, "run_one", side_effect=complete) as run:
                G.main()
            self.assertEqual(run.call_count, 1)
            self.assertIn("STOPPED_AFTER_CURRENT", (root / "campaign_driver.log").read_text())

    def test_resume_skips_only_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            done = Path(tmp) / "baseline/open_task/s01g01"
            done.mkdir(parents=True)
            (done / "RUN_COMPLETE").touch()
            argv = ["run_grid_campaign.py", "--output-root", tmp,
                    "--scenes", "open_task", "--resume"]
            with patch("sys.argv", argv), patch.object(G, "run_one", return_value={"failures": []}) as run:
                G.main()
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[1], "relu")


if __name__ == "__main__":
    unittest.main()
