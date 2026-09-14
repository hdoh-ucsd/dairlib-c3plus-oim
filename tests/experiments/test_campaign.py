from contextlib import redirect_stderr, redirect_stdout
import csv
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from c3plus import configs as S
from c3plus.experiments import run as R, campaign as G

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_six_scene_grid(self):
        jobs = G.jobs(self.args())
        self.assertEqual(len(jobs), 300)
        self.assertEqual([j["obstacle_cost"] for j in jobs[:2]], ["exponential", "relu"])


    def test_smoke_and_diagonal(self):
        self.assertEqual(len(G.jobs(self.args(pairs="smoke"))), 12)
        self.assertEqual(len(G.jobs(self.args(pairs="diagonal"))), 60)


    def test_exact_campaign_manifest(self):
        jobs = G.jobs(self.args(manifest=S.REPO / "c3plus/experiments/manifests/campaign_seed42.json"))
        self.assertEqual(len(jobs), 25)
        self.assertEqual(sum(j["obstacle_cost"] == "exponential" for j in jobs), 13)
        self.assertEqual(jobs[-1]["start"], 3)
        self.assertEqual(jobs[-1]["goal"], 3)
        self.assertEqual(jobs[0]["goal_pose"], [0.381, -0.4, 3.1416])
        for job in jobs:
            plan = R.plan_run(job["scene"], job["obstacle_cost"], job["start"], job["goal"],
                              "/unused/test", goal_pose=job["goal_pose"])
            self.assertEqual(plan["evaluation_goal"], tuple(job["goal_pose"]))


    def test_manifest_rejects_explicit_grid_selection(self):
        manifest = S.REPO / "c3plus/experiments/manifests/campaign_seed42.json"
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "campaign"
            for flags in (["--scenes", "open_task"], ["--obstacle_cost", "both"],
                          ["--pairs", "smoke"]):
                with self.subTest(flags=flags):
                    argv = ["campaign", "--manifest", str(manifest),
                            "--output-root", str(out), *flags]
                    errors = io.StringIO()
                    with patch("sys.argv", argv), patch.object(G, "run_one") as run, \
                            redirect_stderr(errors), self.assertRaises(SystemExit) as error:
                        G.main()
                    self.assertEqual(error.exception.code, 2)
                    self.assertIn("cannot be combined", errors.getvalue())
                    self.assertFalse(out.exists())
                    run.assert_not_called()


    def test_campaign_dry_run_never_creates_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "campaign"
            argv = ["campaign", "--output-root", str(out), "--scenes", *R.SCENES,
                    "--obstacle_cost", "both", "--pairs", "all", "--dry-run"]
            stream = io.StringIO()
            with patch("sys.argv", argv), patch.object(G, "run_one") as run, redirect_stdout(stream):
                G.main()
            plan = json.loads(stream.getvalue())
            self.assertEqual(plan["run_count"], 300)
            self.assertEqual({job["obstacle_cost"] for job in plan["runs"]}, {"exponential", "relu"})
            for job in plan["runs"]:
                self.assertEqual(Path(job["out"]).relative_to(out).parts[0], job["obstacle_cost"])
            self.assertFalse(out.exists())
            run.assert_not_called()


    def test_named_campaign_cli_plans_complete_yaw_grid_without_writes(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name, starts, count in (("run_launch", range(1, 6), 180),
                                        ("run_launch_simple_s2", (2,), 36)):
                with self.subTest(campaign=name):
                    out = Path(tmp) / name
                    result = subprocess.run(
                        [sys.executable, "-m", "tools", name,
                         "--output-root", str(out), "--dry-run"],
                        cwd=R.REPO, capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    plan = json.loads(result.stdout)
                    runs = plan["runs"]
                    self.assertEqual(plan["run_count"], count)
                    self.assertEqual(len(runs), count)
                    self.assertEqual(len({run["run_id"] for run in runs}), count)
                    self.assertEqual(len({run["out"] for run in runs}), count)
                    actual = {(r["scene"], r["start"], r["goal_index"],
                               r["goal_yaw_degrees"], r["obstacle_cost"]) for r in runs}
                    expected = {(scene, start, 2, yaw, cost) for scene in S.SCENES
                                for start in starts for yaw in (90, 0, -90)
                                for cost in ("exponential", "relu")}
                    self.assertEqual(actual, expected)
                    for run in runs:
                        original = S.load_controller_goal(run["demo"])[1]
                        self.assertEqual(run["source_controller_goal"], list(original))
                        self.assertEqual(run["controller_goal"][:2], list(original[:2]))
                        self.assertAlmostEqual(run["controller_goal"][2], math.radians(run["goal_yaw_degrees"]))
                        self.assertEqual(run["evaluation_goal"], run["controller_goal"])
                        self.assertEqual(run["seed"], 42)
                        self.assertEqual(run["wall_cap_seconds"], 600)
                        self.assertTrue(Path(run["out"]).is_relative_to(out))
                    for first, second in zip(runs[::2], runs[1::2]):
                        self.assertEqual(first["controller_goal"], second["controller_goal"])
                        self.assertEqual(first["start"], second["start"])
                        self.assertEqual((first["obstacle_cost"], second["obstacle_cost"]), ("exponential", "relu"))
                    self.assertFalse(out.exists())


    def test_named_campaign_defaults_and_fixed_selection(self):
        for name in ("run_launch", "run_launch_simple_s2"):
            stream = io.StringIO()
            with redirect_stdout(stream), patch.object(G, "run_one") as run:
                G.main([name, "--dry-run"])
            run.assert_not_called()
            for plan in json.loads(stream.getvalue())["runs"]:
                self.assertTrue(Path(plan["out"]).is_relative_to(R.REPO / "results/reproduce" / name))
            for flags in (("--scenes", "open_task"), ("--pairs", "all"),
                          ("--obstacle_cost", "relu"), ("--manifest", "unused"), ("--seed", "7")):
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    G.main([name, "--dry-run", *flags])
                self.assertEqual(error.exception.code, 2)


    def test_named_campaign_stop_resume_and_summary_preserve_all_yaws(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            argv = ["run_launch_simple_s2", "--output-root", str(root)]
            completed = []

            def complete(scene, cost, start, goal, out, cap, port, goal_pose, **yaw):
                plan = R.plan_run(scene, cost, start, goal, out, cap, port, goal_pose, **yaw)
                out.mkdir(parents=True, exist_ok=False)
                result = {"run_id": plan["run_id"], "scenario": scene,
                          "success": False, "t_success": None, "final_position_error": 0.2,
                          "final_orientation_error": 0.3}
                status = {"failures": [], "wrapper_rc": 0, "seed_verified": True,
                          "goal_yaw_verified": True, "simulation_wall_seconds": 600}
                video = out / f"{plan['run_id']}.mp4"
                video.write_bytes(b"validated video fixture")
                result.update(runtime_status=status, package={"status": "complete", "cleanup_complete": True,
                    "video": {"file": video.name, "size_bytes": video.stat().st_size,
                              "sha256": R.hashlib.sha256(video.read_bytes()).hexdigest()}})
                (out / f"{plan['run_id']}_result.json").write_text(json.dumps(result))
                completed.append(plan["run_id"])
                if len(completed) == 1:
                    (root / "STOP_AFTER_CURRENT").touch()
                return status

            def rows():
                with (root / "summary.csv").open() as stream:
                    return list(csv.DictReader(stream))

            with patch.object(G, "run_one", side_effect=complete), redirect_stdout(io.StringIO()):
                G.main(argv)
                self.assertEqual(len(completed), 1)
                self.assertEqual(sum(row["status"] == "complete" for row in rows()), 1)
                self.assertEqual(len(rows()), 36)
                (root / "STOP_AFTER_CURRENT").unlink()
                G.main([*argv, "--resume"])
            self.assertEqual(len(completed), 36)
            self.assertEqual(len(set(completed)), 36)
            self.assertTrue(all(row["status"] == "complete" for row in rows()))
            self.assertTrue(all(row["success"] == "False" for row in rows()))
            self.assertTrue(all((root / row["video"]).is_file() for row in rows()))
            original_plan = (root / "campaign_plan.json").read_text()
            (root / "summary.csv").unlink()
            with patch.object(G, "run_one") as run, redirect_stdout(io.StringIO()):
                G.main([*argv, "--resume"])
                run.assert_not_called()
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    G.main([*argv, "--resume", "--cap", "15"])
                run.assert_not_called()
            self.assertEqual(len(rows()), 36)
            self.assertEqual((root / "campaign_plan.json").read_text(), original_plan)


    def test_named_campaign_resume_refuses_partial_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            partial = root / "exponential/open_task/s02g02_yaw_p090"
            partial.mkdir(parents=True)
            (partial / "planner.log").write_text("partial run evidence")
            with patch.object(R, "logged_command") as launch, redirect_stdout(io.StringIO()), \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                G.main(["run_launch_simple_s2", "--output-root", str(root), "--resume"])
            launch.assert_not_called()
            self.assertEqual((partial / "planner.log").read_text(), "partial run evidence")
            with (root / "summary.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["status"], "partial")
            self.assertTrue(all(row["status"] == "pending" for row in rows[1:]))


    def test_resume_refuses_changed_plan(self):
        with tempfile.TemporaryDirectory() as tmp:
            argv = ["campaign", "--output-root", tmp, "--scenes", "open_task"]
            with patch("sys.argv", argv), patch.object(G, "run_one", return_value={"failures": []}):
                G.main()
            original = (Path(tmp) / "campaign_plan.json").read_text()
            with patch("sys.argv", [*argv, "--resume", "--cap", "1"]), \
                    patch.object(G, "run_one") as run:
                with self.assertRaises(SystemExit):
                    G.main()
            run.assert_not_called()
            self.assertEqual((Path(tmp) / "campaign_plan.json").read_text(), original)


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
            done = Path(tmp) / "exponential/open_task/s01g01"
            done.mkdir(parents=True)
            (done / "RUN_COMPLETE").touch()
            run_id = "exponential_open_task_s01g01_seed42"
            (done / f"{run_id}_result.json").write_text(json.dumps({"run_id": run_id}))
            (done / f"{run_id}.mp4").write_bytes(b"legacy video fixture")
            argv = ["run_grid_campaign.py", "--output-root", tmp,
                    "--scenes", "open_task", "--resume"]
            with patch("sys.argv", argv), patch.object(G, "run_one", return_value={"failures": []}) as run:
                G.main()
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[1], "relu")
