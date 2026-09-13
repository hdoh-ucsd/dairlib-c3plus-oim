"""Read-only regression tests for the reproducible run workflow."""
import argparse
import copy
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

if __package__:
    from . import catalog as S
    from . import __main__ as cli
    from . import run_experiment as R
    from . import run_grid_campaign as G
else:
    import catalog as S
    # Avoid importing the test runner's own __main__ during direct discovery.
    import importlib.util
    spec = importlib.util.spec_from_file_location("experiments_cli", Path(__file__).with_name("__main__.py"))
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    import run_experiment as R
    import run_grid_campaign as G


class WorkflowTests(unittest.TestCase):
    def args(self, **changes):
        values = dict(manifest=None, obstacle_cost="both", scenes=list(R.SCENES), pairs="all")
        values.update(changes)
        return argparse.Namespace(**values)

    def test_checkout_paths(self):
        self.assertEqual(S.REPO, Path(__file__).resolve().parents[2])

    def test_six_scene_grid(self):
        jobs = G.jobs(self.args())
        self.assertEqual(len(jobs), 300)
        self.assertEqual([j["obstacle_cost"] for j in jobs[:2]], ["exponential", "relu"])

    def test_smoke_and_diagonal(self):
        self.assertEqual(len(G.jobs(self.args(pairs="smoke"))), 12)
        self.assertEqual(len(G.jobs(self.args(pairs="diagonal"))), 60)

    def test_exact_campaign_manifest(self):
        jobs = G.jobs(self.args(manifest=Path(__file__).with_name("campaign_seed42.json")))
        self.assertEqual(len(jobs), 25)
        self.assertEqual(sum(j["obstacle_cost"] == "exponential" for j in jobs), 13)
        self.assertEqual(jobs[-1]["start"], 3)
        self.assertEqual(jobs[-1]["goal"], 3)
        self.assertEqual(jobs[0]["goal_pose"], [0.381, -0.4, 3.1416])
        for job in jobs:
            plan = R.plan_run(job["scene"], job["obstacle_cost"], job["start"], job["goal"],
                              "/unused/test", goal_pose=job["goal_pose"])
            self.assertEqual(plan["evaluation_goal"], tuple(job["goal_pose"]))

    def test_controller_goal_is_selected_demo_goal(self):
        plan = R.plan_run("open_task", "exponential", 2, 3, "/unused/test")
        self.assertEqual(plan["demo"], "matched_open_table_xarm6_s2g3")
        self.assertEqual(plan["controller_goal"], S.load_controller_goal(S.demo_name("open_task", 3, 3))[1])
        self.assertEqual(plan["evaluation_goal"], plan["controller_goal"])
        with self.assertRaisesRegex(ValueError, "does not match"):
            R.plan_run("open_task", "exponential", 1, 1, "/unused/test", goal_pose=[0, 0, 0])

    def test_manifest_rejects_explicit_grid_selection(self):
        manifest = Path(__file__).with_name("campaign_seed42.json")
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

    def test_cli_help_without_scientific_dependencies(self):
        result = subprocess.run([sys.executable, "-S", "-m", "tools.experiments", "--help"],
                                cwd=R.REPO, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        for command in ("build", "scenes", "check", "run", "campaign", "postprocess", "render"):
            self.assertIn(command, result.stdout)

    def test_cli_forwards_leaf_exit_code(self):
        with patch.object(cli.subprocess, "call", return_value=17) as call:
            self.assertEqual(cli.main(["run", "--scene", "open_task", "--out", "a path"]), 17)
        self.assertEqual(call.call_args.args[0][-2:], ["--out", "a path"])

    def test_logged_command_records_literal_arguments_before_execution(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "checkout with spaces"
            repo.mkdir()
            script = repo / "capture argv.py"
            script.write_text(
                "import json, os, sys\n"
                "from pathlib import Path\n"
                "print(json.dumps({'argv': sys.argv[1:], 'cwd': os.getcwd(), "
                "'header': Path(sys.argv[1]).read_text()}))\n"
                "sys.exit(17)\n"
            )
            log = repo / "phase output.log"
            arguments = [str(log), "argument with spaces", "$(touch unexpected)",
                         "; touch unexpected;", "'quotes' \"$HOME\" *"]
            command = [sys.executable, str(script), *arguments]
            with patch.object(R, "REPO", repo):
                rc = R.logged_command(command, log, R.os.environ.copy())
            self.assertEqual(rc, 17)
            command_line, cwd_line, output = log.read_text().splitlines()
            self.assertTrue(command_line.startswith("[COMMAND] "))
            self.assertEqual(shlex.split(command_line[len("[COMMAND] "):]), command)
            self.assertTrue(cwd_line.startswith("[CWD] "))
            self.assertEqual(shlex.split(cwd_line[len("[CWD] "):]), [str(repo)])
            observed = json.loads(output)
            self.assertEqual(observed["argv"], arguments)
            self.assertEqual(observed["cwd"], str(repo))
            self.assertEqual(observed["header"], f"{command_line}\n{cwd_line}\n")
            self.assertFalse((repo / "unexpected").exists())

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
                self.assertEqual(plan["run_id"], f"{expected}_open_task_s01g01_seed42")
                self.assertFalse(out.exists())
                run.assert_not_called()

    def test_packaging_uses_recorded_goal_and_run_temporary_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            demo = Path("examples/sampling_c3/matched_single_obstacle_xarm6_t1/parameters")
            shutil.copytree(R.REPO / demo, repo / demo)
            out = repo / "output"
            calls = []

            def logged(command, log, env):
                calls.append((log.name, command, dict(env)))
                log.touch()
                if log.name == "launcher.log":
                    (out / "planner.log").write_text("[SAMPLER-SEED] deterministic seed=42")
                    for name in ("sim.log", "osc.log", "steps_raw.jsonl", "state_trace.jsonl"):
                        (out / name).write_text("test-data")
                return 0

            def git(command, **kwargs):
                return "test-commit" if kwargs.get("text") else b""

            with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                    patch.object(R, "logged_command", side_effect=logged), \
                    patch.dict(R.os.environ, {"C3PLUS_CONTAINER_IMAGE": "test:tag",
                                              "C3PLUS_CONTAINER_IMAGE_ID": "sha256:test",
                                              "SAMPLING_C3_OBS_BOXES": "stale"}), \
                    patch.object(R.subprocess, "check_output", side_effect=git):
                status = R.run_one("single_obstacle", "exponential", 1, 1, out)
            self.assertTrue((out / "RUN_COMPLETE").is_file())
            self.assertEqual(status["runtime"]["container_image"], "test:tag")
            self.assertEqual(status["runtime"]["container_image_id"], "sha256:test")
            self.assertEqual(status["goal"], status["controller_goal"])
            self.assertEqual(status["obstacle_cost"], "exponential")
            self.assertEqual(status["sampler_settings"]["SAMPLING_C3_OBS_BOXES"], "0.35,0,0.05,0.05")
            self.assertEqual(calls[0][2]["SAMPLING_C3_OBS_BOXES"], "0.35,0,0.05,0.05")
            self.assertEqual(len(calls[0][1]), 10)  # bash, launcher, eight arguments; no env script
            self.assertEqual(calls[0][1][4:7], list(map(str, status["goal"])))
            for _, _, env in calls[1:]:
                self.assertEqual(env["TMPDIR"], str(out / "tmp"))
            render = next(command for phase, command, _ in calls if phase == "render.log")
            index = render.index("--goal")
            self.assertEqual(render[index + 1:index + 4], list(map(str, status["goal"])))
            cost_figure = next(command for phase, command, _ in calls if phase == "cost_fig.log")
            self.assertEqual(cost_figure[-2:], ["--obstacle_cost", "exponential"])

    def renderer(self):
        try:
            if __package__:
                from . import render_run_3d
            else:
                import render_run_3d
            return render_run_3d
        except ModuleNotFoundError as exc:
            self.skipTest(f"Renderer dependencies unavailable: {exc}")

    def test_renderer_removes_own_temporary_files_on_failure(self):
        renderer = self.renderer()
        created = []

        def fail(args):
            created.extend([Path(args.assets_tmp), Path(args.frames_dir)])
            for path in created:
                (path / "generated-file").write_text("test")
            raise RuntimeError("encoding failed")

        with patch.object(renderer, "_render_trace", side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, "encoding failed"):
                renderer.render_trace(argparse.Namespace(assets_tmp=None, frames_dir=None))
        self.assertEqual(len(created), 2)
        self.assertTrue(all(not path.exists() for path in created))

    def test_renderer_preserves_requested_temporary_files(self):
        renderer = self.renderer()
        with tempfile.TemporaryDirectory() as tmp:
            args = argparse.Namespace(assets_tmp=tmp, frames_dir=tmp)
            with patch.object(renderer, "_render_trace"):
                renderer.render_trace(args)
            self.assertTrue(Path(tmp).is_dir())

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

    def test_settings_isolated(self):
        with patch.dict(R.os.environ, {"SAMPLING_C3_UNKNOWN": "old",
                                      "SAMPLING_C3_RANK_OBS_MODE": "old"}):
            exponential = R.environment("exponential")
            relu = R.environment("relu")
        self.assertNotIn("SAMPLING_C3_UNKNOWN", exponential)
        self.assertNotIn("SAMPLING_C3_RANK_OBS_MODE", exponential)
        self.assertEqual(exponential["SAMPLING_C3_SEED"], "42")
        self.assertEqual(relu["SAMPLING_C3_OBS_RELU_W"], "200")

    def test_all_configs_and_goals(self):
        for scene in R.SCENES:
            config = R.yaml.safe_load((S.CONFIG_DIR / f"{scene}.yaml").read_text())
            S.planner_environment(config)
            for m in range(1, 6):
                for n in range(1, 6):
                    demo_name = S.demo_name(scene, m, n)
                    demo = R.REPO / "examples/sampling_c3" / demo_name
                    self.assertTrue((demo / "parameters/goal_params.yaml").is_file())
                    self.assertEqual(len(S.load_controller_goal(demo_name)[1]), 3)

    def test_planner_environment_serialization(self):
        config = {
            "obstacles": {"polygons": [
                [[0, 0], [2, 0], [2, 2], [0, 2]],
                [[3, 0], [4, 0], [3, 1]],
            ]},
            "planner": {"box_polygons": [0], "polygon_overrides": {2: 1}},
        }
        self.assertEqual(S.planner_environment(config), {
            "SAMPLING_C3_OBS_BOXES": "1,1,1,1",
            "SAMPLING_C3_OBS_POLYS": "2|3,0|4,0|3,1",
        })
        self.assertEqual(S.planner_environment({}), {})

    def test_planner_environment_rejects_invalid_geometry(self):
        valid = {"obstacles": {"polygons": [[[0, 0], [1, 0], [1, 1], [0, 1]]]},
                 "planner": {"box_polygons": [0]}}
        cases = []
        bad_reference = copy.deepcopy(valid)
        bad_reference["planner"]["box_polygons"] = [1]
        cases.append(bad_reference)
        triangle = copy.deepcopy(valid)
        triangle["obstacles"]["polygons"][0].pop()
        cases.append(triangle)
        nonfinite = copy.deepcopy(valid)
        nonfinite["obstacles"]["polygons"][0][0][0] = float("nan")
        cases.append(nonfinite)
        overlapping_slots = copy.deepcopy(valid)
        overlapping_slots["planner"]["polygon_overrides"] = {0: 0}
        cases.append(overlapping_slots)
        for config in cases:
            with self.subTest(config=config), self.assertRaises(ValueError):
                S.planner_environment(config)

    def test_planner_environment_preserves_scene_indices(self):
        def env(scene):
            return S.planner_environment(R.yaml.safe_load((S.CONFIG_DIR / f"{scene}.yaml").read_text()))
        shelf = env("shelf_gap")["SAMPLING_C3_OBS_BOXES"].split(";")
        self.assertEqual(shelf, ["0.62,0,0.13,0.08"] * 3 + ["0.24,0,0.05,0.08"] * 2)
        ycb = env("ycb_clutter")
        self.assertEqual([p.split("|")[0] for p in ycb["SAMPLING_C3_OBS_POLYS"].split(";")], ["2", "3"])
        icra = env("icra_sign")
        self.assertEqual(icra["SAMPLING_C3_OBJECT_FOOTPRINT"], "c_glyph")
        self.assertEqual(icra["SAMPLING_C3_OBS_TOP_Z"], "0.046")
        self.assertEqual([p.split("|")[0] for p in icra["SAMPLING_C3_OBS_POLYS"].split(";")], list(map(str, range(7))))

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
            argv = ["run_grid_campaign.py", "--output-root", tmp,
                    "--scenes", "open_task", "--resume"]
            with patch("sys.argv", argv), patch.object(G, "run_one", return_value={"failures": []}) as run:
                G.main()
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[1], "relu")


if __name__ == "__main__":
    unittest.main()
