"""Read-only regression tests for the reproducible run workflow."""
import argparse
import copy
from contextlib import redirect_stderr, redirect_stdout
import csv
import io
import json
import math
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

    def test_named_campaign_cli_plans_complete_yaw_grid_without_writes(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name, starts, count in (("run_launch", range(1, 6), 180),
                                        ("run_launch_simple_s2", (2,), 36)):
                with self.subTest(campaign=name):
                    out = Path(tmp) / name
                    result = subprocess.run(
                        [sys.executable, "-m", "tools.experiments", name,
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
                result = {"success": False, "t_success": None, "final_position_error": 0.2,
                          "final_orientation_error": 0.3}
                status = {"failures": [], "wrapper_rc": 0, "seed_verified": True,
                          "goal_yaw_verified": True, "simulation_wall_seconds": 600}
                (out / f"{plan['run_id']}_result.json").write_text(json.dumps(result))
                (out / "runtime_status.json").write_text(json.dumps(status))
                (out / f"{plan['run_id']}.mp4").touch()
                (out / "RUN_COMPLETE").touch()
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

    def test_yaw_override_validation_preserves_indexed_position(self):
        base = dict(scene="single_obstacle", obstacle_cost="relu", start=2, goal=2, out="/unused/yaw")
        original = R.plan_run(**base)
        self.assertNotIn("goal_yaw_degrees", original)
        self.assertEqual(original["run_id"], "relu_single_obstacle_s02g02_seed42")
        for value in (45, 180, float("nan"), float("inf"), True, "90"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                R.plan_run(**base, goal_yaw_degrees=value)
        for value in (90, 0, -90):
            changed = R.plan_run(**base, goal_yaw_degrees=value)
            self.assertEqual(changed["controller_goal"][:2], original["controller_goal"][:2])
            self.assertAlmostEqual(changed["controller_goal"][2], math.radians(value))
            with self.assertRaisesRegex(ValueError, "does not match"):
                R.plan_run(**base, goal_yaw_degrees=value, goal_pose=[0, 0, math.radians(value)])

    def test_yaw_launch_packaging_and_native_target_verification(self):
        cases = [(90, "correct"), (0, "correct"), (-90, "correct"),
                 (90, "missing"), (90, "wrong_position"), (90, "stale_binary")]
        for yaw, condition in cases:
            with self.subTest(yaw=yaw, condition=condition), tempfile.TemporaryDirectory() as tmp:
                repo = Path(tmp)
                demo = Path("examples/sampling_c3/matched_single_obstacle_xarm6_t2/parameters")
                shutil.copytree(R.REPO / demo, repo / demo)
                out = repo / "output"
                phases = []

                def logged(command, log, env):
                    phases.append((log.name, command))
                    log.touch()
                    if log.name != "launcher.log":
                        return 0
                    if condition == "stale_binary":
                        log.write_text("Controller does not support --goal_yaw_degrees; rebuild")
                        return 2
                    banner = "[SAMPLER-SEED] deterministic seed=42\n"
                    if condition != "missing":
                        x = "0.9" if condition == "wrong_position" else "0.397"
                        banner += f"[GOAL-YAW] goal_yaw_degrees={yaw} goal_x={x} goal_y=-0.431\n"
                    (out / "planner.log").write_text(banner)
                    for name in ("sim.log", "osc.log", "steps_raw.jsonl", "state_trace.jsonl"):
                        (out / name).write_text("test-data")
                    return 0

                with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                        patch.object(R, "logged_command", side_effect=logged), \
                        patch.object(R.subprocess, "check_output", side_effect=lambda cmd, **kw: "test" if kw.get("text") else b""):
                    if condition == "correct":
                        status = R.run_one("single_obstacle", "relu", 2, 2, out, goal_yaw_degrees=yaw)
                        self.assertTrue(status["goal_yaw_verified"])
                    else:
                        with self.assertRaises(RuntimeError):
                            R.run_one("single_obstacle", "relu", 2, 2, out, goal_yaw_degrees=yaw)
                saved = json.loads((out / "runtime_status.json").read_text())
                self.assertEqual(saved["goal_yaw_degrees"], yaw)
                self.assertEqual((out / "RUN_COMPLETE").exists(), condition == "correct")
                self.assertEqual(phases[0][1][-1], str(yaw))
                if condition == "correct":
                    target = [0.397, -0.431, math.radians(yaw)]
                    self.assertEqual(saved["controller_goal"], target)
                    evaluation = R.yaml.safe_load((out / "evaluation_scene_config.yaml").read_text())
                    self.assertEqual(evaluation["goal"], target)
                    self.assertEqual(phases[0][1][4:7], list(map(str, target)))
                    render = next(cmd for phase, cmd in phases if phase == "render.log")
                    index = render.index("--goal")
                    self.assertEqual(render[index + 1:index + 4], list(map(str, target)))
                else:
                    self.assertEqual(len(phases), 1)

    def test_yaw_launcher_rejects_stale_binary_without_starting_processes(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            launcher = repo / "tools/experiments/launch_run.sh"
            launcher.parent.mkdir(parents=True)
            shutil.copy2(S.TOOL_DIR / "launch_run.sh", launcher)
            controller = repo / "bazel-bin/examples/sampling_c3/franka_sampling_c3_controller"
            controller.parent.mkdir(parents=True)
            controller.write_text("#!/bin/sh\n[ \"$1\" = --helpshort ] || exit 99\nprintf 'old controller flags\\n'\nexit 1\n")
            controller.chmod(0o755)
            for yaw in ("90", "0", "-90", "45"):
                out = repo / ("output" + yaw)
                result = subprocess.run(["bash", str(launcher), "unused_demo", "object", "0.397", "-0.431",
                                         "0", "600", "19001", str(out), yaw], capture_output=True, text=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("must be" if yaw == "45" else "Rebuild", result.stderr)
                self.assertFalse(out.exists())

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
