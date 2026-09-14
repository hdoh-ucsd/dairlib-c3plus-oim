"""Read-only regression tests for the reproducible run workflow."""
import argparse
import base64
import copy
from contextlib import redirect_stderr, redirect_stdout
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import runpy
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

if __package__:
    from . import catalog as S
    from . import __main__ as cli
    from . import run_experiment as R
    from . import run_grid_campaign as G
    from . import visualize_mesh as V
else:
    import catalog as S
    # Avoid importing the test runner's own __main__ during direct discovery.
    import importlib.util
    spec = importlib.util.spec_from_file_location("experiments_cli", Path(__file__).with_name("__main__.py"))
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    import run_experiment as R
    import run_grid_campaign as G
    import visualize_mesh as V


class WorkflowTests(unittest.TestCase):
    def source_state_fixture(self):
        empty = {"encoding": "utf-8", "content": "", "size_bytes": 0,
                 "sha256": hashlib.sha256(b"").hexdigest()}
        return {"format": "git-source-state/v1", "base_commit": "test-commit",
                "worktree_dirty": False, "scope": "test fixture",
                "tracked_patch": empty, "git_status": dict(empty), "untracked_files": {}}

    def copy_demo_configs(self, demo, destination):
        for source in S.load_demo_configs(demo):
            target = destination / source.relative_to(S.REPO)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)

    def args(self, **changes):
        values = dict(manifest=None, obstacle_cost="both", scenes=list(R.SCENES), pairs="all")
        values.update(changes)
        return argparse.Namespace(**values)

    def test_checkout_paths(self):
        self.assertEqual(S.REPO, Path(__file__).resolve().parents[2])

    def test_source_capture_reproduces_dirty_tree_without_changing_index(self):
        def decode(entry):
            raw = (entry["content"].encode("utf-8") if entry["encoding"] == "utf-8"
                   else base64.b64decode(entry["content"], validate=True))
            self.assertEqual(len(raw), entry["size_bytes"])
            self.assertEqual(hashlib.sha256(raw).hexdigest(), entry["sha256"])
            return raw

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            repo = root / "source"
            repo.mkdir()

            def git(*args):
                return subprocess.check_output(["git", "--no-optional-locks", *args], cwd=repo,
                                               stderr=subprocess.PIPE)

            git("init", "-q")
            git("config", "core.fileMode", "true")
            (repo / ".gitignore").write_text("generated/\n")
            for name in ("staged.txt", "unstaged.txt", "both.txt", "removed.txt", "renamed.txt", "script.sh"):
                (repo / name).write_text("first\nsecond\n")
            (repo / "binary.dat").write_bytes(b"\0before\xff")
            git("add", ".")
            git("-c", "user.name=Test", "-c", "user.email=test@example.invalid",
                "-c", "commit.gpgsign=false", "commit", "-qm", "base")
            clean = R.capture_source_state(repo)
            self.assertFalse(clean["worktree_dirty"])
            self.assertEqual(decode(clean["tracked_patch"]), b"")
            self.assertEqual(clean["untracked_files"], {})

            (repo / "staged.txt").write_text("staged edit\n")
            (repo / "both.txt").write_text("staged first\nsecond\n")
            (repo / "new_tracked.txt").write_text("staged new file\n")
            git("add", "staged.txt", "both.txt", "new_tracked.txt")
            git("mv", "renamed.txt", "new_name.txt")
            (repo / "both.txt").write_text("staged first\nunstaged second\n")
            (repo / "unstaged.txt").write_text("unstaged edit\n")
            (repo / "new_tracked.txt").write_text("staged new file\nthen unstaged edit\n")
            (repo / "removed.txt").unlink()
            (repo / "script.sh").chmod(0o755)
            (repo / "binary.dat").write_bytes(b"\0after\x80\xfe")
            (repo / "new directory").mkdir()
            (repo / "new directory/module\nname.py").write_text("print('π')\n")
            (repo / "new directory/data.bin").write_bytes(b"\xff\x80\0")
            (repo / "new directory/executable").write_text("#!/bin/sh\nexit 0\n")
            (repo / "new directory/executable").chmod(0o750)
            outside = root / "outside.txt"
            outside.write_text("do not follow this symlink")
            (repo / "outside-link").symlink_to(outside)
            (repo / "generated").mkdir()
            (repo / "generated/ignored.txt").write_text("not part of source capture")
            before_index = (repo / ".git/index").read_bytes()
            before_status = git("status", "--porcelain=v1", "--untracked-files=all", "-z")

            saved = R.capture_source_state(repo)
            self.assertEqual((repo / ".git/index").read_bytes(), before_index)
            self.assertEqual(saved["base_commit"], clean["base_commit"])
            self.assertTrue(saved["worktree_dirty"])
            self.assertEqual(decode(saved["git_status"]), before_status)
            source_patch = decode(saved["tracked_patch"])
            self.assertIn(b"GIT binary patch", source_patch)
            self.assertEqual(set(saved["untracked_files"]), {
                "new directory/module\nname.py", "new directory/data.bin",
                "new directory/executable", "outside-link"})
            self.assertEqual(saved["untracked_files"]["new directory/data.bin"]["encoding"], "base64")
            self.assertEqual(decode(saved["untracked_files"]["outside-link"]), os.fsencode(outside))

            restored = root / "restored"
            subprocess.run(["git", "clone", "-q", "--no-local", str(repo), str(restored)],
                           check=True, capture_output=True)
            subprocess.run(["git", "apply", "--binary", "-"], cwd=restored, input=source_patch,
                           check=True, capture_output=True)
            for name, entry in saved["untracked_files"].items():
                path = restored / name
                path.parent.mkdir(parents=True, exist_ok=True)
                if entry["kind"] == "symlink":
                    os.symlink(decode(entry), os.fsencode(path))
                else:
                    path.write_bytes(decode(entry))
                    path.chmod(int(entry["mode"], 8))
            names = [os.fsdecode(name) for name in git("ls-files", "-z").split(b"\0") if name]
            for name in {*names, *saved["untracked_files"]}:
                original, copy_path = repo / name, restored / name
                self.assertEqual(original.is_symlink(), copy_path.is_symlink(), name)
                self.assertEqual(original.exists(), copy_path.exists(), name)
                if original.is_symlink():
                    self.assertEqual(original.readlink(), copy_path.readlink(), name)
                elif original.exists():
                    self.assertEqual(original.read_bytes(), copy_path.read_bytes(), name)
                    self.assertEqual(original.stat().st_mode & 0o7777, copy_path.stat().st_mode & 0o7777, name)
            self.assertFalse((restored / "renamed.txt").exists())
            self.assertFalse((restored / "generated").exists())

    def test_source_capture_failure_prevents_output_and_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            self.copy_demo_configs("matched_single_obstacle_xarm6_t1", repo)
            out = repo / "output"
            with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                    patch.object(R, "capture_source_state", side_effect=RuntimeError("capture failed")), \
                    patch.object(R, "logged_command") as launch, \
                    self.assertRaisesRegex(RuntimeError, "capture failed"):
                R.run_one("single_obstacle", "exponential", 1, 1, out)
            launch.assert_not_called()
            self.assertFalse(out.exists())

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
        for command in ("build", "scenes", "check", "run", "campaign", "postprocess", "render", "visualize_mesh"):
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

    def test_logged_command_forwards_only_progress_live_and_preserves_log_and_exit_code(self):
        first = "[sugar_box] step=0010 sim=0.36s wall=1.2s pos_err=0.842m yaw_err=4.2deg within_goal=no"
        second = "[sugar_box] step=0020 sim=0.72s wall=2.4s pos_err=0.020m yaw_err=2.9deg within_goal=yes"
        flushed = threading.Event()

        class Output(io.StringIO):
            def flush(self):
                if first in self.getvalue():
                    flushed.set()
                super().flush()

        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            gate = directory / "allow_exit"
            script = directory / "progress_child.py"
            script.write_text(
                "import sys, time\nfrom pathlib import Path\n"
                "print('recorder startup noise', flush=True)\n"
                f"print({first!r}, flush=True)\n"
                "deadline = time.monotonic() + 5\n"
                "while not Path(sys.argv[1]).exists():\n"
                "    if time.monotonic() > deadline: sys.exit(99)\n"
                "    time.sleep(.01)\n"
                "print('[GOAL-YAW] goal_yaw_degrees=90', flush=True)\n"
                "print(' prefix [sugar_box] step=0030 ignored', flush=True)\n"
                "print('[sugar_box] step=bad ignored', flush=True)\n"
                "print('SUCCESS t=0.36', flush=True)\n"
                "print('stderr details', file=sys.stderr, flush=True)\n"
                f"print({second!r}, flush=True)\n"
                "sys.exit(17)\n"
            )
            log = directory / "launcher.log"
            output, outcomes = Output(), []

            def run():
                try:
                    outcomes.append(R.logged_command([sys.executable, str(script), str(gate)],
                                                     log, R.os.environ.copy()))
                except BaseException as exc:
                    outcomes.append(exc)

            with patch.object(R, "REPO", directory), redirect_stdout(output):
                worker = threading.Thread(target=run, daemon=True)
                worker.start()
                try:
                    self.assertTrue(flushed.wait(timeout=3), "Progress was buffered until child exit")
                    self.assertTrue(worker.is_alive(), "Child exited before its release gate")
                    self.assertFalse(gate.exists())
                finally:
                    gate.touch()
                    worker.join(timeout=6)
            self.assertFalse(worker.is_alive())
            self.assertEqual(outcomes, [17])
            self.assertEqual(output.getvalue().splitlines(), [first, second])
            lines = log.read_text().splitlines()
            self.assertTrue(lines[0].startswith("[COMMAND] "))
            self.assertTrue(lines[1].startswith("[CWD] "))
            self.assertEqual(lines[2:], ["recorder startup noise", first,
                "[GOAL-YAW] goal_yaw_degrees=90", " prefix [sugar_box] step=0030 ignored",
                "[sugar_box] step=bad ignored", "SUCCESS t=0.36", "stderr details", second])

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
        recorder = Path(__file__).with_name("record_metrics.py")
        with tempfile.TemporaryDirectory() as tmp:
            steps_file, trace_file = Path(tmp) / "steps.jsonl", Path(tmp) / "trace.jsonl"
            argv = [str(recorder), "--goal", *map(str, goal), "--object-name", "sugar_box_base",
                    "--out-steps", str(steps_file), "--out-trace", str(trace_file),
                    "--url", "memq://fake", "--duration", "10"]
            output = io.StringIO()
            with patch.dict(sys.modules, {"pydrake": pydrake, "pydrake.lcm": lcm}), \
                    patch.object(sys, "argv", argv), patch("time.time", side_effect=lambda: clock.wall), \
                    patch("time.monotonic", side_effect=lambda: clock.monotonic), redirect_stdout(output):
                namespace = runpy.run_path(str(recorder), run_name="__main__")
            try:
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
            finally:
                namespace["steps_f"].close()
                namespace["trace_f"].close()

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
                self.copy_demo_configs("matched_single_obstacle_xarm6_t2", repo)
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
                        patch.object(R, "compact_run") as compact, \
                        patch.object(R, "capture_source_state", return_value=self.source_state_fixture()):
                    if condition == "correct":
                        status = R.run_one("single_obstacle", "relu", 2, 2, out, goal_yaw_degrees=yaw)
                        self.assertTrue(status["goal_yaw_verified"])
                    else:
                        with self.assertRaises(RuntimeError):
                            R.run_one("single_obstacle", "relu", 2, 2, out, goal_yaw_degrees=yaw)
                saved = json.loads((out / "runtime_status.json").read_text())
                self.assertEqual(saved["goal_yaw_degrees"], yaw)
                self.assertEqual(compact.call_count, int(condition == "correct"))
                self.assertFalse((out / "RUN_COMPLETE").exists())
                self.assertEqual(phases[0][1][-1], str(yaw))
                self.assertEqual(phases[0][1][-2], "--goal-yaw-degrees")
                self.assertIn("--controller-params", phases[0][1])
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
                                         "0", "600", "19001", str(out), "--goal-yaw-degrees", yaw],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("must be" if yaw == "45" else "Rebuild", result.stderr)
                self.assertFalse(out.exists())

    def test_packaging_uses_recorded_goal_and_run_temporary_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            self.copy_demo_configs("matched_single_obstacle_xarm6_t1", repo)
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

            def capture(repo_path):
                self.assertEqual(repo_path, repo)
                self.assertFalse(out.exists())
                saved = self.source_state_fixture()
                saved["worktree_dirty"] = True
                return saved

            with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                    patch.object(R, "logged_command", side_effect=logged), \
                    patch.object(R, "compact_run") as compact, \
                    patch.dict(R.os.environ, {"C3PLUS_CONTAINER_IMAGE": "test:tag",
                                              "C3PLUS_CONTAINER_IMAGE_ID": "sha256:test",
                                              "SAMPLING_C3_OBS_BOXES": "stale"}), \
                    patch.object(R, "capture_source_state", side_effect=capture) as capture_call:
                status = R.run_one("single_obstacle", "exponential", 1, 1, out)
            capture_call.assert_called_once_with(repo)
            source_bytes = (out / "config/source_state.json").read_bytes()
            self.assertEqual(status["source_state"]["sha256"], hashlib.sha256(source_bytes).hexdigest())
            self.assertEqual(status["source_state"]["size_bytes"], len(source_bytes))
            self.assertEqual(status["source_state"]["path"], "config/source_state.json")
            self.assertEqual(status["commit"], "test-commit")
            self.assertTrue(status["worktree_dirty"])
            compact.assert_called_once_with(out, status["run_id"], status=status,
                                            require_legacy_complete=False)
            self.assertFalse((out / "RUN_COMPLETE").exists())
            self.assertEqual(status["runtime"]["container_image"], "test:tag")
            self.assertEqual(status["runtime"]["container_image_id"], "sha256:test")
            self.assertEqual(status["goal"], status["controller_goal"])
            self.assertEqual(status["obstacle_cost"], "exponential")
            self.assertEqual(status["sampler_settings"]["SAMPLING_C3_OBS_BOXES"], "0.35,0,0.05,0.05")
            self.assertEqual(calls[0][2]["SAMPLING_C3_OBS_BOXES"], "0.35,0,0.05,0.05")
            self.assertEqual(calls[0][1][-2:], ["--controller-params", status["controller_params_file"]])
            self.assertTrue(status["config_sha256"])
            controller = R.yaml.safe_load(Path(status["controller_params_file"]).read_text())
            native_goal = R.yaml.safe_load(Path(controller["goal_params_file"]).read_text())
            self.assertEqual(native_goal["fixed_target_position"][:2], list(status["goal"][:2]))
            self.assertEqual(calls[0][1][4:7], list(map(str, status["goal"])))
            for _, _, env in calls[1:]:
                self.assertEqual(env["TMPDIR"], str(out / "tmp"))
            render = next(command for phase, command, _ in calls if phase == "render.log")
            index = render.index("--goal")
            self.assertEqual(render[index + 1:index + 4], list(map(str, status["goal"])))
            self.assertEqual(render[render.index("--result") + 1], str(out / f"{status['run_id']}_result.json"))
            self.assertEqual([phase for phase, _, _ in calls], ["launcher.log", "postprocess.log", "render.log"])

    def test_launcher_requires_config_support_in_each_native_binary(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            launcher = repo / "tools/experiments/launch_run.sh"
            launcher.parent.mkdir(parents=True)
            shutil.copy2(S.TOOL_DIR / "launch_run.sh", launcher)
            binary_dir = repo / "bazel-bin/examples/sampling_c3"
            binary_dir.mkdir(parents=True)
            config = repo / "controller.yaml"
            config.write_text("{}\n")
            for stale in S.BINARIES:
                for name in S.BINARIES:
                    binary = binary_dir / name
                    flag = "old_flags" if name == stale else "controller_params"
                    binary.write_text(f"#!/bin/sh\n[ \"$1\" = --helpshort ] || exit 99\necho ' -{flag} (path)'\nexit 1\n")
                    binary.chmod(0o755)
                out = repo / stale
                result = subprocess.run(["bash", str(launcher), "unused", "object", "0", "0", "0",
                                         "600", "19001", str(out), "--controller-params", str(config)],
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn(stale, result.stderr)
                self.assertIn("Rebuild", result.stderr)
                self.assertFalse(out.exists())

    def test_launcher_passes_same_snapshot_to_all_native_processes(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "checkout with spaces"
            launcher = repo / "tools/experiments/launch_run.sh"
            launcher.parent.mkdir(parents=True)
            shutil.copy2(S.TOOL_DIR / "launch_run.sh", launcher)
            binaries = repo / "bazel-bin/examples/sampling_c3"
            binaries.mkdir(parents=True)
            for name in S.BINARIES:
                binary = binaries / name
                binary.write_text("#!/bin/sh\n[ \"$1\" = --helpshort ] || exit 99\necho ' -controller_params (path) -goal_yaw_degrees (degrees)'\nexit 1\n")
                binary.chmod(0o755)
            stubs = repo / "stubs"
            stubs.mkdir()
            capture = repo / "calls.jsonl"
            setsid = stubs / "setsid"
            setsid.write_text(f"#!{sys.executable}\nimport json, os, sys\n"
                              "with open(os.environ['TEST_LAUNCH_CAPTURE'], 'a') as f:\n"
                              "    f.write(json.dumps(sys.argv[1:]) + '\\n')\n")
            setsid.chmod(0o755)
            sleep = stubs / "sleep"
            sleep.write_text("#!/bin/sh\nexit 0\n")
            sleep.chmod(0o755)
            config = repo / "controller with spaces.yaml"
            config.write_text("{}\n")
            out = repo / "run"
            env = dict(R.os.environ, PATH=str(stubs) + ":" + R.os.environ["PATH"],
                       TEST_LAUNCH_CAPTURE=str(capture))
            result = subprocess.run(["bash", str(launcher), S.demo_name("single_obstacle", 2, 2),
                                     "object", "0.397", "-0.431", "0", "600", "19001", str(out),
                                     "--controller-params", str(config), "--goal-yaw-degrees", "0"],
                                    text=True, capture_output=True, env=env)
            self.assertEqual(result.returncode, 0, result.stderr)
            calls = [json.loads(line) for line in capture.read_text().splitlines()]
            native = {Path(call[0]).name: call for call in calls if Path(call[0]).name in S.BINARIES}
            self.assertEqual(set(native), set(S.BINARIES))
            for call in native.values():
                self.assertIn(f"--controller_params={config}", call)
            self.assertIn("--goal_yaw_degrees=0", native["franka_sampling_c3_controller"])
            self.assertNotIn("--goal_yaw_degrees=0", native["franka_sim"])

    def test_launcher_tees_recorder_progress_preserves_failure_and_cleans_process_groups(self):
        if not shutil.which("setsid"):
            self.skipTest("setsid is unavailable")
        progress = "[sugar_box] step=0010 sim=0.36s wall=1.2s pos_err=0.842m yaw_err=4.2deg within_goal=no"
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "stub checkout"
            launcher = repo / "tools/experiments/launch_run.sh"
            launcher.parent.mkdir(parents=True)
            shutil.copy2(S.TOOL_DIR / "launch_run.sh", launcher)
            binaries = repo / "bazel-bin/examples/sampling_c3"
            binaries.mkdir(parents=True)
            captures = repo / "processes"
            captures.mkdir()
            native_source = (f"#!{sys.executable}\n"
                "import json, os, signal, sys\nfrom pathlib import Path\n"
                "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
                "record = {'pid': os.getpid(), 'pgid': os.getpgrp()}\n"
                "path = Path(os.environ['TEST_PROCESS_CAPTURE']) / ('native_' + Path(sys.argv[0]).name + '.json')\n"
                "path.write_text(json.dumps(record))\n"
                "signal.pause()\n")
            for name in S.BINARIES:
                binary = binaries / name
                binary.write_text(native_source)
                binary.chmod(0o755)
            stubs = repo / "stubs"
            stubs.mkdir()
            sleep = stubs / "sleep"
            sleep.write_text("#!/bin/sh\nexit 0\n")
            sleep.chmod(0o755)
            python = stubs / "recorder_python"
            python.write_text(f"#!{sys.executable}\n"
                "import json, os, sys, time\nfrom pathlib import Path\n"
                "directory = Path(os.environ['TEST_PROCESS_CAPTURE'])\n"
                "(directory / 'recorder.json').write_text(json.dumps({'pid': os.getpid(), 'pgid': os.getpgrp()}))\n"
                "deadline = time.monotonic() + 5\n"
                "while len(list(directory.glob('native_*.json'))) != 3:\n"
                "    if time.monotonic() > deadline: sys.exit(98)\n"
                "    time.sleep(.01)\n"
                f"print({progress!r}, flush=True)\n"
                "print('synthetic recorder stderr', file=sys.stderr, flush=True)\n"
                "print('FINAL {\"control_steps\": 10}', flush=True)\n"
                "sys.exit(17)\n")
            python.chmod(0o755)
            out = repo / "output"
            env = dict(R.os.environ, PATH=str(stubs) + ":" + R.os.environ["PATH"],
                       PYTHON=str(python), TEST_PROCESS_CAPTURE=str(captures))
            command = ["bash", str(launcher), "open_table_mesh_sugar_box_xarm6_t2", "sugar_box_base",
                       ".4", "-.4", "0", "600", "19001", str(out)]
            try:
                result = subprocess.run(command, capture_output=True, text=True, env=env, timeout=10)
                self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
                self.assertEqual((out / "recorder.log").read_text().splitlines(),
                                 [progress, "synthetic recorder stderr", 'FINAL {"control_steps": 10}'])
                for line in (progress, "synthetic recorder stderr", 'FINAL {"control_steps": 10}'):
                    self.assertEqual(result.stdout.count(line), 1)
                processes = [json.loads(path.read_text()) for path in captures.glob("*.json")]
                self.assertEqual(len(processes), 4)
                self.assertEqual(len({process["pgid"] for process in processes}), 4)
                for process in processes:
                    stat = Path(f"/proc/{process['pid']}/stat")
                    if stat.exists():
                        self.assertEqual(stat.read_text().rsplit(")", 1)[1].split()[0], "Z",
                                         f"Stub process remained alive: {process}")
                recorder_group = json.loads((captures / "recorder.json").read_text())["pgid"]
                with self.assertRaises(ProcessLookupError):
                    R.os.killpg(recorder_group, 0)
            finally:
                # Limit emergency cleanup to process groups created by this test.
                for path in captures.glob("*.json"):
                    try:
                        R.os.killpg(json.loads(path.read_text())["pgid"], 9)
                    except ProcessLookupError:
                        pass

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
                    configs = S.load_demo_configs(demo_name)
                    source, pose = S.load_controller_goal(demo_name)
                    self.assertIn(source, configs)
                    self.assertEqual(source.name, "experiments.yaml")
                    resolved = S.compose_demo_configs(demo_name)
                    self.assertEqual(resolved["goal"]["fixed_target_position"][:2], list(pose[:2]))
                    self.assertEqual(len(S.load_controller_goal(demo_name)[1]), 3)

    def test_shared_demo_config_graph_rejects_missing_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            demo = "matched_single_obstacle_xarm6_t2"
            self.copy_demo_configs(demo, repo)
            configs = S.load_demo_configs(demo, repo=repo)
            nested = next(path for path in configs if path.name == "progress_params_c3plus.yaml")
            nested.unlink()
            with self.assertRaises(FileNotFoundError):
                S.load_demo_configs(demo, repo=repo)

    def test_start_goal_and_orientation_compose_independently(self):
        for scene in S.SCENES:
            original = S.compose_demo_configs(S.demo_name(scene, 2, 2))
            other_start = S.compose_demo_configs(S.demo_name(scene, 3, 2))
            other_goal = S.compose_demo_configs(S.demo_name(scene, 2, 3))
            self.assertEqual(original["goal"], other_start["goal"])
            self.assertEqual(original["simulation"], other_goal["simulation"])
            self.assertNotEqual(original["simulation"]["q_init_object"][4:],
                                other_start["simulation"]["q_init_object"][4:])
            self.assertNotEqual(original["goal"]["fixed_target_position"],
                                other_goal["goal"]["fixed_target_position"])
            for yaw in (90, 0, -90):
                rotated = S.compose_demo_configs(S.demo_name(scene, 2, 2), goal_yaw_degrees=yaw)
                self.assertEqual(rotated["simulation"], original["simulation"])
                self.assertEqual(rotated["controller"], original["controller"])
                goal = dict(rotated["goal"])
                quaternion = goal.pop("fixed_target_orientation")
                self.assertEqual(goal.pop("fixed_target_orientations"), [quaternion])
                expected = {k: v for k, v in original["goal"].items()
                            if k not in ("fixed_target_orientation", "fixed_target_orientations")}
                self.assertEqual(goal, expected)
                self.assertAlmostEqual(2 * math.atan2(quaternion[3], quaternion[0]), math.radians(yaw))

    def test_composition_preserves_scene_exceptions(self):
        def resolve(scene, start=2, goal=2):
            return S.compose_demo_configs(S.demo_name(scene, start, goal))
        self.assertTrue(resolve("open_task", 1, 1)["simulation"]["visualize_drake_sim"])
        self.assertFalse(resolve("open_task", 1, 2)["simulation"]["visualize_drake_sim"])
        self.assertEqual(resolve("slalom", 4)["simulation"]["q_init_object"][4:], [0.361, 0.37, 0.0008])
        self.assertEqual(resolve("slalom", goal=4)["goal"]["fixed_target_position"], [0.367, -0.38, 0.0008])
        self.assertEqual(resolve("shelf_gap", goal=3)["goal"]["fixed_target_position"], [0.363, -0.41, 0.0008])
        self.assertEqual(resolve("ycb_clutter")["goal"]["fixed_target_orientation"], [-0.049184, 0, 0, 0.99879])
        self.assertEqual(resolve("single_obstacle")["goal"]["fixed_target_orientation"], [-0.0491838, 0, 0, 0.99879])
        self.assertEqual(resolve("icra_sign", 4)["simulation"]["q_init_franka"],
                         [0.972422, 0.310762, -1.230368, 0.06679, 0.916921])

    def test_resolved_yaml_snapshot_is_complete_and_independent_of_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "source checkout"
            demo = S.demo_name("shelf_gap", 2, 2)
            self.copy_demo_configs(demo, repo)
            out = Path(tmp) / "saved run"
            controller_path = S.write_demo_configs(demo, out, repo=repo, goal_yaw_degrees=-90)
            def resolve(value):
                if isinstance(value, dict):
                    return {k: resolve(v) for k, v in value.items()}
                if isinstance(value, list):
                    return [resolve(v) for v in value]
                if isinstance(value, str) and value.endswith(".yaml"):
                    path = Path(value)
                    self.assertTrue(path.is_absolute())
                    self.assertTrue(path.is_relative_to(out))
                    return resolve(R.yaml.safe_load(path.read_text()))
                return value
            controller = R.yaml.safe_load(controller_path.read_text())
            resolved = resolve(controller)
            expected = S.compose_demo_configs(demo, repo=repo, goal_yaw_degrees=-90)
            self.assertEqual(resolved["goal_params_file"], expected["goal"])
            self.assertEqual(resolved["sim_params_file"], expected["simulation"])
            shutil.rmtree(repo)
            self.assertEqual(resolve(controller), resolved)
            with self.assertRaises(FileExistsError):
                S.write_demo_configs(demo, out)

    def test_configuration_digest_tracks_only_selected_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            demo = S.demo_name("single_obstacle", 2, 2)
            self.copy_demo_configs(demo, repo)
            source = repo / S.EXPERIMENTS_FILE
            data = R.yaml.safe_load(source.read_text())
            original = S.demo_config_digest(demo, repo=repo)
            data["start_positions"]["icra_sign_s01"][0] += 0.01
            source.write_text(R.yaml.safe_dump(data))
            self.assertEqual(S.demo_config_digest(demo, repo=repo), original)
            data["start_positions"]["t_shape_s02"][0] += 0.01
            source.write_text(R.yaml.safe_dump(data))
            changed = S.demo_config_digest(demo, repo=repo)
            self.assertNotEqual(changed, original)
            dependency = next(path for path in S.load_demo_configs(demo, repo=repo)
                              if path.name == "progress_params_c3plus.yaml")
            progress = R.yaml.safe_load(dependency.read_text())
            progress["num_control_loops_to_wait"] += 1
            dependency.write_text(R.yaml.safe_dump(progress))
            self.assertNotEqual(S.demo_config_digest(demo, repo=repo), changed)

    def test_catalogue_orientation_selection_and_invalid_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            demo = S.demo_name("single_obstacle", 2, 2)
            self.copy_demo_configs(demo, repo)
            source = repo / S.EXPERIMENTS_FILE
            data = R.yaml.safe_load(source.read_text())
            data["scenes"]["single_obstacle"]["start_orientations"] = {2: "start_s01"}
            source.write_text(R.yaml.safe_dump(data))
            resolved = S.compose_demo_configs(demo, repo=repo)
            self.assertEqual(resolved["simulation"]["q_init_object"], [1, 0, 0, 0, 0.366, 0.431, 0.0008])
            self.assertEqual(resolved["goal"]["fixed_target_position"], [0.397, -0.431, 0.0008])
            for invalid in ([0, 0, 0, 0], [1, 0], [float("nan"), 0, 0, 0]):
                data["orientations"]["start_s01"] = invalid
                source.write_text(R.yaml.safe_dump(data))
                with self.assertRaises(ValueError):
                    S.compose_demo_configs(demo, repo=repo)
            data["orientations"]["start_s01"] = [1, 0, 0, 0]
            data["defaults"]["goal"]["goal_mode"] = 1
            source.write_text(R.yaml.safe_dump(data))
            with self.assertRaisesRegex(ValueError, "fixed"):
                S.compose_demo_configs(demo, repo=repo, goal_yaw_degrees=90)

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
            run_id = "exponential_open_task_s01g01_seed42"
            (done / f"{run_id}_result.json").write_text(json.dumps({"run_id": run_id}))
            (done / f"{run_id}.mp4").write_bytes(b"legacy video fixture")
            argv = ["run_grid_campaign.py", "--output-root", tmp,
                    "--scenes", "open_task", "--resume"]
            with patch("sys.argv", argv), patch.object(G, "run_one", return_value={"failures": []}) as run:
                G.main()
            self.assertEqual(run.call_count, 1)
            self.assertEqual(run.call_args.args[1], "relu")


class MeshPreviewTests(unittest.TestCase):
    def settings(self, *flags):
        stream = io.StringIO()
        with patch.object(V, "render_image") as viewer, redirect_stdout(stream):
            V.main([*flags, "--dry-run"])
        viewer.assert_not_called()
        return json.loads(stream.getvalue())

    def test_help_without_drake(self):
        result = subprocess.run(
            [sys.executable, "-S", str(S.TOOL_DIR / "visualize_mesh.py"), "--help"],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--mesh", result.stdout)

    def test_renamed_cli_dry_run_creates_no_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = Path(tmp) / "new" / "object.png"
            result = subprocess.run(
                [sys.executable, "-m", "tools.experiments", "visualize_mesh",
                 "--output", str(image), "--view", "top", "--ee-samples", "--dry-run"],
                cwd=S.REPO, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            settings = json.loads(result.stdout)
            self.assertEqual(settings["output"], str(image))
            self.assertEqual(settings["view"], "top")
            self.assertEqual(settings["ee_sampling"]["count"], 64)
            self.assertEqual(settings["ee_sampling"]["seed"], 42)
            self.assertFalse(image.parent.exists())
            self.assertNotIn("visualize", cli.COMMANDS)

    def test_independent_start_and_goal_placements(self):
        start = self.settings("--scene", "open_task", "--start", "2", "--goal", "5")
        self.assertEqual(start["position_m"], [0.366, 0.431, 0.0008])
        goal = self.settings("--scene", "open_task", "--start", "5", "--pose", "goal",
                             "--goal", "2", "--goal-yaw-degrees", "-90")
        self.assertEqual(goal["position_m"][:2], [0.397, -0.431])
        self.assertAlmostEqual(goal["quaternion_wxyz"][0], math.sqrt(0.5))
        self.assertAlmostEqual(goal["quaternion_wxyz"][3], -math.sqrt(0.5))
        self.assertTrue(self.settings("--scene", "icra_sign")["object_file"].endswith("push_c_glyph.sdf"))

    def test_invalid_preview_options_fail_before_viewer(self):
        mesh = str(S.REPO / "examples/sampling_c3/urdf/c_glyph_base/c_glyph_base.obj")
        cases = (("--mesh-scale", "0.001"), ("--mesh", mesh, "--mesh-scale", "0"),
                 ("--mesh", mesh, "--mesh-scale", "nan"), ("--position", "nan", "0", "0"),
                 ("--goal-yaw-degrees", "90"), ("--width", "0"), ("--height", "-1"),
                 ("--output", "image.jpg"), ("--mesh", mesh, "--geometry", "collision"),
                 ("--ee-samples", "0"), ("--ee-samples", "10001"),
                 ("--sample-seed", "42"), ("--ee-samples", "--sample-seed", "-1"),
                 ("--sample-height", "-0.012"),
                 ("--mesh", mesh, "--ee-samples", "--position", "0.4", "0", "0", "--sample-height", "nan"),
                 ("--mesh", mesh, "--ee-samples"),
                 ("--model", str(S.REPO / "examples/sampling_c3/urdf/push_c_glyph.sdf"), "--ee-samples"),
                 ("--model", mesh), ("--mesh", "/missing/object.obj"),
                 ("--pose", "goal", "--goal-yaw-degrees", "90", "--rpy-degrees", "0", "0", "0"))
        for flags in cases:
            with self.subTest(flags=flags), patch.object(V, "render_image") as viewer, \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                V.main(list(flags))
            self.assertEqual(error.exception.code, 1)
            viewer.assert_not_called()

    def test_geometry_and_camera_poses_without_dynamics(self):
        try:
            import numpy as np
            from pydrake.common.eigen_geometry import Quaternion
            from pydrake.math import RollPitchYaw, RotationMatrix
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        with tempfile.TemporaryDirectory() as tmp:
            # An open mesh needs no volume for a visual preview. Its unused
            # normal declaration must not prevent repairing face normals.
            mesh = Path(tmp) / 'open & "quoted" mesh.obj'
            mesh.write_text("v 0 0 0\nv 0.01 0 0\nv 0 0.01 0\nvn 0 0 1\nf 1 2 3\n")
            mjcf = Path(tmp) / "object.xml"
            mjcf.write_text('<mujoco model="preview_test"><worldbody><body name="test_object">'
                            '<freejoint/><geom type="box" size=".01 .02 .03"/>'
                            '</body></worldbody></mujoco>')
            cases = [(self.settings("--scene", scene, "--start", "2"),
                      "c_glyph_base" if scene == "icra_sign" else "vertical_link")
                     for scene in S.SCENES]
            for scene, body in (("open_task", "vertical_link"), ("icra_sign", "c_glyph_base")):
                for yaw in (90, 0, -90):
                    cases.append((self.settings("--scene", scene, "--pose", "goal", "--goal", "2",
                                                "--goal-yaw-degrees", str(yaw)), body))
            cases.extend([
                (self.settings("--mesh", str(mesh), "--mesh-scale", "0.001",
                               "--position", "20", "-11", "0.0008", "--rpy-degrees", "10", "20", "30"), "object"),
                (self.settings("--model", str(mjcf)), "test_object"),
            ])
            for settings, body_name in cases:
                settings.update(width=320, height=240, frames=True)
                with self.subTest(scene=settings["scene"], pose=settings["pose"], model=body_name), \
                        tempfile.TemporaryDirectory(dir=tmp) as assets:
                    diagram, plant, sensor = V.build_preview(settings, assets)
                    context = diagram.CreateDefaultContext()
                    plant_context = plant.GetMyMutableContextFromRoot(context)
                    body = plant.GetBodyByName(body_name)
                    pose = plant.EvalBodyPoseInWorld(plant_context, body)
                    np.testing.assert_allclose(pose.translation(), settings["position_m"], atol=1e-8, rtol=0)
                    if settings["rpy_degrees"] is None:
                        quat = np.asarray(settings["quaternion_wxyz"])
                        expected = RotationMatrix(Quaternion(quat / np.linalg.norm(quat)))
                    else:
                        expected = RollPitchYaw(np.radians(settings["rpy_degrees"])).ToRotationMatrix()
                    np.testing.assert_allclose(pose.rotation().matrix(), expected.matrix(), atol=5e-8, rtol=0)
                    for index, angle in enumerate(settings["robot_joints_rad"], 1):
                        self.assertAlmostEqual(plant.GetJointByName(f"xarm6_joint{index}").get_angle(plant_context), angle)
                    for name, height in (("ground", -0.029), ("platform", -0.0145)):
                        position = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName(name)).translation()
                        np.testing.assert_allclose(position, [0, 0, height], atol=1e-12)
                    self.assertEqual(context.get_time(), 0)
                    pixels = sensor.color_image_output_port().Eval(sensor.GetMyContextFromRoot(context)).data
                    self.assertEqual(pixels.shape, (240, 320, 4))
                    self.assertGreater(np.unique(pixels[:, :, :3].reshape(-1, 3), axis=0).shape[0], 10)

    def test_ee_candidate_clearance_and_coordinate_transforms(self):
        try:
            import pydrake.common
            import numpy as np
            from pydrake.math import RollPitchYaw
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")

        def box_distance(points, center, half_size):
            offset = np.abs(points - center) - half_size
            return np.linalg.norm(np.maximum(offset, 0), axis=1) + np.minimum(offset.max(axis=1), 0)

        for scene in ("open_task", "icra_sign"):
            for yaw in (90, 0, -90):
                with self.subTest(scene=scene, yaw=yaw):
                    settings = self.settings("--scene", scene, "--pose", "goal", "--goal", "2",
                                             "--goal-yaw-degrees", str(yaw), "--ee-samples", "32")
                    first = V.sample_ee_candidates(settings, 32, 42)
                    self.assertEqual(first, V.sample_ee_candidates(settings, 32, 42))
                    world, local = np.array(first["points_world"]), np.array(first["points_object"])
                    rotation = RollPitchYaw(0, 0, math.radians(yaw)).ToRotationMatrix().matrix()
                    np.testing.assert_allclose(local @ rotation.T + settings["position_m"], world, atol=1e-12)
                    self.assertEqual(world.shape, (32, 3))
                    height = 0.005 if scene == "open_task" else -0.012
                    np.testing.assert_allclose(world[:, 2], height, atol=1e-12)
                    self.assertTrue(np.all((world[:, 0] >= 0.17) & (world[:, 0] <= 0.73)))
                    self.assertTrue(np.all((world[:, 1] >= -0.58) & (world[:, 1] <= 0.58)))
                    radius = np.linalg.norm(world[:, :2], axis=1)
                    self.assertTrue(np.all((radius >= 0.27) & (radius <= 0.68)))
                    if scene == "open_task":
                        distances = np.minimum(
                            box_distance(local, [0, 0.0099, 0], [0.0445, 0.0099, 0.0298]),
                            box_distance(local, [0, -0.0397, 0], [0.0099, 0.0397, 0.0298]))
                        self.assertTrue(np.all(distances - 0.00555 > 0.019 - 1e-12))
                    else:
                        distances = np.minimum.reduce([
                            box_distance(local, [-0.0323, 0, 0], [0.016, 0.0515, 0.0125]),
                            box_distance(local, [0, 0.0355, 0], [0.0483, 0.016, 0.0125]),
                            box_distance(local, [0, -0.0355, 0], [0.0483, 0.016, 0.0125]),
                        ])
                        self.assertTrue(np.all(distances > 0.027))
                    self.assertNotEqual(first["points_world"], V.sample_ee_candidates(settings, 32, 43)["points_world"])

    def test_raw_mesh_ee_samples_preserve_concavity(self):
        try:
            import numpy as np
            import trimesh
            import scipy
        except ImportError as exc:
            self.skipTest(f"Mesh runtime unavailable: {exc}")

        def box_distance(points, center, half_size):
            offset = np.abs(points - center) - half_size
            return np.linalg.norm(np.maximum(offset, 0), axis=1) + np.minimum(offset.max(axis=1), 0)

        # Build a closed concave prism without optional polygon triangulators.
        outline = np.array([[-0.0483, -0.0515], [-0.0163, -0.0515], [0.0483, -0.0515],
                            [0.0483, -0.0195], [-0.0163, -0.0195], [-0.0163, 0.0195],
                            [0.0483, 0.0195], [0.0483, 0.0515], [-0.0163, 0.0515], [-0.0483, 0.0515]])
        triangles = np.array([[0, 1, 4], [0, 4, 5], [0, 5, 8], [0, 8, 9],
                              [1, 2, 3], [1, 3, 4], [5, 6, 7], [5, 7, 8]])
        fixture = trimesh.creation.extrude_triangulation(outline, triangles, 0.025)
        fixture.apply_translation([0, 0, -0.0125])
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        mesh = str(Path(directory.name) / "concave.obj")
        fixture.export(mesh)
        for yaw in (0, 90, -90):
            with self.subTest(yaw=yaw):
                # A doubled C has a cavity wide enough to hold candidate tip
                # spheres; convex-hull distance would incorrectly reject them.
                settings = self.settings("--mesh", mesh, "--mesh-scale", "2", "--ee-samples", "128",
                                         "--position", "0.4", "0", "-0.004", "--rpy-degrees", "0", "0", str(yaw))
                report = V.sample_ee_candidates(settings, 128, 42)
                self.assertEqual(report, V.sample_ee_candidates(settings, 128, 42))
                world, local = np.array(report["points_world"]), np.array(report["points_object"])
                angle = math.radians(yaw)
                rotation = np.array([[math.cos(angle), -math.sin(angle), 0],
                                     [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
                np.testing.assert_allclose(local @ rotation.T + settings["position_m"], world, atol=1e-12)
                np.testing.assert_allclose(world[:, 2], -0.012, atol=1e-12)
                distances = np.minimum.reduce([
                    box_distance(local, [-0.0646, 0, 0], [0.032, 0.103, 0.025]),
                    box_distance(local, [0, 0.071, 0], [0.0966, 0.032, 0.025]),
                    box_distance(local, [0, -0.071, 0], [0.0966, 0.032, 0.025]),
                ])
                self.assertTrue(np.all(distances >= 0.027 - 1e-10))
                self.assertTrue(np.any((np.abs(local[:, 0]) < 0.0966) & (np.abs(local[:, 1]) < 0.103)))
                self.assertTrue(np.all(world[:, 2] - 0.00555 >= -0.029))
                self.assertTrue(np.all((world[:, 0] >= 0.17) & (world[:, 0] <= 0.73)))
                radial = np.linalg.norm(world[:, :2], axis=1)
                self.assertTrue(np.all((radial >= 0.27) & (radial <= 0.68)))
                self.assertNotEqual(report["points_world"], V.sample_ee_candidates(settings, 128, 43)["points_world"])

    def test_ee_overlay_and_json_sidecar(self):
        try:
            import pydrake.common
            import numpy as np
            from PIL import Image
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        with tempfile.TemporaryDirectory() as tmp:
            for scene in ("open_task", "icra_sign"):
                for geometry in ("visual", "collision"):
                    with self.subTest(scene=scene, geometry=geometry):
                        output = Path(tmp) / f"{scene}_{geometry}.png"
                        settings = self.settings("--scene", scene, "--view", "top", "--geometry", geometry,
                                                 "--ee-samples", "24", "--hide-robot", "--output", str(output),
                                                 "--width", "640", "--height", "480")
                        V.render_image(settings)
                        report = json.loads(output.with_suffix(".ee_samples.json").read_text())
                        self.assertEqual(report["count"], 24)
                        self.assertEqual(len(report["points_world"]), 24)
                        self.assertEqual(report["object_name"], settings["object_name"])
                        with Image.open(output) as image:
                            pixels = np.asarray(image)[80:]
                            blue = (pixels[:, :, 2] > 120) & (pixels[:, :, 1] > 70) & (pixels[:, :, 0] < 90)
                            self.assertGreater(int(blue.sum()), 10)

    def test_png_views_and_collision_output(self):
        try:
            import pydrake.common
            import numpy as np
            from PIL import Image
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        with tempfile.TemporaryDirectory() as tmp:
            for scene, view, geometry in (("open_task", "scene", "visual"),
                                          ("icra_sign", "object", "visual"),
                                          ("ycb_clutter", "top", "collision")):
                with self.subTest(scene=scene, view=view, geometry=geometry):
                    output = Path(tmp) / "images" / f"{scene}.png"
                    settings = self.settings("--scene", scene, "--view", view, "--geometry", geometry,
                                             "--frames", "--width", "320", "--height", "240",
                                             "--output", str(output))
                    self.assertEqual(V.render_image(settings), output)
                    with Image.open(output) as image:
                        self.assertEqual(image.format, "PNG")
                        self.assertEqual(image.size, (320, 240))
                        colors = np.asarray(image)[48:, :, :]
                        self.assertGreater(np.unique(colors.reshape(-1, 3), axis=0).shape[0], 10)


if __name__ == "__main__":
    unittest.main()
