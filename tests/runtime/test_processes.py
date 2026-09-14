from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from c3plus import configs as S
from c3plus.runtime import processes as Processes
from c3plus.experiments import run as R

from tests.fixtures.results import WorkflowFixtures, launch_subprocess

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
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
            with patch.object(Processes, "REPO", repo):
                rc = Processes.logged_command(command, log, R.os.environ.copy())
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
                    outcomes.append(Processes.logged_command([sys.executable, str(script), str(gate)],
                                                     log, R.os.environ.copy()))
                except BaseException as exc:
                    outcomes.append(exc)

            with patch.object(Processes, "REPO", directory), redirect_stdout(output):
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


    def test_yaw_launcher_rejects_stale_binary_without_starting_processes(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            launcher = repo / "c3plus/runtime/launcher.py"
            shutil.copytree(S.REPO / "c3plus", repo / "c3plus")
            controller = repo / ".build/bin/examples/sampling_c3/franka_sampling_c3_controller"
            controller.parent.mkdir(parents=True)
            controller.write_text("#!/bin/sh\n[ \"$1\" = --helpshort ] || exit 99\nprintf 'old controller flags\\n'\nexit 1\n")
            controller.chmod(0o755)
            for yaw in ("90", "0", "-90", "45"):
                out = repo / ("output" + yaw)
                result = launch_subprocess(["bash", str(launcher), "unused_demo", "object", "0.397", "-0.431",
                                         "0", "600", "19001", str(out), "--goal-yaw-degrees", yaw],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("must be" if yaw == "45" else "Rebuild", result.stderr)
                self.assertFalse(out.exists())


    def test_launcher_requires_config_support_in_each_native_binary(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            launcher = repo / "c3plus/runtime/launcher.py"
            shutil.copytree(S.REPO / "c3plus", repo / "c3plus")
            binary_dir = repo / ".build/bin/examples/sampling_c3"
            binary_dir.mkdir(parents=True)
            config = repo / "controller.yaml"
            config.write_text("{}\n")
            for stale in S.BINARIES:
                for name in S.BINARIES:
                    binary = binary_dir / name
                    flag = "old_flags" if name == stale else "controller_params"
                    binary.write_text(f"#!/bin/sh\n[ \"$1\" = --helpshort ] || exit 99\necho ' -{flag} (path) -execution_logging (bool)'\nexit 1\n")
                    binary.chmod(0o755)
                out = repo / stale
                result = launch_subprocess(["bash", str(launcher), "unused", "object", "0", "0", "0",
                                         "600", "19001", str(out), "--controller-params", str(config)],
                                        text=True, capture_output=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn(stale, result.stderr)
                self.assertIn("Rebuild", result.stderr)
                self.assertFalse(out.exists())


    def test_launcher_passes_same_snapshot_to_all_native_processes(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "checkout with spaces"
            launcher = repo / "c3plus/runtime/launcher.py"
            shutil.copytree(S.REPO / "c3plus", repo / "c3plus")
            binaries = repo / ".build/bin/examples/sampling_c3"
            binaries.mkdir(parents=True)
            for name in S.BINARIES:
                binary = binaries / name
                binary.write_text("#!/bin/sh\n[ \"$1\" = --helpshort ] || exit 99\necho ' -controller_params (path) -goal_yaw_degrees (degrees) -execution_logging (bool)'\nexit 1\n")
                binary.chmod(0o755)
            stubs = repo / "stubs"
            stubs.mkdir()
            capture = repo / "calls.jsonl"
            setsid = stubs / "setsid"
            setsid.write_text(f"#!{sys.executable}\nimport json, os, sys, time\n"
                              "with open(os.environ['TEST_LAUNCH_CAPTURE'], 'a') as f:\n"
                              "    f.write(json.dumps(sys.argv[1:]) + '\\n')\n"
                              # The launcher waits for the recorder. Keep its
                              # stub alive until all fake native spawns report.
                              "if sys.argv[1] == 'bash':\n"
                              "    deadline = time.monotonic() + 5\n"
                              "    while time.monotonic() < deadline:\n"
                              "        with open(os.environ['TEST_LAUNCH_CAPTURE']) as f:\n"
                              "            if len(f.readlines()) == 4: break\n"
                              "        time.sleep(.01)\n")
            setsid.chmod(0o755)
            sleep = stubs / "sleep"
            sleep.write_text("#!/bin/sh\nexit 0\n")
            sleep.chmod(0o755)
            config = repo / "controller with spaces.yaml"
            config_target = repo / "controller target.yaml"
            config_target.write_text("{}\n")
            config.symlink_to(config_target)
            out = repo / "run"
            env = dict(R.os.environ, PATH=str(stubs) + ":" + R.os.environ["PATH"],
                       TEST_LAUNCH_CAPTURE=str(capture))
            result = launch_subprocess(["bash", str(launcher), S.demo_name("single_obstacle", 2, 2),
                                     "object", "0.397", "-0.431", "0", "600", "19001", str(out),
                                     "--controller-params", str(config), "--goal-yaw-degrees", "0"],
                                    text=True, capture_output=True, env=env)
            self.assertEqual(result.returncode, 0, result.stderr)
            calls = [json.loads(line) for line in capture.read_text().splitlines()]
            native = {Path(call[0]).name: call for call in calls if Path(call[0]).name in S.BINARIES}
            self.assertEqual(set(native), set(S.BINARIES))
            for call in native.values():
                self.assertIn(f"--controller_params={config}", call)
                self.assertNotIn(f"--controller_params={config_target}", call)
            self.assertIn("--goal_yaw_degrees=0", native["franka_sampling_c3_controller"])
            self.assertNotIn("--goal_yaw_degrees=0", native["franka_sim"])
            for name in ("franka_sim", "franka_osc_controller"):
                self.assertIn("--execution_logging=true", native[name])


    def test_launcher_tees_recorder_progress_preserves_failure_and_cleans_process_groups(self):
        if not shutil.which("setsid"):
            self.skipTest("setsid is unavailable")
        progress = "[sugar_box] step=0010 sim=0.36s wall=1.2s pos_err=0.842m yaw_err=4.2deg within_goal=no"
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "stub checkout"
            launcher = repo / "c3plus/runtime/launcher.py"
            shutil.copytree(S.REPO / "c3plus", repo / "c3plus")
            binaries = repo / ".build/bin/examples/sampling_c3"
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
                result = launch_subprocess(command, capture_output=True, text=True, env=env, timeout=10)
                self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
                self.assertNotIn("Aborted", result.stderr)
                self.assertNotIn("Segmentation fault", result.stderr)
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


    def test_launcher_preserves_silent_native_signal_failure_diagnostics(self):
        if not shutil.which("setsid"):
            self.skipTest("setsid is unavailable")
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "stub checkout"
            shutil.copytree(S.REPO / "c3plus", repo / "c3plus")
            binaries = repo / ".build/bin/examples/sampling_c3"
            binaries.mkdir(parents=True)
            captures = repo / "processes"
            captures.mkdir()
            native_source = (f"#!{sys.executable}\n"
                "import os, resource, signal, sys\nfrom pathlib import Path\n"
                "resource.setrlimit(resource.RLIMIT_CORE, (0, 0))\n"
                "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
                "name = Path(sys.argv[0]).name\n"
                "(Path(os.environ['TEST_PROCESS_CAPTURE']) / name).write_text(str(os.getpid()))\n"
                "death = {'franka_osc_controller': signal.SIGSEGV, 'franka_sampling_c3_controller': signal.SIGABRT}.get(name)\n"
                "if death is not None: os.kill(os.getpid(), death)\n"
                "signal.pause()\n")
            for name in S.BINARIES:
                binary = binaries / name
                binary.write_text(native_source)
                binary.chmod(0o755)
            recorder = repo / "recorder_python"
            recorder.write_text(f"#!{sys.executable}\n"
                "import os, sys, time\nfrom pathlib import Path\n"
                "directory = Path(os.environ['TEST_PROCESS_CAPTURE'])\n"
                "deadline = time.monotonic() + 5\n"
                "while len(list(directory.iterdir())) != 3:\n"
                "    if time.monotonic() > deadline: sys.exit(98)\n"
                "    time.sleep(.01)\n"
                "sys.exit(17)\n")
            recorder.chmod(0o755)
            out = repo / "output"
            env = dict(R.os.environ, PYTHON=str(recorder), TEST_PROCESS_CAPTURE=str(captures))
            try:
                result = launch_subprocess(
                    ["bash", str(repo / "c3plus/runtime/launcher.py"), "unused_demo", "object",
                     "0", "0", "0", "600", "19001", str(out)],
                    capture_output=True, text=True, env=env, timeout=10)
                self.assertEqual(result.returncode, 17, result.stdout + result.stderr)
                self.assertEqual(result.stderr.splitlines(), ["osc: Segmentation fault", "planner: Aborted"])
                for name in ("sim", "osc", "planner"):
                    self.assertEqual((out / f"{name}.log").read_text(), "")
                (out / "launcher.log").write_text(result.stdout + result.stderr)
                self.assertEqual(Processes.classify_failure(out),
                                 [{"process": "unknown", "reason": "see_launcher_log"}])
            finally:
                for path in captures.iterdir():
                    try:
                        R.os.killpg(int(path.read_text()), 9)
                    except ProcessLookupError:
                        pass


    def test_launcher_hangup_preserves_signal_status_and_cleans_all_process_groups(self):
        if not shutil.which("setsid"):
            self.skipTest("setsid is unavailable")
        for ignored in (False, True):
            with self.subTest(inherited_hangup_ignored=ignored):
                self._check_launcher_hangup(ignored)

    def _check_launcher_hangup(self, ignored):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "stub checkout"
            shutil.copytree(S.REPO / "c3plus", repo / "c3plus")
            binaries = repo / ".build/bin/examples/sampling_c3"
            binaries.mkdir(parents=True)
            captures = repo / "processes"
            captures.mkdir()
            source = (f"#!{sys.executable}\n"
                "import json, os, signal, sys\nfrom pathlib import Path\n"
                "path = Path(os.environ['TEST_PROCESS_CAPTURE']) / Path(sys.argv[0]).name\n"
                "def terminated(*_):\n"
                "    path.with_suffix('.terminated').write_text('SIGTERM')\n"
                "    sys.exit(0)\n"
                "signal.signal(signal.SIGTERM, terminated)\n"
                "path.with_suffix('.pid').write_text(json.dumps({'pid': os.getpid(), 'pgid': os.getpgrp()}))\n"
                "signal.pause()\n")
            recorder = repo / "recorder_python"
            for path in [*(binaries / name for name in S.BINARIES), recorder]:
                path.write_text(source)
                path.chmod(0o755)
            env = dict(R.os.environ, PYTHON=str(recorder), TEST_PROCESS_CAPTURE=str(captures))
            prefix = [sys.executable, "-m", "c3plus.runtime.launcher"]
            if ignored:
                prefix = [sys.executable, "-c",
                    "import runpy, signal; signal.signal(signal.SIGHUP, signal.SIG_IGN); "
                    "runpy.run_module('c3plus.runtime.launcher', run_name='__main__', alter_sys=True)"]
            launcher = subprocess.Popen(
                [*prefix, "unused_demo", "object",
                 "0", "0", "0", "600", "19001", str(repo / "output")],
                cwd=repo, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 8
                while len(list(captures.glob("*.pid"))) != 4:
                    self.assertIsNone(launcher.poll(), "Launcher exited before synthetic processes started")
                    self.assertLess(time.monotonic(), deadline, "Synthetic processes did not start")
                    time.sleep(.01)
                launcher.send_signal(signal.SIGHUP)
                if ignored:
                    time.sleep(.1)
                    self.assertIsNone(launcher.poll(), "Inherited ignored SIGHUP terminated launcher")
                    self.assertEqual(list(captures.glob("*.terminated")), [])
                    launcher.send_signal(signal.SIGTERM)
                stdout, stderr = launcher.communicate(timeout=4)
                self.assertEqual(launcher.returncode, 143 if ignored else -signal.SIGHUP, stdout + stderr)
                self.assertEqual(len(list(captures.glob("*.terminated"))), 4)
                for path in captures.glob("*.pid"):
                    process = json.loads(path.read_text())
                    stat = Path(f"/proc/{process['pid']}/stat")
                    if stat.exists():
                        self.assertEqual(stat.read_text().rsplit(")", 1)[1].split()[0], "Z",
                                         f"Stub process remained alive: {path.name}")
                    with self.assertRaises(ProcessLookupError):
                        R.os.killpg(process["pgid"], 0)
            finally:
                if launcher.poll() is None:
                    launcher.kill()
                    launcher.wait(timeout=3)
                for path in captures.glob("*.pid"):
                    try:
                        R.os.killpg(json.loads(path.read_text())["pgid"], 9)
                    except ProcessLookupError:
                        pass
