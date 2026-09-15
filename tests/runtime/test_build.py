from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from c3plus import configs as S
from c3plus.utils import __main__ as cli

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_build_cli_dry_run_and_jobs_use_all_native_targets(self):
        output = io.StringIO()
        with patch.object(cli.subprocess, "call") as call, redirect_stdout(output):
            self.assertEqual(cli.main(["build", "--jobs", "4", "--dry-run"]), 0)
        call.assert_not_called()
        self.assertEqual(shlex.split(output.getvalue()), [str(S.REPO / "build_support/bazel"), "build",
                         "//examples/sampling_c3:franka_sim",
                         "//examples/sampling_c3:franka_osc_controller",
                         "//examples/sampling_c3:franka_sampling_c3_controller", "--jobs=4"])
        with patch.object(cli.subprocess, "call", return_value=7) as call:
            self.assertEqual(cli.main(["build", "--jobs", "2"]), 7)
        self.assertEqual(call.call_args.kwargs["cwd"], S.REPO)
        self.assertEqual(call.call_args.args[0][-1], "--jobs=2")


    def test_build_cli_rejects_invalid_jobs_before_launch(self):
        for value in ("0", "-1"):
            with self.subTest(value=value), patch.object(cli.subprocess, "call") as call, \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                cli.main(["build", "--jobs", value])
            self.assertEqual(error.exception.code, 2)
            call.assert_not_called()


    def test_build_cli_missing_bazel_points_to_canonical_docker_workflow(self):
        output = io.StringIO()
        with patch.object(cli.subprocess, "call", side_effect=FileNotFoundError), \
                redirect_stderr(output), self.assertRaises(SystemExit) as error:
            cli.main(["build"])
        self.assertEqual(error.exception.code, 1)
        self.assertIn("./docker/shell.sh", output.getvalue())
        self.assertIn("README.md", output.getvalue())
        self.assertNotIn("docker/README.md", output.getvalue())


class BazelWrapperTests(unittest.TestCase):
    def wrapper_fixture(self, directory):
        workspace = Path(directory) / "checkout with spaces"
        support = workspace / "build_support"
        support.mkdir(parents=True)
        (workspace / "MODULE.bazel").write_text("module(name = 'test')\n")
        wrapper = support / "bazel"
        shutil.copy2(S.REPO / "build_support/bazel", wrapper)
        shutil.copy2(S.REPO / "build_support/noros.bazelrc", support / "noros.bazelrc")
        child = workspace / "nested"
        child.mkdir()
        binary = Path(directory) / "bin"
        binary.mkdir()
        fake_bazel = binary / "bazel"
        fake_bazel.write_text(
            f"#!{sys.executable}\n"
            "import json, os, sys\n"
            "print(json.dumps({'args':sys.argv[1:],'cwd':os.getcwd(),"
            "'skip_wrapper':os.environ.get('BAZELISK_SKIP_WRAPPER'),"
            "'cache':os.environ.get('BAZELISK_HOME')}))\n"
            "sys.exit(int(os.environ.get('FAKE_BAZEL_RC','0')))\n")
        fake_bazel.chmod(0o755)
        environment = dict(os.environ, PATH=f"{binary}:/usr/bin:/bin",
                           BAZELISK_HOME=str(Path(directory) / "cache with spaces"))
        environment.pop("BAZEL_REAL", None)
        environment.pop("DAIRLIB_WITH_ROS", None)
        return workspace, wrapper, child, fake_bazel, environment

    def test_explicit_wrapper_preserves_flags_resources_cache_and_ros_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace, wrapper, child, fake, environment = self.wrapper_fixture(tmp)
            arguments = ["--output_user_root=/tmp/cache path", "build", "//target:example", "--jobs=4"]
            for direct_real in (False, True):
                for ros in (None, "ON", "OFF"):
                    with self.subTest(direct_real=direct_real, ros=ros):
                        env = dict(environment, FAKE_BAZEL_RC="7")
                        if direct_real:
                            env["BAZEL_REAL"] = str(fake)
                        if ros is not None:
                            env["DAIRLIB_WITH_ROS"] = ros
                        result = subprocess.run([str(wrapper), *arguments], cwd=child, env=env,
                                                capture_output=True, text=True, timeout=10)
                        self.assertEqual(result.returncode, 7, result.stderr)
                        recorded = json.loads(result.stdout)
                        prefix = [] if ros == "ON" else [f"--bazelrc={workspace}/build_support/noros.bazelrc"]
                        self.assertEqual(recorded["args"], prefix + arguments)
                        self.assertEqual(recorded["cwd"], str(child))
                        self.assertEqual(recorded["skip_wrapper"], "true")
                        self.assertEqual(recorded["cache"], environment["BAZELISK_HOME"])

    def test_missing_bazel_reports_docker_workflow(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, wrapper, child, _, environment = self.wrapper_fixture(tmp)
            environment["BAZEL_REAL"] = str(Path(tmp) / "missing bazel")
            result = subprocess.run([str(wrapper), "build", "//target:example"],
                                    cwd=child, env=environment, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 127)
            self.assertIn("Bazel is missing", result.stderr)
            self.assertIn("./docker/shell.sh", result.stderr)
