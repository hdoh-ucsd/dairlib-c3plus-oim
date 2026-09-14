from contextlib import redirect_stderr, redirect_stdout
import io
import shlex
import unittest
from unittest.mock import patch

from c3plus import configs as S
from tools import __main__ as cli

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_build_cli_dry_run_and_jobs_use_all_native_targets(self):
        output = io.StringIO()
        with patch.object(cli.subprocess, "call") as call, redirect_stdout(output):
            self.assertEqual(cli.main(["build", "--jobs", "4", "--dry-run"]), 0)
        call.assert_not_called()
        self.assertEqual(shlex.split(output.getvalue()), ["bazel", "build",
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
