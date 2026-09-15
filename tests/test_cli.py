import subprocess
import sys
import unittest
from unittest.mock import patch

from c3plus.utils import __main__ as cli
from c3plus.utils import run as R

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_cli_help_without_scientific_dependencies(self):
        result = subprocess.run([sys.executable, "-S", "-m", "c3plus.utils", "--help"],
                                cwd=R.REPO, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        for command in ("build", "scenes", "check", "run", "campaign", "eval", "postprocess", "render", "visualize"):
            self.assertIn(command, result.stdout)


    def test_cli_forwards_arguments_and_leaf_exit_code(self):
        modules = {
            "check": "c3plus.runtime.environment",
            "run": "c3plus.utils.run",
            "campaign": "c3plus.utils.campaign",
            "run_launch": "c3plus.utils.campaign",
            "run_launch_simple_s2": "c3plus.utils.campaign",
            "eval": "c3plus.evaluation.postprocess",
            "postprocess": "c3plus.evaluation.postprocess",
            "compact": "c3plus.evaluation.package",
            "render": "c3plus.visualization.render",
            "visualize": "c3plus.visualization.objects",
            "cost-figure": "c3plus.visualization.costs",
        }
        for command, module in modules.items():
            with self.subTest(command=command), patch.object(cli.subprocess, "call", return_value=17) as call:
                arguments = ["--scene", "open_task", "--run-dir", "a path", "literal;$value", ""]
                self.assertEqual(cli.main([command, *arguments]), 17)
                forwarded = [command, *arguments] if command.startswith("run_launch") else arguments
                self.assertEqual(call.call_args.args[0], [sys.executable, "-m", module, *forwarded])

    def test_every_subcommand_help_is_callable_without_launching_work(self):
        for command in ("build", "scenes", *cli.COMMANDS):
            with self.subTest(command=command):
                result = subprocess.run(
                    [sys.executable, "-m", "c3plus.utils", command, "--help"],
                    cwd=R.REPO, text=True, capture_output=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("usage:", result.stdout)

    def test_cli_has_no_root_tools_wrapper_directory(self):
        self.assertFalse((R.REPO / "tools").exists())
