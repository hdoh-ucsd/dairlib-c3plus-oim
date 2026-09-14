import subprocess
import sys
import unittest
from unittest.mock import patch

from tools import __main__ as cli
from c3plus.experiments import run as R

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_cli_help_without_scientific_dependencies(self):
        result = subprocess.run([sys.executable, "-S", "-m", "tools", "--help"],
                                cwd=R.REPO, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        for command in ("build", "scenes", "check", "run", "campaign", "postprocess", "render", "visualize"):
            self.assertIn(command, result.stdout)


    def test_cli_forwards_leaf_exit_code(self):
        with patch.object(cli.subprocess, "call", return_value=17) as call:
            self.assertEqual(cli.main(["run", "--scene", "open_task", "--out", "a path"]), 17)
        self.assertEqual(call.call_args.args[0][-2:], ["--out", "a path"])
