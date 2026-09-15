"""Discovery, selective reads, reporting and source preservation for evaluation."""

from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from c3plus.evaluation import run_eval
from c3plus.evaluation.metrics import trial_metrics
from c3plus.evaluation.serialization import _json
from tests.evaluation.test_metrics import run_fixture


class RunEvalTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "runs"
        self.root.mkdir()

    def write_run(self, name="one", object_name="T_shape", task="open_table"):
        run = run_fixture()
        run["run"].update(run_id=name, object=object_name, task=task)
        run["hyperparameters"]["object"] = object_name
        path = self.root / object_name / f"{name}_result.json"
        path.parent.mkdir(exist_ok=True)
        path.write_text(json.dumps(run))
        return path, run

    def test_projection_retains_metrics_and_never_decodes_raw_payloads(self):
        path, run = self.write_run()
        raw = {"large": [[i, i, i] for i in range(10000)],
               "escaped": 'braces { [ and quote " and Unicode \u2764 and slash \\'}
        for section in ("source_state", "provenance", "configuration"):
            run[section] = raw
        run["dynamic"]["raw_recorder"] = raw
        run["hyperparameters"]["provenance"] = raw
        path.write_text(json.dumps(run))
        with patch.object(run_eval, "_json", wraps=_json) as decoder:
            selected = run_eval.read_metric_fields(path)
        self.assertEqual(trial_metrics(selected), trial_metrics(run))
        self.assertNotIn("source_state", selected)
        self.assertNotIn("raw_recorder", selected["dynamic"])
        self.assertNotIn("provenance", selected["hyperparameters"])
        self.assertLess(max(len(call.args[0]) for call in decoder.call_args_list), 1000)

    def test_selected_fields_and_enclosing_json_are_strict(self):
        path = self.root / "bad_result.json"
        invalid = ('{"run":{},"run":{}}', '{"run":{"task":NaN}}',
                   '{"run":{"task":"bad\\x"}}', '{"run":{] }',
                   '{"run":{"task":"unclosed}}', '{"run":{}} trailing',
                   '{"run":{"task":true,}}', '{"run":{"task":[1,]}}',
                   '{"provenance":"bad\\uqqqq"}', '{"schema":\x0b{}}', '')
        for contents in invalid:
            path.write_text(contents)
            with self.subTest(contents=contents), self.assertRaises(ValueError):
                run_eval.read_metric_fields(path)

    def test_only_valid_results_count_and_nested_output_is_not_rediscovered(self):
        first, _ = self.write_run()
        self.write_run("two", "banana")
        for name in ("manifest.json", "runtime_status.json", "configuration.json", "provenance.json"):
            (self.root / name).write_text("not a trial or even JSON")
        (self.root / "summary_result.json").write_text('{"schema":"c3plus-aggregate-evaluation-v1"}')
        (self.root / "foreign_result.json").write_text('{"schema":{"version":"foreign"}}')
        (self.root / "alias_result.json").symlink_to(first)
        out = self.root / "eval"
        out.mkdir()
        (out / "old_result.json").write_text(first.read_text())
        summary = run_eval.evaluate(self.root, out)
        self.assertEqual(summary["n_runs"], 2)
        self.assertEqual(summary["n_task_groups"], 1)
        self.assertEqual(summary["averaged_over"], {"object": ["T_shape", "banana"]})
        self.assertEqual(len(summary["results"]), 1)
        self.assertEqual(summary["results"][0]["method"], "c3plus")

    def test_output_ancestor_does_not_exclude_the_runs_directory(self):
        self.write_run()
        self.assertEqual(run_eval.evaluate(self.root, self.root.parent)["n_runs"], 1)

    def test_duplicate_identity_and_invalid_modern_result_fail_instead_of_biasing_mean(self):
        path, run = self.write_run()
        duplicate = self.root / "copy_result.json"
        duplicate.write_text(path.read_text())
        with self.assertRaisesRegex(ValueError, "Duplicate run_id"):
            run_eval.evaluate(self.root)
        duplicate.unlink()
        del run["execution"]["alignment"]
        path.write_text(json.dumps(run))
        with self.assertRaisesRegex(ValueError, "alignment"):
            run_eval.evaluate(self.root)

    def test_empty_and_missing_input_fail(self):
        for path in (self.root, self.root / "missing"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                run_eval.evaluate(path)

    def test_task_groups_and_varied_settings_exclude_counts_and_identifiers(self):
        self.write_run()
        path, run = self.write_run("two", "banana")
        run["run"]["seed"] = 43
        run["planning"]["n_updates"] = 101
        run["recording"]["n_snapshots"] = 202
        path.write_text(json.dumps(run))
        self.write_run("three", "hammer", "shelf_gap")
        summary = run_eval.evaluate(self.root)
        self.assertEqual(summary["n_task_groups"], 2)
        self.assertEqual(summary["results"][0]["n"], 2)
        self.assertEqual(summary["results"][1]["n"], 1)
        self.assertEqual(summary["results"][0]["averaged_over"],
                         {"object": ["T_shape", "banana"], "seed": [42, 43]})
        self.assertIn("setting seed varies", " ".join(summary["warnings"]))

    def test_unlimited_failures_have_explicit_partial_steps_mean(self):
        self.write_run()
        path, run = self.write_run("two", "banana")
        run["execution"]["step_budget"] = run["hyperparameters"]["steps"] = None
        run["dynamic"]["object_pose"][2] = [1, 0, 0]
        path.write_text(json.dumps(run))
        summary = run_eval.evaluate(self.root)
        self.assertEqual(summary["results"][0]["steps"], 2)
        self.assertEqual(summary["results"][0]["available"]["steps"], 1)
        self.assertIn("steps available for 1/2", " ".join(summary["warnings"]))
        self.assertNotIn("Warning:", run_eval.format_report(summary))
        self.assertIsNone(summary["trials"][1]["steps_to_goal"])

    def test_formats_round_columns_and_escape_names(self):
        self.write_run()
        row = run_eval.evaluate(self.root)["results"][0]
        text = run_eval.format_table([row])
        self.assertEqual(text.splitlines()[-1].split(),
                         ["open_table", "c3plus", "1", "1.00", "0.030", "0.030", "0.020", "0.020", "2", "4.29", "3.00"])
        row = deepcopy(row)
        row.update(eps_d_success=None, eps_o_success=None, steps=None, frequency_hz=None)
        self.assertEqual(run_eval.format_table([row]).splitlines()[-1].split().count("-"), 4)
        self.assertIn("| --- |", run_eval.format_table([row], "markdown"))
        self.assertIn(r"open\_table", run_eval.format_table([row], "latex"))
        self.assertIn(r"\begin{tabular}{llrrrrrrrrr}", run_eval.format_table([row], "latex"))

    def test_cli_print_only_and_saved_outputs_leave_sources_unchanged(self):
        path, _ = self.write_run()
        before = path.read_bytes(), path.stat().st_mtime_ns
        with redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(run_eval.main(["--runs-dir", str(self.root), "--diagnostics"]), 0)
        self.assertIn('"execution_steps_recorded": 3', stdout.getvalue())
        diagnostic = json.loads(stdout.getvalue().splitlines()[0])
        self.assertAlmostEqual(diagnostic["eps_o"], .02)
        self.assertAlmostEqual(diagnostic["trajectory_mean_orientation_error"], .16)
        self.assertEqual(list(self.root.iterdir()), [path.parent])
        out = self.root / "eval"
        for output_format in ("text", "markdown", "latex"):
            with redirect_stdout(io.StringIO()):
                run_eval.main(["--runs-dir", str(self.root), "--out-dir", str(out), "--format", output_format])
            saved = json.loads((out / "runs.json").read_text())
            self.assertEqual(saved["schema"], "c3plus-aggregate-evaluation-v2")
            self.assertEqual(saved["n_runs"], 1)
            self.assertEqual(saved["grouping"], ["task", "method"])
            row = saved["results"][0]
            self.assertAlmostEqual(row["eps_d_success"], .03)
            self.assertAlmostEqual(row["eps_o_success"], .02)
            self.assertAlmostEqual(row["trajectory_mean_position_error_success"], .115)
            self.assertAlmostEqual(row["trajectory_mean_orientation_error_success"], .16)
            self.assertEqual(row["theta"], row["trajectory_mean_orientation_error"])
            self.assertIn("eps_o_success", saved["metric_definitions"])
            self.assertIn("Legacy alias", saved["metric_definitions"]["theta"])
            self.assertNotIn("dynamic", saved["trials"][0])
            self.assertEqual(before, (path.read_bytes(), path.stat().st_mtime_ns))
        self.assertEqual({path.name for path in out.iterdir()}, {"runs.json", "runs.txt", "runs.md", "runs.tex"})
        self.assertEqual(run_eval.evaluate(self.root)["n_runs"], 1)

    def test_output_symlink_cannot_overwrite_source(self):
        path, _ = self.write_run()
        before = path.read_bytes()
        out = self.root / "eval"
        out.mkdir()
        (out / "runs.json").symlink_to(path)
        with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as error:
            run_eval.main(["--runs-dir", str(self.root), "--out-dir", str(out)])
        self.assertEqual(error.exception.code, 1)
        self.assertEqual(path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
