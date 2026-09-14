from c3plus.evaluation import postprocess as Postprocess
from contextlib import redirect_stdout
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import yaml

from c3plus.evaluation import exporter as P

from tests.fixtures.results import ObjectFixtures, ProjectionFixtures

class ObjectRunTests(ObjectFixtures, unittest.TestCase):
    def test_evaluation_records_selected_object_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            identity = {"object_name": "banana", "simulation_model": "banana.sdf",
                        "controller_model": "banana_controller.sdf", "object_body_name": "banana_base",
                        "object_channel_substring": "banana_base"}
            cfg = {**identity, "goal": [.4, -.4, 0], "footprint": [[-.02, -.02], [.02, -.02],
                                                                       [.02, .02], [-.02, .02]],
                   "pusher_radius": .00555, "block_half_height": .015, "tip_target_z": -.014,
                   "tip_floor_z_real": -.02345,
                   "obstacles": {}}
            config = out / "evaluation_scene_config.yaml"
            config.write_text(yaml.safe_dump(cfg))
            steps = [{"control_step": k, "sim_time": .1 * k, "robot_q": [0] * 5,
                      "objects": {"OBJECT_banana_base_STATE_SIMULATION": [1, 0, 0, 0, .4, -.4, -.029]}}
                     for k in (1, 2)]
            (out / "steps_raw.jsonl").write_text("".join(json.dumps(step) + "\n" for step in steps))
            (out / "recorder.log").write_text("FINAL {}\n")
            (out / "runtime_status.json").write_text(json.dumps({
                "run_id": "object_test", "scene": "open_task", "commit": "saved-launch-commit"}))
            fk = lambda q: (np.array([.37, -.4, -.014]), np.zeros(3), np.diag([1, -1, -1]))
            argv = ["postprocess", "--run-dir", str(out), "--scene", "open_task",
                    "--run-id", "object_test", "--scene-config", str(config)]
            with patch("sys.argv", argv), patch.object(Postprocess, "build_fk", return_value=fk), \
                    patch("subprocess.check_output", side_effect=AssertionError("Current checkout lookup forbidden")), \
                    redirect_stdout(io.StringIO()):
                Postprocess.main()
            result = json.loads((out / "object_test_result.json").read_text())
            manifest = yaml.safe_load((out / "object_test_manifest.yaml").read_text())
            self.assertEqual(manifest["git_commit"], "saved-launch-commit")
            for key, value in identity.items():
                self.assertEqual(result[key], value)
                self.assertEqual(manifest[key], value)
            self.assertEqual(result["n_control_steps"], 2)
            with (out / "object_test_metrics.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(float(rows[0]["tip_z_cost"]), 0.0)


class ResultProjectionTests(ProjectionFixtures, unittest.TestCase):
    def test_json_writer_sanitizes_missing_values_and_preserves_existing_on_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            result_file = Path(tmp) / "result.json"
            payload = {"missing": [float("nan"), float("inf"), -float("inf")],
                       "array": np.array([1.0, np.nan]), "numpy_scalar": np.float64(np.inf),
                       "nested": {"value": np.int64(7), "boolean": np.bool_(True)}}
            P.write_result_json(result_file, payload)
            def reject_constant(value):
                raise AssertionError(f"Nonstandard JSON constant: {value}")
            stored = json.loads(result_file.read_text(), parse_constant=reject_constant)
            self.assertEqual(stored, {"missing": [None] * 3, "array": [1.0, None],
                                      "numpy_scalar": None, "nested": {"value": 7, "boolean": True}})
            before = result_file.read_bytes()
            with patch.object(P.os, "replace", side_effect=OSError("synthetic replace failure")), \
                    self.assertRaises(OSError):
                P.write_result_json(result_file, {"replace": "should not be installed"})
            self.assertEqual(result_file.read_bytes(), before)
            self.assertEqual([p.name for p in Path(tmp).iterdir()], ["result.json"])


    def test_export_only_updates_result_json_without_fk_or_other_writes(self):
        cfg, summary, steps, rows = self.fixture()
        run_id = summary["run_id"]
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            result_file = out / f"{run_id}_result.json"
            result_file.write_text(json.dumps(summary))
            (out / "steps_raw.jsonl").write_text("".join(json.dumps(step) + "\n" for step in steps))
            config = out / "evaluation_scene_config.yaml"
            config.write_text(yaml.safe_dump(cfg))
            (out / f"{run_id}_manifest.yaml").write_text(yaml.safe_dump({
                "run_id": run_id, "scenario": "open_task", "goal": cfg["goal"],
                "tolerances": {"pos_tol": .0123, "ang_tol": .0456}}))
            with (out / f"{run_id}_metrics.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            (out / f"{run_id}_cost_diagnostics.png").write_bytes(b"existing plot must be untouched")
            (out / "video.mp4").write_bytes(b"existing video must be untouched")
            (out / "recorder.log").write_text('FINAL {"pos_tol": 99, "ang_tol": 99}\n')
            before = {path.relative_to(out): (path.read_bytes(), path.stat().st_mtime_ns)
                      for path in out.rglob("*") if path.is_file() and path != result_file}
            argv = ["postprocess", "--run-dir", str(out), "--scene", "open_task",
                    "--run-id", run_id, "--scene-config", str(config), "--export-only"]
            with patch("sys.argv", argv), patch.object(Postprocess, "build_fk", side_effect=AssertionError("FK forbidden")) as fk, \
                    patch("matplotlib.pyplot.subplots", side_effect=AssertionError("Plotting forbidden")), \
                    redirect_stdout(io.StringIO()):
                Postprocess.main()
            fk.assert_not_called()
            after = {path.relative_to(out): (path.read_bytes(), path.stat().st_mtime_ns)
                     for path in out.rglob("*") if path.is_file() and path != result_file}
            self.assertEqual(after, before)
            result = json.loads(result_file.read_text())
            self.assertEqual(result["legacy_annotation"], summary["legacy_annotation"])
            self.assertEqual(result["n_control_steps"], 5)
            self.assertEqual(result["steps_run"], 4)
            self.assertEqual(result["hyperparameters"]["goal_pos_tol"], .0123)
            self.assertEqual(result["hyperparameters"]["goal_theta_tol"], .0456)
            self.assertEqual(len(result["dynamic"]["object_pose"]), 5)
