from c3plus.evaluation import postprocess as Postprocess
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from c3plus.evaluation import exporter as P, package as A
from c3plus.configs.paths import REPO

from tests.fixtures.results import ArtifactFixtures, RUN_ID

class RunArtifactTests(ArtifactFixtures, unittest.TestCase):
    def test_offline_entrypoints_parse_help_and_reject_missing_required_arguments(self):
        for entrypoint in (("c3plus.evaluation.postprocess",), ("c3plus.utils", "postprocess")):
            with self.subTest(entrypoint=entrypoint):
                command = [sys.executable, "-m", *entrypoint]
                help_result = subprocess.run([*command, "--help"], cwd=REPO, capture_output=True,
                                             text=True, timeout=15)
                self.assertEqual(help_result.returncode, 0, help_result.stderr)
                for option in ("--run-dir", "--scene", "--run-id", "--scene-config", "--export-only"):
                    self.assertIn(option, help_result.stdout)
                invalid = subprocess.run(command, cwd=REPO, capture_output=True, text=True, timeout=15)
                self.assertEqual(invalid.returncode, 2, invalid.stdout + invalid.stderr)
                self.assertIn("required", invalid.stderr)

    def test_direct_and_cli_postprocess_export_identical_saved_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "saved run with spaces"
            self.make_run(root)
            payload = self.compact(root)
            for key in ("evaluation", "native_controller", "n_snapshots", "n_recorded_intervals"):
                payload.pop(key, None)
            payload["schema"].pop("legacy_fields", None)
            result_file = root / f"{RUN_ID}_result.json"
            P.write_result_json(result_file, payload)
            original = result_file.read_bytes()
            retained = {path.name: (path.read_bytes(), path.stat().st_mtime_ns)
                        for path in root.iterdir() if path != result_file}
            exports = []
            for entrypoint in (("c3plus.evaluation.postprocess",), ("c3plus.utils", "postprocess")):
                with self.subTest(entrypoint=entrypoint):
                    result_file.write_bytes(original)
                    result = subprocess.run(
                        [sys.executable, "-m", *entrypoint, "--run-dir", str(root), "--scene", "open_task",
                         "--run-id", RUN_ID, "--export-only"], cwd=REPO, capture_output=True,
                        text=True, timeout=15)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn("WROTE", result.stdout)
                    exports.append(result_file.read_bytes())
                    self.assertIn("evaluation", json.loads(exports[-1]))
                    self.assertEqual({path.name: (path.read_bytes(), path.stat().st_mtime_ns)
                                      for path in root.iterdir() if path != result_file}, retained)
            self.assertEqual(exports, [exports[0]] * 2)

    def test_compacted_export_only_adds_semantics_without_changing_recordings_or_video(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root)
            result = self.compact(root)
            # Model an already compacted result from before the semantic additions.
            for key in ("evaluation", "native_controller", "n_snapshots", "n_recorded_intervals"):
                result.pop(key, None)
            result["schema"].pop("legacy_fields", None)
            result_file = root / f"{RUN_ID}_result.json"
            P.write_result_json(result_file, result)
            before = deepcopy(result)
            video = root / f"{RUN_ID}.mp4"
            video_before = (video.read_bytes(), video.stat().st_mtime_ns)
            argv = ["postprocess", "--run-dir", str(root), "--scene", "open_task",
                    "--run-id", RUN_ID, "--export-only"]
            with patch("sys.argv", argv), redirect_stdout(io.StringIO()), \
                    patch.object(Postprocess, "build_fk", side_effect=AssertionError("FK forbidden")) as fk, \
                    patch("subprocess.run", side_effect=AssertionError("External process forbidden")), \
                    patch("subprocess.check_output", side_effect=AssertionError("Current runtime lookup forbidden")):
                Postprocess.main()
            fk.assert_not_called()
            updated = json.loads(result_file.read_text())
            self.assertEqual(set(path.name for path in root.iterdir()), {result_file.name, video.name})
            self.assertEqual((video.read_bytes(), video.stat().st_mtime_ns), video_before)
            self.assertEqual(updated["package"], before["package"])
            self.assertEqual(updated["recording"], before["recording"])
            self.assertEqual(updated["dynamic"], before["dynamic"])
            self.assertEqual(updated["runtime_status"], before["runtime_status"])
            self.assertEqual(updated["provenance"]["c3plus"], before["provenance"]["c3plus"])
            self.assertEqual(updated["provenance"]["configuration"], before["provenance"]["configuration"])
            for key in original["result"]:
                if key in ("schema", "provenance", "evaluation", "native_controller", "n_snapshots", "n_recorded_intervals"):
                    continue
                self.assertEqual(updated[key], before[key], key)
            self.assertEqual(updated["n_snapshots"], 3)
            self.assertEqual(updated["n_recorded_intervals"], 2)
            self.assertFalse(updated["evaluation"]["ever_success"])
            self.assertFalse(updated["evaluation"]["final_success"])
            self.assertIsNone(updated["evaluation"]["first_success_t"])
            self.assertEqual(updated["evaluation"]["thresholds"], {"position_m": .05, "orientation_rad": .1})
            self.assertEqual(updated["native_controller"]["success_thresholds"],
                             {"position_m": .02, "orientation_rad": .1})
            self.assertIsNone(updated["native_controller"]["success"])
            self.assertTrue(A.completion(root, RUN_ID))


    def test_legacy_compact_schema_upgrade_preserves_recordings_and_exact_configuration(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            self.make_run(root)
            result = self.compact(root)
            result["schema"]["semantics_version"] = 2
            result["dynamic"]["compute_time"] = [None] * result["steps_run"]
            hp = result["hyperparameters"]
            hp["control_dt"] = None
            hp.pop("control_dt_source")
            hp["costs"] = result["evaluation"]["weights"]
            hp["c3plus"] = result["provenance"].pop("c3plus")
            hp["c3plus"]["configurations"] = {
                key: entry["data"] for key, entry in result["provenance"]["configuration"]["files"].items()
                if key.startswith("config/") and key.endswith(".yaml")}
            before = deepcopy(result)
            result_file = root / f"{RUN_ID}_result.json"
            P.write_result_json(result_file, result)
            video_before = (root / f"{RUN_ID}.mp4").read_bytes()
            argv = ["postprocess", "--run-dir", str(root), "--scene", "open_task",
                    "--run-id", RUN_ID, "--export-only"]
            for attempt in range(2):
                with patch("sys.argv", argv), redirect_stdout(io.StringIO()), \
                        patch("subprocess.run", side_effect=AssertionError("External process forbidden")):
                    Postprocess.main()
                updated = json.loads(result_file.read_text())
                for key in ("recording", "runtime_status", "package", "static", "run"):
                    self.assertEqual(updated[key], before[key], key)
                self.assertEqual(updated["provenance"]["configuration"], before["provenance"]["configuration"])
                self.assertEqual(updated["evaluation"], before["evaluation"])
                self.assertEqual(updated["dynamic"], {key: value for key, value in before["dynamic"].items()
                                                      if key != "compute_time"})
                self.assertNotIn("c3plus", updated["hyperparameters"])
                self.assertNotIn("costs", updated["hyperparameters"])
                self.assertNotIn("configurations", updated["provenance"]["c3plus"])
                times = updated["dynamic"]["time"]
                self.assertAlmostEqual(updated["steps_run"] * updated["hyperparameters"]["control_dt"],
                                       times[-1] - times[0])
                self.assertEqual((root / f"{RUN_ID}.mp4").read_bytes(), video_before)
                if attempt == 0:
                    once = result_file.read_bytes()
                else:
                    self.assertEqual(result_file.read_bytes(), once)
