import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import yaml

from c3plus.evaluation import package as A

from tests.fixtures.results import ArtifactFixtures, RUN_ID, VIDEO_PROBE, file_snapshot

class RunArtifactTests(ArtifactFixtures, unittest.TestCase):
    def test_complete_unsuccessful_run_retains_recordings_and_metadata_in_two_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root, success=False)
            result = self.compact(root)
            self.assertEqual(set(path.name for path in root.iterdir()),
                             {f"{RUN_ID}_result.json", f"{RUN_ID}.mp4"})
            self.assertEqual(result, json.loads((root / f"{RUN_ID}_result.json").read_text()))
            self.assertFalse(result["success"])
            self.assertEqual(result["package"]["status"], "complete")
            self.assertTrue(result["package"]["cleanup_complete"])
            for key in ("dynamic", "schema", "static", "hyperparameters", "run", "legacy_annotation",
                        "n_control_steps", "steps_run", "final_position_error", "final_orientation_error"):
                self.assertEqual(result[key], original["result"][key], key)
            self.assertEqual(result["recording"]["steps_raw"], original["steps"])
            self.assertEqual(result["recording"]["state_trace"], original["trace"])
            csv_bytes = original["before"][f"{RUN_ID}_metrics.csv"][1]
            self.assertEqual(result["recording"]["metrics_csv"], csv_bytes.decode())
            self.assertIsNone(result["dynamic"]["min_obstacle_clearance"][0])
            self.assertEqual(result["runtime_status"], original["status"])
            files = result["provenance"]["configuration"]["files"]
            for name in ("config/controller.yaml", "config/sampling.yaml", "config/simulation.yaml",
                         "config/goal.yaml", "evaluation_scene_config.yaml", f"{RUN_ID}_manifest.yaml"):
                raw = original["before"][name][1]
                self.assertEqual(files[name]["text"], raw.decode(), name)
                self.assertEqual(files[name]["data"], yaml.safe_load(raw), name)
                self.assertEqual(files[name]["sha256"], hashlib.sha256(raw).hexdigest(), name)
            self.assertEqual(result["diagnostics"]["recorder_final"], original["recorder_final"])
            log = result["diagnostics"]["logs"]["planner.log"]
            self.assertEqual(log["commands"], ["[COMMAND] fixture planner", "[CWD] /recorded/checkout"])
            self.assertTrue(any("early diagnostic" in row["text"] for row in log["warnings_and_errors"]))
            self.assertEqual(log["tail"][-1], "ordinary progress 74")
            self.assertEqual(log["sha256"], hashlib.sha256(original["before"]["planner.log"][1]).hexdigest())
            self.assertEqual((root / f"{RUN_ID}.mp4").read_bytes(), original["video"])
            self.assertEqual(result["package"]["video"]["sha256"], hashlib.sha256(original["video"]).hexdigest())
            self.assertEqual(result["package"]["video"]["probe"], VIDEO_PROBE)
            self.assertTrue(A.completion(root, RUN_ID))
            self.assertEqual(A.load_status(root, RUN_ID), original["status"])


    def test_skipped_rendering_packages_one_json_and_is_recognized_complete(self):
        """--no-video keeps every recording; only the MP4 and its checks are dropped."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root, success=False)
            (root / f"{RUN_ID}.mp4").unlink()
            result = self.compact(root, video_required=False)
            self.assertEqual(set(path.name for path in root.iterdir()),
                             {f"{RUN_ID}_result.json"})
            self.assertIsNone(result["package"]["video"])
            self.assertTrue(result["package"]["video_skipped"])
            self.assertEqual(result["package"]["status"], "complete")
            self.assertTrue(result["package"]["cleanup_complete"])
            # The recordings a skipped render must never cost us.
            self.assertEqual(result["recording"]["steps_raw"], original["steps"])
            self.assertEqual(result["recording"]["state_trace"], original["trace"])
            self.assertTrue(A.completion(root, RUN_ID))

    def test_missing_video_is_still_incomplete_when_rendering_was_expected(self):
        """A video-less bundle must not pass as complete unless it says so."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            self.make_run(root, success=False)
            self.compact(root)
            (root / f"{RUN_ID}.mp4").unlink()
            self.assertFalse(A.completion(root, RUN_ID))

    def test_skipped_rendering_refuses_a_present_video(self):
        """Disagreement between the flag and the artifacts must not be silent."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            self.make_run(root, success=False)
            self.assert_rejected_without_changes(root, video_required=False)


    def test_finished_inline_packaging_does_not_require_a_legacy_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root)
            (root / "RUN_COMPLETE").unlink()
            (root / "runtime_status.json").unlink()
            result = self.compact(root, status=original["status"], require_legacy_complete=False)
            self.assertTrue(A.completion(root, RUN_ID))
            self.assertEqual(result["runtime_status"], original["status"])


    def test_completion_and_status_support_legacy_and_compact_layouts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root)
            self.assertTrue(A.completion(root, RUN_ID))
            self.assertEqual(A.load_status(root, RUN_ID), original["status"])
            (root / "RUN_COMPLETE").unlink()
            self.assertFalse(A.completion(root, RUN_ID))
            (root / "RUN_COMPLETE").touch()
            result = self.compact(root)
            self.assertTrue(A.completion(root))
            self.assertEqual(A.load_status(root), original["status"])
            before = file_snapshot(root)
            with patch.object(A.subprocess, "run") as ffprobe:
                self.assertEqual(A.compact_run(root), result)
                ffprobe.assert_not_called()
            self.assertEqual(file_snapshot(root), before)
            video = root / f"{RUN_ID}.mp4"
            video.write_bytes(video.read_bytes() + b"changed after completion")
            self.assertFalse(A.completion(root, RUN_ID))


    def test_mesh_snapshot_must_match_the_referenced_repository_asset(self):
        source = A.REPO / "examples/sampling_c3/urdf/c_glyph_base/c_glyph_base.obj"
        original_asset = source.read_bytes()
        for match in (True, False):
            with self.subTest(match=match), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "run"
                self.make_run(root)
                relative = Path("config/repository") / source.relative_to(A.REPO)
                snapshot = root / relative
                snapshot.parent.mkdir(parents=True)
                snapshot.write_bytes(original_asset + (b"" if match else b"\n# Changed asset\n"))
                if match:
                    result = self.compact(root)
                    asset = result["provenance"]["configuration"]["repository_assets"][str(relative)]
                    self.assertEqual(asset["source_path"], str(source.relative_to(A.REPO)))
                    self.assertEqual(asset["sha256"], hashlib.sha256(original_asset).hexdigest())
                    self.assertFalse(snapshot.exists())
                else:
                    self.assert_rejected_without_changes(root)
                self.assertEqual(source.read_bytes(), original_asset)


    def test_partial_cleanup_preserves_every_recording_and_can_be_retried(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root)
            self.interrupt_cleanup(root)
            saved = json.loads((root / f"{RUN_ID}_result.json").read_text())
            self.assertFalse(saved["package"]["cleanup_complete"])
            self.assertEqual(saved["recording"]["steps_raw"], original["steps"])
            self.assertEqual(saved["recording"]["state_trace"], original["trace"])
            self.assertFalse((root / "state_trace.jsonl").exists())
            self.assertFalse((root / "evaluation_scene_config.yaml").exists())
            self.assertFalse((root / "runtime_status.json").exists())
            self.assertFalse(A.completion(root, RUN_ID))
            self.assertEqual(A.load_status(root, RUN_ID), original["status"])
            retried = self.compact(root)
            self.assertEqual(retried["recording"], saved["recording"])
            self.assertTrue(A.completion(root, RUN_ID))
            self.assertEqual(set(path.name for path in root.iterdir()),
                             {f"{RUN_ID}_result.json", f"{RUN_ID}.mp4"})


    def test_retry_refuses_a_remaining_artifact_changed_after_partial_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            self.make_run(root)
            self.interrupt_cleanup(root)
            path = root / "steps_raw.jsonl"
            path.write_text(path.read_text() + '{"new": "unarchived sample"}\n')
            self.assert_rejected_without_changes(root)


    def test_bad_or_missing_video_preserves_raw_data_and_logs(self):
        for probe in ({"streams": [], "format": {"duration": "1"}},
                      {"streams": [{"codec_type": "audio"}], "format": {"duration": "1"}},
                      {**VIDEO_PROBE, "format": {"duration": "0"}}):
            with self.subTest(probe=probe), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "run"
                self.make_run(root)
                before = file_snapshot(root)
                response = subprocess.CompletedProcess(["ffprobe"], 0, json.dumps(probe), "")
                with patch.object(A.subprocess, "run", return_value=response), \
                        self.assertRaises((ValueError, RuntimeError)):
                    A.compact_run(root, RUN_ID)
                self.assertEqual(file_snapshot(root), before)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            self.make_run(root)
            (root / f"{RUN_ID}.mp4").unlink()
            self.assert_rejected_without_changes(root)
