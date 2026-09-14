"""Exercise lossless run packaging on real temporary files, without dynamics."""

import csv
from contextlib import redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

if __package__:
    from . import postprocess_run as P
    from . import run_artifacts as A
    from . import run_experiment as R
else:
    import postprocess_run as P
    import run_artifacts as A
    import run_experiment as R


RUN_ID = "exponential_open_task_banana_s02g02_seed42"
VIDEO_PROBE = {
    "streams": [{"codec_type": "video", "codec_name": "h264", "width": 960,
                 "height": 720, "nb_frames": "3", "duration": "0.3"}],
    "format": {"duration": "0.3", "format_name": "mov,mp4,m4a,3gp,3g2,mj2"},
}


def file_snapshot(root):
    """Include link targets and empty directories without following links."""
    values = {}
    for path in sorted(root.rglob("*")):
        key = str(path.relative_to(root))
        if path.is_symlink():
            values[key] = ("symlink", os.readlink(path))
        elif path.is_dir():
            values[key] = ("directory", None)
        else:
            values[key] = ("file", path.read_bytes())
    return values


class RunArtifactTests(unittest.TestCase):
    def make_run(self, root, success=False):
        root.mkdir(parents=True, exist_ok=True)
        cfg = {"object_name": "banana", "object_body_name": "banana_base",
               "object_channel_substring": "banana_base", "goal": [.4, -.4, 0],
               "footprint": [[-.02, -.02], [.02, -.02], [.02, .02], [-.02, .02]],
               "pusher_radius": .00555, "block_half_height": .01837,
               "tip_target_z": -.01063, "tip_floor_z_real": -.02345, "obstacles": {}}
        steps, rows, trace = [], [], []
        for index, (time, x) in enumerate(zip((.1, .2, .3), (.30, .32, .34)), 1):
            pose = [1, 0, 0, 0, x, -.4, -.029]
            q = [index + j / 10 for j in range(5)]
            step = {"control_step": index, "sim_time": time, "robot_q": q,
                    "robot_v": [.1 * j for j in range(5)],
                    "robot_u": [index * 10 + j for j in range(5)],
                    "objects": {"OBJECT_banana_base_STATE_SIMULATION": pose},
                    "additional_recorded_field": {"sequence": index, "value": "preserve me"}}
            steps.append(step)
            trace.append({"t": time, "q": q, "obj": pose, "pos_err": .4 - x,
                          "ang_err": 0, "extra_trace_value": index})
            rows.append({"run_id": RUN_ID, "scenario": "open_task", "control_step": index,
                         "sim_time": time, "object_x": x, "object_y": -.4, "object_yaw": 0,
                         "goal_x": .4, "goal_y": -.4, "goal_yaw": 0,
                         "position_error_m": .4 - x, "orientation_error_rad": 0,
                         "tip_x": x - .03, "tip_y": -.4, "tip_z": -.01063,
                         "tip_roll": math.pi, "tip_pitch": 0, "tip_yaw": 0,
                         "physical_contact_active": 0, "pusher_object_gap": .02,
                         "min_obstacle_clearance": float("nan"), "evaluation_total": 3.5,
                         **{name: float("nan") if name == "admm_penalty" else .25
                            for name in P.BLOCKS}})
        for filename, records in (("steps_raw.jsonl", steps), ("state_trace.jsonl", trace)):
            (root / filename).write_text("".join(json.dumps(row) + "\n" for row in records))
        with (root / f"{RUN_ID}_metrics.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        (root / "evaluation_scene_config.yaml").write_text(yaml.safe_dump(cfg))
        config = root / "config"
        config.mkdir()
        (config / "controller.yaml").write_text(
            "# Preserve the actual run configuration, including comments.\n"
            "object_body_name: banana_base\n"
            "sampling_params_file: config/sampling.yaml\n")
        (config / "sampling.yaml").write_text("sample_random_seed: 42\nsampling_strategy: 8\n")
        (config / "simulation.yaml").write_text("dt: 0.001\n")
        (config / "goal.yaml").write_text("goal_position: [0.4, -0.4]\n"
                                         "position_success_threshold: 0.02\n"
                                         "orientation_success_threshold: 0.1\n")
        status = {"run_id": RUN_ID, "scene": "open_task", "object_name": "banana",
                  "wrapper_rc": 0, "postprocess_rc": 0, "render_rc": 0, "cost_fig_rc": 0,
                  "seed": 42, "seed_verified": True, "failures": [], "success": success,
                  "simulation_wall_seconds": 12.5,
                  "config_sha256": {str(path.relative_to(root)):
                                    hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in config.glob("*.yaml")},
                  "diagnostic_extension": {"native_timeout": False, "stop_reason": "wall_cap"}}
        (root / "runtime_status.json").write_text(json.dumps(status, indent=2) + "\n")
        costs = {"W_Z_TIP": 2.5, "diagnostic_only": True}
        manifest = {"run_id": RUN_ID, "scenario": "open_task", "goal": cfg["goal"],
                    "tolerances": {"pos_tol": .05, "ang_tol": .1},
                    "evaluation": {"costs": costs}, "git_commit": "fixture-commit"}
        (root / f"{RUN_ID}_manifest.yaml").write_text(yaml.safe_dump(manifest))
        summary = {"run_id": RUN_ID, "scenario": "open_task", "success": success,
                   "n_control_steps": len(steps), "sim_time_end": .3,
                   "first_success_t": None, "final_position_error": .06,
                   "final_orientation_error": 0, "legacy_annotation": {"keep": "unchanged"}}
        result = P.project_result(root, "open_task", RUN_ID, cfg, summary, steps, rows,
                                  evaluation_costs=costs)
        result_path = root / f"{RUN_ID}_result.json"
        P.write_result_json(result_path, result)
        recorder_final = {"steps": 3, "success": success, "custom_counter": 17}
        for phase in ("launcher", "planner", "sim", "osc", "postprocess", "render", "cost_fig"):
            (root / f"{phase}.log").write_text(f"[COMMAND] fixture {phase}\n{phase}: λ diagnostic\n")
        (root / "planner.log").write_text(
            "[COMMAND] fixture planner\n[CWD] /recorded/checkout\n"
            "WARNING preserve an early diagnostic\n"
            + "".join(f"ordinary progress {index}\n" for index in range(75)))
        (root / "recorder.log").write_text("Recorder ready\nFINAL " + json.dumps(recorder_final) + "\n")
        scratch = root / "tmp/matplotlib"
        scratch.mkdir(parents=True)
        (scratch / "fontlist.json").write_text('{"generated_cache": true}\n')
        (root / f"{RUN_ID}_cost_diagnostics.png").write_bytes(b"derived diagnostic figure")
        # ffprobe is the one external boundary mocked below; no encoder is run.
        video = b"\x00\x00\x00\x18ftypmp42\x00\x00\x00\x00mp42isom" + b"fixture video payload"
        (root / f"{RUN_ID}.mp4").write_bytes(video)
        (root / "RUN_COMPLETE").touch()
        return {"result": json.loads(result_path.read_text()), "steps": steps, "trace": trace,
                "status": status, "cfg": cfg, "manifest": manifest, "recorder_final": recorder_final,
                "video": video, "before": file_snapshot(root)}

    def ffprobe(self, argv, *args, **kwargs):
        self.assertEqual(Path(argv[0]).name, "ffprobe")
        return subprocess.CompletedProcess(argv, 0, json.dumps(VIDEO_PROBE), "")

    def compact(self, root, **kwargs):
        with patch.object(A.subprocess, "run", side_effect=self.ffprobe):
            return A.compact_run(root, RUN_ID, **kwargs)

    def assert_rejected_without_changes(self, root, **kwargs):
        before = file_snapshot(root)
        with self.assertRaises((ValueError, RuntimeError, OSError)):
            self.compact(root, **kwargs)
        self.assertEqual(file_snapshot(root), before)

    def test_execution_events_survive_compaction_and_reject_timing_tampering(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            self.make_run(root)
            # Four execution handoffs, including both modes, versus three snapshots.
            events = [{"execution_step": i, "execution_wall_time": t,
                       "utime": 100_000 + i, "mode": "c3" if i > 1 else "reposition"}
                      for i, t in enumerate([0.0, .03, .08, .11])]
            with (root / "planner.log").open("a") as stream:
                for event in events:
                    stream.write("[C3_EXECUTION_STEP] " + json.dumps(event) + "\n")
            path = root / f"{RUN_ID}_result.json"
            result = json.loads(path.read_text())
            P.add_execution_timing(result, events)
            P.write_result_json(path, result)
            compacted = self.compact(root)
            self.assertEqual(compacted["recording"]["execution_steps"], events)
            self.assertEqual(compacted["dynamic"]["execution_wall_time"], [0.0, .03, .08, .11])
            self.assertAlmostEqual(compacted["execution_timing"]["frequency_hz"], 3 / .11)
            cfg = compacted["provenance"]["evaluation_scene_config"]
            A._validate(compacted, compacted["recording"], cfg, root)
            for field in ("frequency_hz", "n_steps"):
                changed = deepcopy(compacted)
                changed["execution_timing"][field] += 1
                with self.subTest(field=field), self.assertRaises(ValueError):
                    A._validate(changed, changed["recording"], cfg, root)
            changed = deepcopy(compacted)
            changed["dynamic"]["execution_wall_time"][1] += .001
            with self.assertRaises(ValueError):
                A._validate(changed, changed["recording"], cfg, root)
            self.assertEqual(set(p.name for p in root.iterdir()), {path.name, f"{RUN_ID}.mp4"})

    def test_physical_execution_survives_packaging_with_independent_snapshot_data(self):
        from . import test_object_runs as fixtures
        native, updates = fixtures.ResultProjectionTests().physical_fixture()
        native["headers"][0]["step_budget"] = None
        native["terminals"][0]["reason"] = "shutdown"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root)
            with (root / "sim.log").open("a") as stream:
                for key, prefix in (("headers", "C3_EXECUTION_LOGGING"),
                                    ("boundaries", "C3_EXECUTION_BOUNDARY"),
                                    ("terminals", "C3_EXECUTION_TERMINAL")):
                    for record in native[key]:
                        stream.write(f"[{prefix}] " + json.dumps(record) + "\n")
            with (root / "planner.log").open("a") as stream:
                for update in updates:
                    stream.write("[C3_PLANNING_UPDATE] " + json.dumps(update) + "\n")
            path = root / f"{RUN_ID}_result.json"
            result = json.loads(path.read_text())
            cfg = result["provenance"]["evaluation_scene_config"]
            P.add_execution_projection(result, cfg, native, updates)
            P.write_result_json(path, result)
            # Physical telemetry is present before compaction, but raw CSV and
            # observations still live in sidecars at this point.
            with redirect_stdout(io.StringIO()):
                P.export_existing_result(SimpleNamespace(run_dir=str(root), scene="open_task", run_id=RUN_ID), None)
            self.assertEqual(json.loads(path.read_text())["execution"], result["execution"])
            compacted = self.compact(root)
            self.assertEqual(compacted["recording"]["execution_native"], native)
            self.assertEqual(compacted["recording"]["planning_updates"], updates)
            self.assertEqual(compacted["recording"]["steps_raw"], original["steps"])
            self.assertEqual(compacted["recording"]["state_trace"], original["trace"])
            self.assertEqual(compacted["dynamic"], result["dynamic"])
            self.assertEqual(compacted["execution"]["n_steps_executed"], 2)
            A._validate(compacted, compacted["recording"], cfg, root)
            for key in ("frequency_hz", "n_steps_executed"):
                bad = deepcopy(compacted)
                bad["execution"][key] += 1
                with self.subTest(key=key), self.assertRaises(ValueError):
                    A._validate(bad, bad["recording"], cfg, root)
            self.assertEqual({p.name for p in root.iterdir()}, {path.name, f"{RUN_ID}.mp4"})
            with redirect_stdout(io.StringIO()):
                P.export_existing_result(SimpleNamespace(run_dir=str(root), scene="open_task", run_id=RUN_ID), None)
            self.assertEqual(json.loads(path.read_text()), compacted)

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

    def test_finished_inline_packaging_does_not_require_a_legacy_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root)
            (root / "RUN_COMPLETE").unlink()
            (root / "runtime_status.json").unlink()
            result = self.compact(root, status=original["status"], require_legacy_complete=False)
            self.assertTrue(A.completion(root, RUN_ID))
            self.assertEqual(result["runtime_status"], original["status"])

    def test_captured_source_state_survives_projection_and_compaction(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "source"
            repo.mkdir()

            def git(*arguments):
                return subprocess.check_output(["git", *arguments], cwd=repo,
                                               stderr=subprocess.PIPE)

            git("init", "-q")
            tracked = repo / "controller.py"
            tracked.write_text("version = 'committed'\n")
            git("add", "controller.py")
            git("-c", "user.name=Fixture", "-c", "user.email=fixture@example.invalid",
                "commit", "-qm", "fixture baseline")
            tracked.write_text("version = 'staged'\n")
            git("add", "controller.py")
            tracked.write_text("version = 'launch-time worktree'\n")
            (repo / "untracked_source.bin").write_bytes(b"\xffsource bytes\x00")
            captured = R.capture_source_state(repo)
            self.assertTrue(captured["worktree_dirty"])
            self.assertIn("untracked_source.bin", captured["untracked_files"])

            root = Path(tmp) / "run"
            original = self.make_run(root)
            raw = (json.dumps(captured, indent=2, allow_nan=False) + "\n").encode("utf-8")
            source_path = root / "config/source_state.json"
            source_path.write_bytes(raw)
            descriptor = {"path": "config/source_state.json",
                          "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw),
                          "base_commit": captured["base_commit"], "scope": captured["scope"]}
            status = {**original["status"], "commit": captured["base_commit"],
                      "worktree_dirty": captured["worktree_dirty"], "source_state": descriptor}
            (root / "runtime_status.json").write_text(json.dumps(status))
            with (root / f"{RUN_ID}_metrics.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            projected = P.project_result(root, "open_task", RUN_ID, original["cfg"],
                                         original["result"], original["steps"], rows,
                                         evaluation_costs=original["manifest"]["evaluation"]["costs"])
            self.assertEqual(projected["provenance"]["source_state"]["status"], "recorded")
            self.assertIsNone(projected["provenance"]["source_state"]["embedded_json_pointer"])
            P.write_result_json(root / f"{RUN_ID}_result.json", projected)

            # The captured hash must reject changes before any run evidence is deleted.
            source_path.write_bytes(raw + b"\n")
            self.assert_rejected_without_changes(root)
            source_path.write_bytes(raw)
            result = self.compact(root)
            self.assertEqual({path.name for path in root.iterdir()},
                             {f"{RUN_ID}_result.json", f"{RUN_ID}.mp4"})
            embedded = result["provenance"]["configuration"]["files"]["config/source_state.json"]
            self.assertEqual(embedded["text"].encode("utf-8"), raw)
            self.assertEqual(embedded["data"], captured)
            self.assertEqual(embedded["sha256"], descriptor["sha256"])
            self.assertEqual(embedded["size_bytes"], descriptor["size_bytes"])
            self.assertEqual(result["runtime_status"], status)
            provenance = result["provenance"]["source_state"]
            self.assertEqual({key: provenance[key] for key in descriptor}, descriptor)
            target = result
            for token in provenance["embedded_json_pointer"].lstrip("/").split("/"):
                target = target[token.replace("~1", "/").replace("~0", "~")]
            self.assertEqual(target, captured)
            self.assertIsNone(result["native_controller"]["success"])
            self.assertEqual(result["dynamic"], P._json_values(projected["dynamic"]))
            self.assertEqual(result["recording"]["steps_raw"], original["steps"])
            self.assertEqual((root / f"{RUN_ID}.mp4").read_bytes(), original["video"])
            # Validation and semantic refresh use embedded evidence after config/ is gone.
            A._validate(result, result["recording"], original["cfg"], root)
            self.assertEqual(P.add_result_semantics(deepcopy(result), root), result)
            self.assertTrue(A.completion(root, RUN_ID))

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
            argv = ["postprocess_run.py", "--run-dir", str(root), "--scene", "open_task",
                    "--run-id", RUN_ID, "--export-only"]
            with patch("sys.argv", argv), redirect_stdout(io.StringIO()), \
                    patch.object(P, "build_fk", side_effect=AssertionError("FK forbidden")) as fk, \
                    patch("subprocess.run", side_effect=AssertionError("External process forbidden")), \
                    patch("subprocess.check_output", side_effect=AssertionError("Current runtime lookup forbidden")):
                P.main()
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
            argv = ["postprocess_run.py", "--run-dir", str(root), "--scene", "open_task",
                    "--run-id", RUN_ID, "--export-only"]
            for attempt in range(2):
                with patch("sys.argv", argv), redirect_stdout(io.StringIO()), \
                        patch("subprocess.run", side_effect=AssertionError("External process forbidden")):
                    P.main()
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

    def test_render_obstacle_fallback_reads_new_and_legacy_native_snapshots(self):
        try:
            from . import render_run_3d as renderer
        except ModuleNotFoundError as exc:
            if exc.name == "pydrake":
                self.skipTest("Drake unavailable for importing the renderer")
            raise
        configurations = {"config/simulation.yaml": {"scenario_name": "saved-scene",
                                                     "obstacle_model": "saved-obstacle.sdf"}}
        variants = ({"hyperparameters": {"c3plus": {"configurations": configurations}}},
                    {"provenance": {"c3plus": {"configurations": configurations}}},
                    {"provenance": {"configuration": {"files": {
                        name: {"data": value} for name, value in configurations.items()}}}})
        trace = [{"t": 1., "q": [0.] * 5, "obj": [1., 0., 0., 0., .4, .1, .03]}]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "result.json"
            for variant in variants:
                with self.subTest(layout=variant):
                    path.write_text(json.dumps(variant))
                    args = SimpleNamespace(result=str(path), object_sdf="object.sdf")
                    with patch.object(renderer, "load_result_trace", return_value=trace), \
                            patch.object(renderer, "resolve_result_model", return_value="resolved-obstacle.sdf") as resolve:
                        rows, obj, obstacle, goal = renderer.load_render_input(args)
                    self.assertEqual((rows, obj, obstacle, goal), (trace, "object.sdf", "resolved-obstacle.sdf", None))
                    resolve.assert_called_once_with("saved-obstacle.sdf", str(path), variant)

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

    def interrupt_cleanup(self, root):
        original_unlink = Path.unlink
        target = root / "steps_raw.jsonl"

        def injected_disk_error(path, *args, **kwargs):
            if path == target:
                raise PermissionError("Injected cleanup failure")
            return original_unlink(path, *args, **kwargs)

        with patch.object(Path, "unlink", autospec=True, side_effect=injected_disk_error), \
                self.assertRaisesRegex(PermissionError, "Injected cleanup failure"):
            self.compact(root)

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

    def test_invalid_inputs_never_delete_or_replace_original_artifacts(self):
        def invalid_json(root):
            (root / f"{RUN_ID}_result.json").write_text("{broken JSON")

        def wrong_run_id(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["run_id"] = "a different run"
            path.write_text(json.dumps(value))

        def changed_projected_trajectory(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["dynamic"]["object_pose"][1][0] += 1
            path.write_text(json.dumps(value))

        def changed_observed_dt(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["hyperparameters"]["control_dt"] = .0001
            path.write_text(json.dumps(value))

        def invented_compute_time(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["dynamic"]["compute_time"] = [.001] * value["steps_run"]
            path.write_text(json.dumps(value))

        def incomplete_phase(root):
            path = root / "runtime_status.json"
            status = json.loads(path.read_text())
            del status["render_rc"]
            path.write_text(json.dumps(status))

        def failed_phase(root):
            path = root / "runtime_status.json"
            status = json.loads(path.read_text())
            status["postprocess_rc"] = 17
            path.write_text(json.dumps(status))

        mutations = {
            "invalid result JSON": invalid_json,
            "wrong run identity": wrong_run_id,
            "projection changed": changed_projected_trajectory,
            "physics dt substituted for observed interval": changed_observed_dt,
            "compute time invented without a recorded signal": invented_compute_time,
            "missing packaging phase": incomplete_phase,
            "failed packaging phase": failed_phase,
            "missing completion marker": lambda p: (p / "RUN_COMPLETE").unlink(),
            "invalid raw JSONL": lambda p: (p / "steps_raw.jsonl").write_text("not JSON\n"),
            "empty replay trace": lambda p: (p / "state_trace.jsonl").write_text(""),
            "missing metrics rows": lambda p: (p / f"{RUN_ID}_metrics.csv").write_text("control_step,sim_time\n"),
            "configuration changed after recording": lambda p: (p / "config/sampling.yaml").write_text(
                "sample_random_seed: 99\nsampling_strategy: 8\n"),
            "unknown personal file": lambda p: (p / "notes.txt").write_text("Keep these notes"),
            "untrusted symlink": lambda p: (p / "config/external.yaml").symlink_to(p / "config/goal.yaml"),
        }
        for description, mutate in mutations.items():
            with self.subTest(description=description), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "run"
                self.make_run(root)
                mutate(root)
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


if __name__ == "__main__":
    unittest.main()
