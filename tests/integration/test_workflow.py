import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from c3plus.utils import run as R

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_source_capture_failure_prevents_output_and_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            self.copy_demo_configs("matched_single_obstacle_xarm6_t1", repo)
            out = repo / "output"
            with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                    patch.object(R, "capture_source_state", side_effect=RuntimeError("capture failed")), \
                    patch.object(R, "logged_command") as launch, \
                    self.assertRaisesRegex(RuntimeError, "capture failed"):
                R.run_one("single_obstacle", "exponential", 1, 1, out)
            launch.assert_not_called()
            self.assertFalse(out.exists())


    def test_yaw_launch_packaging_and_native_target_verification(self):
        cases = [(90, "correct"), (0, "correct"), (-90, "correct"),
                 (90, "missing"), (90, "wrong_position"), (90, "stale_binary")]
        for yaw, condition in cases:
            with self.subTest(yaw=yaw, condition=condition), tempfile.TemporaryDirectory() as tmp:
                repo = Path(tmp)
                self.copy_demo_configs("matched_single_obstacle_xarm6_t2", repo)
                out = repo / "output"
                phases = []

                def logged(command, log, env):
                    phases.append((log.name, command))
                    log.touch()
                    if log.name != "launcher.log":
                        return 0
                    if condition == "stale_binary":
                        log.write_text("Controller does not support --goal_yaw_degrees; rebuild")
                        return 2
                    banner = "[SAMPLER-SEED] deterministic seed=42\n"
                    if condition != "missing":
                        x = "0.9" if condition == "wrong_position" else "0.397"
                        banner += f"[GOAL-YAW] goal_yaw_degrees={yaw} goal_x={x} goal_y=-0.431\n"
                    (out / "planner.log").write_text(banner)
                    for name in ("sim.log", "osc.log", "steps_raw.jsonl", "state_trace.jsonl"):
                        (out / name).write_text("test-data")
                    return 0

                with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                        patch.object(R, "logged_command", side_effect=logged), \
                        patch.object(R, "compact_run") as compact, \
                        patch.object(R, "capture_source_state", return_value=self.source_state_fixture()):
                    if condition == "correct":
                        status = R.run_one("single_obstacle", "relu", 2, 2, out, goal_yaw_degrees=yaw)
                        self.assertTrue(status["goal_yaw_verified"])
                    else:
                        with self.assertRaises(RuntimeError):
                            R.run_one("single_obstacle", "relu", 2, 2, out, goal_yaw_degrees=yaw)
                saved = json.loads((out / "runtime_status.json").read_text())
                self.assertEqual(saved["goal_yaw_degrees"], yaw)
                self.assertEqual(compact.call_count, int(condition == "correct"))
                self.assertFalse((out / "RUN_COMPLETE").exists())
                self.assertEqual(phases[0][1][-1], str(yaw))
                self.assertEqual(phases[0][1][-2], "--goal-yaw-degrees")
                self.assertIn("--controller-params", phases[0][1])
                if condition == "correct":
                    target = [0.397, -0.431, math.radians(yaw)]
                    self.assertEqual(saved["controller_goal"], target)
                    evaluation = R.yaml.safe_load((out / "evaluation_scene_config.yaml").read_text())
                    self.assertEqual(evaluation["goal"], target)
                    self.assertEqual(phases[0][1][5:8], list(map(str, target)))
                    render = next(cmd for phase, cmd in phases if phase == "render.log")
                    index = render.index("--goal")
                    self.assertEqual(render[index + 1:index + 4], list(map(str, target)))
                else:
                    self.assertEqual(len(phases), 1)


    def test_packaging_uses_recorded_goal_and_run_temporary_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            self.copy_demo_configs("matched_single_obstacle_xarm6_t1", repo)
            out = repo / "output"
            calls = []

            def logged(command, log, env):
                calls.append((log.name, command, dict(env)))
                log.touch()
                if log.name == "launcher.log":
                    (out / "planner.log").write_text("[SAMPLER-SEED] deterministic seed=42")
                    for name in ("sim.log", "osc.log", "steps_raw.jsonl", "state_trace.jsonl"):
                        (out / name).write_text("test-data")
                return 0

            def capture(repo_path):
                self.assertEqual(repo_path, repo)
                self.assertFalse(out.exists())
                saved = self.source_state_fixture()
                saved["worktree_dirty"] = True
                return saved

            with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                    patch.object(R, "logged_command", side_effect=logged), \
                    patch.object(R, "compact_run") as compact, \
                    patch.dict(R.os.environ, {"C3PLUS_CONTAINER_IMAGE": "test:tag",
                                              "C3PLUS_CONTAINER_IMAGE_ID": "sha256:test",
                                              "SAMPLING_C3_OBS_BOXES": "stale"}), \
                    patch.object(R, "capture_source_state", side_effect=capture) as capture_call:
                status = R.run_one("single_obstacle", "exponential", 1, 1, out)
            capture_call.assert_called_once_with(repo)
            source_bytes = (out / "config/source_state.json").read_bytes()
            self.assertEqual(status["source_state"]["sha256"], hashlib.sha256(source_bytes).hexdigest())
            self.assertEqual(status["source_state"]["size_bytes"], len(source_bytes))
            self.assertEqual(status["source_state"]["path"], "config/source_state.json")
            self.assertEqual(status["commit"], "test-commit")
            self.assertTrue(status["worktree_dirty"])
            compact.assert_called_once_with(out, status["run_id"], status=status,
                                            require_legacy_complete=False,
                                            video_required=True)
            self.assertFalse((out / "RUN_COMPLETE").exists())
            self.assertEqual(status["runtime"]["container_image"], "test:tag")
            self.assertEqual(status["runtime"]["container_image_id"], "sha256:test")
            self.assertEqual(status["goal"], status["controller_goal"])
            self.assertEqual(status["obstacle_cost"], "exponential")
            self.assertEqual(status["sampler_settings"]["SAMPLING_C3_OBS_BOXES"], "0.35,0,0.05,0.05")
            self.assertEqual(calls[0][2]["SAMPLING_C3_OBS_BOXES"], "0.35,0,0.05,0.05")
            self.assertEqual(calls[0][1][-2:], ["--controller-params", status["controller_params_file"]])
            self.assertTrue(status["config_sha256"])
            controller = R.yaml.safe_load(Path(status["controller_params_file"]).read_text())
            native_goal = R.yaml.safe_load(Path(controller["goal_params_file"]).read_text())
            self.assertEqual(native_goal["fixed_target_position"][:2], list(status["goal"][:2]))
            self.assertEqual(calls[0][1][5:8], list(map(str, status["goal"])))
            for _, _, env in calls[1:]:
                self.assertEqual(env["TMPDIR"], str(out / "tmp"))
            render = next(command for phase, command, _ in calls if phase == "render.log")
            index = render.index("--goal")
            self.assertEqual(render[index + 1:index + 4], list(map(str, status["goal"])))
            self.assertEqual(render[render.index("--result") + 1], str(out / f"{status['run_id']}_result.json"))
            self.assertEqual([phase for phase, _, _ in calls], ["launcher.log", "postprocess.log", "render.log"])
