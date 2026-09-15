from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch
import yaml

from c3plus import configs as S
from c3plus.utils import run as R
from c3plus.runtime import provenance as Provenance
from c3plus.visualization import costs as C

from tests.fixtures.results import ObjectFixtures

class ObjectRunTests(ObjectFixtures, unittest.TestCase):
    def test_execution_budget_is_explicit_and_does_not_change_native_parameters(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "planned"
            unlimited = self.dry_run(out)
            bounded = self.dry_run(out, "--steps", "25")
            self.assertIsNone(unlimited.get("execution_step_budget"))
            self.assertEqual(bounded.pop("execution_step_budget"), 25)
            self.assertEqual(bounded, unlimited)
            for value in ("0", "-1"):
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    self.dry_run(out, "--steps", value)
            self.assertFalse(out.exists())


    def test_five_object_dry_run_preserves_identity_and_creates_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "planned"
            data = self.dry_run(out, "--objects", *S.RUN_OBJECTS, "--start", "2", "--goal", "2",
                                "--goal-yaw-degrees", "90")
            self.assertFalse(out.exists())
            self.assertEqual(data["execution"], "serial")
            self.assertEqual(S.RUN_OBJECTS, S.OBJECTS)
            self.assertEqual(data["run_count"], 5)
            self.assertEqual([p["object_name"] for p in data["runs"]], list(S.RUN_OBJECTS))
            self.assertEqual(len({p["run_id"] for p in data["runs"]}), 5)
            self.assertEqual(len({p["configuration_digest"] for p in data["runs"]}), 5)
            for name, plan in zip(S.RUN_OBJECTS, data["runs"]):
                self.assertEqual(Path(plan["out"]), out / name)
                self.assertIn(name, plan["run_id"])
                if name == "T_shape":
                    self.assertEqual(plan["demo"], S.demo_name("open_task", 2, 2))
                else:
                    self.assertIn(name, plan["demo"])
                self.assertEqual(plan["simulation_model"], plan["object_profile"]["simulation_model"])
                self.assertEqual(plan["controller_model"], plan["object_profile"]["controller_model"])
                self.assertEqual(plan["goal_yaw_degrees"], 90)


    def test_single_object_exact_output_and_legacy_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "exact"
            selected = self.dry_run(out, "--objects", "banana")
            self.assertEqual(Path(selected["out"]), out)
            self.assertEqual(selected["object_name"], "banana")
            default = self.dry_run(out)
            self.assertEqual(default["run_id"], "exponential_open_table_T_shape_s01g01_seed42")
            self.assertEqual(default["object_name"], "T_shape")
            self.assertFalse(out.exists())


    def test_explicit_t_block_preserves_native_configuration_and_default_run_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "planned"
            for start, goal, yaw in ((1, 1, None), (2, 2, 90), (5, 3, -90)):
                with self.subTest(start=start, goal=goal, yaw=yaw):
                    flags = ["--start", str(start), "--goal", str(goal)]
                    if yaw is not None:
                        flags += ["--goal-yaw-degrees", str(yaw)]
                    default = self.dry_run(out, *flags)
                    selected = self.dry_run(out, "--objects", "T_block", *flags)
                    for key in ("demo", "controller_goal", "evaluation_goal", "start_pose", "configuration_digest"):
                        self.assertEqual(selected[key], default[key], key)
                    self.assertEqual(selected["run_id"], default["run_id"])
                    self.assertEqual(selected["object_name"], "T_shape")
                    self.assertEqual(Path(selected["out"]), out)
                    self.assertEqual(selected["object_channel_substring"], "G_shape_video")
                    self.assertTrue(selected["asset_sha256"])
                    native = S.compose_demo_configs(default["demo"], goal_yaw_degrees=yaw)
                    explicit = S.compose_demo_configs(default["demo"], goal_yaw_degrees=yaw, object_name="T_block")
                    self.assertEqual(explicit, native)
                    self.assertEqual(explicit["controller"]["object_model"],
                                     "examples/sampling_c3/urdf/push_t_oimscale_m01_control.sdf")
                    self.assertEqual(explicit["simulation"]["object_model"],
                                     "examples/sampling_c3/urdf/push_t_oimscale_m01.sdf")
                    self.assertIn("profiles/t_shape/", explicit["controller"]["sampling_params_file"])
            self.assertFalse(out.exists())


    def test_multiselect_calls_serially_and_checks_all_destinations_first(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            flags = ["--scene", "open_task", "--objects", *S.RUN_OBJECTS, "--out", str(root)]
            with patch.object(R, "run_one") as launch:
                R.main(flags)
            self.assertEqual(launch.call_count, 5)
            for name, call in zip(S.RUN_OBJECTS, launch.call_args_list):
                self.assertEqual(call.args[4], root / name)
                self.assertEqual(call.kwargs["object_name"], name)
            (root / S.RUN_OBJECTS[-1]).mkdir(parents=True)
            with patch.object(R, "run_one") as launch, redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit):
                R.main(flags)
            launch.assert_not_called()


    def test_bad_selection_is_rejected_before_any_launch_or_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "unused"
            for flags in (["--scene", "open_task", "--objects", "banana", "banana"],
                          ["--scene", "open_task", "--objects", "Tblock"],
                          ["--scene", "open_task", "--objects", "Cblock"],
                          ["--scene", "open_task", "--objects", "unknown"]):
                with self.subTest(flags=flags), patch.object(R, "run_one") as launch, \
                        redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    R.main([*flags, "--out", str(out)])
                launch.assert_not_called()
                self.assertFalse(out.exists())


    def test_object_packaging_uses_selected_channel_model_and_geometry(self):
        for name in S.RUN_OBJECTS:
            with self.subTest(object=name), tempfile.TemporaryDirectory() as tmp:
                repo = Path(tmp)
                demo = S.demo_name("open_task", 2, 2, object_name=name)
                resolved = S.compose_demo_configs(demo, object_name=name)
                profile = S.resolve_object_profile("open_task", name)
                sources = [*S.load_demo_configs(demo, object_name=name), *S.model_assets(resolved)]
                if name in S.MESH_OBJECTS:
                    sources.append(S.REPO / profile["physics_metadata_file"])
                for source in sources:
                    target = repo / source.relative_to(S.REPO)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source, target)
                out = repo / "output"
                phases = {}

                def logged(command, log, env):
                    phases[log.name] = command
                    log.touch()
                    if log.name == "launcher.log":
                        (out / "planner.log").write_text("[SAMPLER-SEED] deterministic seed=42\n")
                        for filename in ("sim.log", "osc.log", "steps_raw.jsonl", "state_trace.jsonl"):
                            (out / filename).write_text("mock-data")
                    return 0

                with patch.object(R, "REPO", repo), patch.object(R, "BINARIES", ()), \
                        patch.object(R, "logged_command", side_effect=logged), \
                        patch.object(R, "capture_source_state", return_value={
                            "format": "git-source-state/v1", "base_commit": "test-commit",
                            "worktree_dirty": False, "scope": "test fixture",
                            "tracked_patch": {"encoding": "utf-8", "content": "", "size_bytes": 0,
                                              "sha256": hashlib.sha256(b"").hexdigest()},
                            "git_status": {"encoding": "utf-8", "content": "", "size_bytes": 0,
                                           "sha256": hashlib.sha256(b"").hexdigest()},
                            "untracked_files": {}}), \
                        patch.object(R, "compact_run") as compact, \
                        patch.object(Provenance.subprocess, "check_output",
                                     side_effect=lambda cmd, **kw: "test" if kw.get("text") else b""), \
                        redirect_stdout(io.StringIO()):
                    status = R.run_one("open_task", "relu", 2, 2, out, object_name=name)
                self.assertEqual(status["object_name"], name)
                self.assertTrue(status["asset_sha256"])
                channel = profile["object_channel_substring"] if name in S.MESH_OBJECTS else "G_shape_video"
                self.assertEqual(phases["launcher.log"][4], channel)
                render = phases["render.log"]
                simulation_model = ((out / "config/repository" if name in S.MESH_OBJECTS else repo)
                                    / profile["simulation_model"])
                self.assertEqual(render[render.index("--object-sdf") + 1], str(simulation_model))
                self.assertEqual(status["render_object_model"], str(simulation_model))
                self.assertTrue(simulation_model.is_file())
                evaluation_path = out / "evaluation_scene_config.yaml"
                cfg = yaml.safe_load(evaluation_path.read_text())
                expected_evaluation = S.evaluation_config("open_table", name)
                for field in ("footprint", "block_half_height", "tip_target_z", "object_channel_substring"):
                    self.assertEqual(cfg[field], expected_evaluation[field])
                if "tip_floor_z_real" in expected_evaluation:
                    self.assertEqual(cfg["tip_floor_z_real"], expected_evaluation["tip_floor_z_real"])
                else:
                    self.assertNotIn("tip_floor_z_real", cfg)
                if name == "T_shape":
                    for key, value in expected_evaluation.items():
                        if key != "goal":
                            self.assertEqual(cfg[key], value, key)
                    self.assertEqual(cfg["object_channel_substring"], "G_shape_video")
                self.assertEqual(cfg["goal"], list(status["evaluation_goal"]))
                self.assertNotIn("cost_fig.log", phases)
                self.assertEqual(render[render.index("--result") + 1], str(out / f"{status['run_id']}_result.json"))
                self.assertEqual(C.load_scene("open_task", evaluation_path)["footprint"],
                                 [tuple(map(float, p)) for p in expected_evaluation["footprint"]])
                sim = yaml.safe_load((out / "config/simulation.yaml").read_text())
                controller = yaml.safe_load((out / "config/controller.yaml").read_text())
                self.assertEqual(sim["object_model"], str(simulation_model) if name in S.MESH_OBJECTS
                                 else profile["simulation_model"])
                self.assertEqual(controller["object_model"],
                                 str(out / "config/repository" / profile["controller_model"])
                                 if name in S.MESH_OBJECTS else profile["controller_model"])
                compact.assert_called_once_with(out, status["run_id"], status=status,
                                                require_legacy_complete=False)
                self.assertFalse((out / "RUN_COMPLETE").exists())
