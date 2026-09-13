"""Object selection and packaging checks; native dynamics are always mocked."""
from contextlib import redirect_stderr, redirect_stdout
import csv
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from . import catalog as S
from . import cost_fig as C
from . import postprocess_run as P
from . import run_experiment as R


class ObjectRunTests(unittest.TestCase):
    def dry_run(self, out, *flags):
        stream = io.StringIO()
        with redirect_stdout(stream), patch.object(R, "run_one") as launch:
            R.main(["--scene", "open_task", "--out", str(out), "--dry-run", *flags])
        launch.assert_not_called()
        return json.loads(stream.getvalue())

    def test_four_object_dry_run_preserves_identity_and_creates_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "planned"
            data = self.dry_run(out, "--objects", *S.MESH_OBJECTS, "--start", "2", "--goal", "2",
                                "--goal-yaw-degrees", "90")
            self.assertFalse(out.exists())
            self.assertEqual(data["execution"], "serial")
            self.assertEqual(data["run_count"], 4)
            self.assertEqual([p["object_name"] for p in data["runs"]], list(S.MESH_OBJECTS))
            self.assertEqual(len({p["run_id"] for p in data["runs"]}), 4)
            self.assertEqual(len({p["configuration_digest"] for p in data["runs"]}), 4)
            for name, plan in zip(S.MESH_OBJECTS, data["runs"]):
                self.assertEqual(Path(plan["out"]), out / name)
                self.assertIn(name, plan["run_id"])
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
            self.assertEqual(default["run_id"], "exponential_open_task_s01g01_seed42")
            self.assertNotIn("object_name", default)
            self.assertFalse(out.exists())

    def test_multiselect_calls_serially_and_checks_all_destinations_first(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "runs"
            flags = ["--scene", "open_task", "--objects", *S.MESH_OBJECTS, "--out", str(root)]
            with patch.object(R, "run_one") as launch:
                R.main(flags)
            self.assertEqual(launch.call_count, 4)
            for name, call in zip(S.MESH_OBJECTS, launch.call_args_list):
                self.assertEqual(call.args[4], root / name)
                self.assertEqual(call.kwargs["object_name"], name)
            (root / S.MESH_OBJECTS[-1]).mkdir(parents=True)
            with patch.object(R, "run_one") as launch, redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit):
                R.main(flags)
            launch.assert_not_called()

    def test_bad_selection_is_rejected_before_any_launch_or_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "unused"
            for flags in (["--scene", "shelf_gap", "--objects", "banana"],
                          ["--scene", "open_task", "--objects", "banana", "banana"],
                          ["--scene", "open_task", "--objects", "unknown"]):
                with self.subTest(flags=flags), patch.object(R, "run_one") as launch, \
                        redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    R.main([*flags, "--out", str(out)])
                launch.assert_not_called()
                self.assertFalse(out.exists())

    def test_object_packaging_uses_selected_channel_model_and_geometry(self):
        for name in S.MESH_OBJECTS:
            with self.subTest(object=name), tempfile.TemporaryDirectory() as tmp:
                repo = Path(tmp)
                demo = S.demo_name("open_task", 2, 2, object_name=name)
                resolved = S.compose_demo_configs(demo, object_name=name)
                profile = S.resolve_object_profile("open_task", name)
                sources = [*S.load_demo_configs(demo, object_name=name), *S.model_assets(resolved),
                           S.REPO / profile["physics_metadata_file"]]
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
                        patch.object(R.subprocess, "check_output",
                                     side_effect=lambda cmd, **kw: "test" if kw.get("text") else b""), \
                        redirect_stdout(io.StringIO()):
                    status = R.run_one("open_task", "relu", 2, 2, out, object_name=name)
                self.assertEqual(status["object_name"], name)
                self.assertTrue(status["asset_sha256"])
                self.assertEqual(phases["launcher.log"][3], profile["object_channel_substring"])
                render = phases["render.log"]
                simulation_model = out / "config/repository" / profile["simulation_model"]
                self.assertEqual(render[render.index("--object-sdf") + 1], str(simulation_model))
                self.assertEqual(status["render_object_model"], str(simulation_model))
                self.assertTrue(simulation_model.is_file())
                evaluation_path = out / "evaluation_scene_config.yaml"
                cfg = yaml.safe_load(evaluation_path.read_text())
                for field in ("footprint", "block_half_height", "tip_target_z", "object_channel_substring"):
                    self.assertEqual(cfg[field], profile[field])
                self.assertEqual(cfg["tip_floor_z_real"], profile["tip_floor_z_real"])
                cost = phases["cost_fig.log"]
                self.assertEqual(cost[cost.index("--scene-config") + 1], str(evaluation_path))
                self.assertEqual(C.load_scene("open_task", evaluation_path)["footprint"],
                                 [tuple(map(float, p)) for p in profile["footprint"]])
                sim = yaml.safe_load((out / "config/simulation.yaml").read_text())
                controller = yaml.safe_load((out / "config/controller.yaml").read_text())
                self.assertEqual(sim["object_model"], str(simulation_model))
                self.assertEqual(controller["object_model"],
                                 str(out / "config/repository" / profile["controller_model"]))
                self.assertTrue((out / "RUN_COMPLETE").is_file())

    def test_cost_reconstruction_uses_saved_footprint(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "scene.yaml"
            config.write_text(yaml.safe_dump({"footprint": [[-.1, -.1], [.1, -.1], [.1, .1], [-.1, .1]],
                                              "obstacles": {"discs": [[.1, .1, .01]]}}))
            zeros = np.array([0.0])
            native_scene, native_has_obstacle = C.obs_curve("relu", "open_task", zeros, zeros, zeros)
            selected, selected_has_obstacle = C.obs_curve("relu", "open_task", zeros, zeros, zeros, config)
            self.assertFalse(native_has_obstacle)
            self.assertEqual(native_scene[0], 0)
            self.assertTrue(selected_has_obstacle)
            self.assertGreater(selected[0], 0)

    def test_model_snapshots_are_self_contained_and_keep_contact_properties(self):
        try:
            from pydrake.geometry import Convex, Sphere
            from pydrake.multibody.parsing import Parser
            from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
            from pydrake.systems.framework import DiagramBuilder
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        for name in S.MESH_OBJECTS:
            with self.subTest(object=name), tempfile.TemporaryDirectory() as tmp:
                directory = Path(tmp) / "config"
                demo = S.demo_name("open_task", 2, 2, name)
                profile = S.resolve_object_profile("open_task", name)
                controller_file = S.write_demo_configs(demo, directory)
                controller = yaml.safe_load(controller_file.read_text())
                simulation = yaml.safe_load((directory / "simulation.yaml").read_text())
                channels = yaml.safe_load(Path(controller["lcm_channels_simulation_file"]).read_text())
                self.assertEqual(channels["object_state_channels"][0], f"OBJECT_{name}_base_STATE_SIMULATION")
                self.assertTrue(Path(controller["sampling_mesh_files"][0]).is_relative_to(directory))
                for role, config in (("simulation", simulation), ("controller", controller)):
                    model_file = Path(config["object_models"][0])
                    self.assertTrue(model_file.is_relative_to(directory))
                    self.assertIn("drake:declare_convex", model_file.read_text())
                    builder = DiagramBuilder()
                    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
                    instance, = Parser(plant).AddModels(str(model_file))
                    plant.Finalize()
                    context = plant.CreateDefaultContext()
                    body = plant.GetBodyByName(profile["object_body_name"], instance)
                    self.assertAlmostEqual(body.get_mass(context), 0.1)
                    ids = plant.GetCollisionGeometriesForBody(body)
                    inspector = scene_graph.model_inspector()
                    pieces = profile["physics"]["collision_piece_count"]
                    self.assertEqual(len(ids), pieces + (3 if role == "controller" else 0))
                    self.assertTrue(all(isinstance(inspector.GetShape(g), Convex) for g in ids[:pieces]))
                    for gid in ids[:pieces]:
                        friction = inspector.GetProximityProperties(gid).GetProperty("material", "coulomb_friction")
                        self.assertAlmostEqual(friction.static_friction(), 0.3)
                    if role == "controller":
                        self.assertTrue(all(isinstance(inspector.GetShape(g), Sphere) for g in ids[-3:]))
                    self.assertEqual(context.get_time(), 0)

    def test_selected_geometry_and_physics_change_the_configuration_digest(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            name = "banana"
            demo = S.demo_name("open_task", 2, 2, name)
            resolved = S.compose_demo_configs(demo)
            profile = S.resolve_object_profile("open_task", name)
            for source in [*S.load_demo_configs(demo), *S.model_assets(resolved),
                           S.REPO / profile["physics_metadata_file"]]:
                target = repo / source.relative_to(S.REPO)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
            original = S.demo_config_digest(demo, repo)
            mesh = repo / resolved["controller"]["sampling_mesh_files"][0]
            mesh.write_text(mesh.read_text() + "\n# Changed source revision\n")
            changed_mesh = S.demo_config_digest(demo, repo)
            self.assertNotEqual(original, changed_mesh)
            metadata_file = repo / profile["physics_metadata_file"]
            metadata = json.loads(metadata_file.read_text())
            metadata["mass_kg"] = 0.2
            metadata_file.write_text(json.dumps(metadata))
            self.assertNotEqual(changed_mesh, S.demo_config_digest(demo, repo))

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
            fk = lambda q: (np.array([.37, -.4, -.014]), np.zeros(3), np.diag([1, -1, -1]))
            argv = ["postprocess_run.py", "--run-dir", str(out), "--scene", "open_task",
                    "--run-id", "object_test", "--scene-config", str(config)]
            with patch("sys.argv", argv), patch.object(P, "build_fk", return_value=fk), \
                    patch.object(P.subprocess, "check_output", return_value=b"test"), \
                    redirect_stdout(io.StringIO()):
                P.main()
            result = json.loads((out / "object_test_result.json").read_text())
            manifest = yaml.safe_load((out / "object_test_manifest.yaml").read_text())
            for key, value in identity.items():
                self.assertEqual(result[key], value)
                self.assertEqual(manifest[key], value)
            self.assertEqual(result["n_control_steps"], 2)
            with (out / "object_test_metrics.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(float(rows[0]["tip_z_cost"]), 0.0)


if __name__ == "__main__":
    unittest.main()
