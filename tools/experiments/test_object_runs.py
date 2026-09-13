"""Object selection and packaging checks; native dynamics are always mocked."""
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import csv
import hashlib
import io
import json
import math
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


class ResultProjectionTests(unittest.TestCase):
    """Synthetic log projections; never construct FK or advance dynamics."""

    def fixture(self):
        run_id = "synthetic_banana_run"
        channel = "OBJECT_banana_base_STATE_SIMULATION"
        cfg = {"object_name": "banana", "object_body_name": "banana_base",
               "object_channel_substring": "banana_base", "goal": [.4, -.4, 0],
               "footprint": [[-.02, -.02], [.02, -.02], [.02, .02], [-.02, .02]],
               "pusher_radius": .00555, "block_half_height": .01837,
               "tip_target_z": -.012, "tip_floor_z_real": -.02345,
               "obstacles": {}}
        times = [10.0, 10.1, 10.4, 10.4, 10.2]
        xs, ys = [.30, .31, .37, .40, .42], [.10, .12, .09, .09, .08]
        zs = [-.029, -.027, -.021, -.019, -.020]
        yaws = [math.pi - .05, -math.pi + .05, -math.pi + .11,
                -math.pi + .20, -math.pi + .30]
        tip_xs, tip_ys = [.20, .21, .27, .29, .30], [.10, .095, .11, .10, .10]
        steps, rows = [], []
        for index, (time, x, y, z, yaw) in enumerate(zip(times, xs, ys, zs, yaws)):
            step = 10 + 3 * index  # A logged sample gap is not a fabricated interval.
            steps.append({"control_step": step, "sim_time": time,
                          "robot_q": [index + j / 10 for j in range(5)],
                          "robot_v": [j / 100 for j in range(5)],
                          "robot_u": [100 + index + j for j in range(5)],
                          "objects": {channel: [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2), x, y, z]}})
            rows.append({"run_id": run_id, "scenario": "open_task", "control_step": step,
                         "sim_time": time, "object_x": x, "object_y": y, "object_yaw": yaw,
                         "goal_x": .4, "goal_y": -.4, "goal_yaw": 0,
                         "position_error_m": math.hypot(x - .4, y + .4),
                         "orientation_error_rad": abs(P.wrap(yaw)),
                         "tip_x": tip_xs[index], "tip_y": tip_ys[index],
                         "tip_z": -.012 + .001 * index,
                         "tip_roll": math.pi - .1 * index, "tip_pitch": 0, "tip_yaw": 0,
                         "physical_contact_active": int(index % 2 == 0),
                         "pusher_object_gap": .002, "min_obstacle_clearance": float("nan"),
                         **{key: float("nan") if key == "admm_penalty" else 1.0 for key in P.BLOCKS},
                         "evaluation_total": 12.0})
        summary = {"run_id": run_id, "scenario": "open_task", "success": False,
                   "t_success": None, "first_success_t": None,
                   "final_position_error": rows[-1]["position_error_m"],
                   "final_orientation_error": rows[-1]["orientation_error_rad"],
                   "best_pos_err": .125, "best_ang_err": .25,
                   "n_control_steps": len(rows), "sim_time_end": times[-1],
                   "object_name": "banana", "legacy_annotation": {"keep": "unchanged"}}
        return cfg, summary, steps, rows

    def project(self, directory, cfg, summary, steps, rows, **kwargs):
        return P.project_result(Path(directory), "open_task", summary["run_id"],
                                cfg, summary, steps, rows, **kwargs)

    def test_state_and_interval_lengths_preserve_legacy_summary(self):
        cfg, summary, steps, rows = self.fixture()
        original = deepcopy(summary)
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows)
        self.assertEqual(summary, original)
        for key, value in original.items():
            self.assertEqual(result[key], value)
        self.assertEqual(result["steps_run"], 4)
        self.assertEqual(result["n_control_steps"], 5)
        dynamic = result["dynamic"]
        for key in ("time", "object_pose", "object_velocity", "robot_pos", "robot_vel",
                    "qpos", "qvel", "object_pose_3d", "tip_z_state", "tip_tilt_state",
                    "robot_joint_effort"):
            self.assertEqual(len(dynamic[key]), 5, key)
        for key in ("robot_control", "compute_time", "contact_normal_force_z", "robot_contact_force",
                    "tip_z", "tip_tilt"):
            self.assertEqual(len(dynamic[key]), 4, key)
        self.assertEqual(dynamic["time"], [row["sim_time"] for row in rows])
        self.assertEqual(dynamic["tip_z"], dynamic["tip_z_state"][1:])
        self.assertEqual(dynamic["tip_tilt"], dynamic["tip_tilt_state"][1:])
        np.testing.assert_allclose(dynamic["tip_tilt_state"], [.0, .1, .2, .3, .4], atol=1e-12)
        self.assertEqual(dynamic["object_pose_3d"], [next(iter(step["objects"].values())) for step in steps])
        self.assertEqual(dynamic["qpos"][1], steps[1]["robot_q"] +
                         [rows[1]["object_x"], rows[1]["object_y"], rows[1]["object_yaw"], -.027])
        self.assertTrue(all(len(row) == 9 for row in dynamic["qpos"]))
        self.assertTrue(all(len(row) == 9 for row in dynamic["qvel"]))
        for key in ("schema", "run", "hyperparameters", "static", "provenance"):
            self.assertIn(key, result)
        for key in ("indexing", "frames", "velocities", "missing", "units", "controls", "qpos"):
            self.assertIn(key, result["schema"])

    def test_finite_differences_use_actual_dt_wrap_yaw_and_null_invalid_dt(self):
        cfg, summary, steps, rows = self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            dynamic = self.project(tmp, cfg, summary, steps, rows)["dynamic"]
        self.assertEqual(dynamic["object_velocity"][0], [None, None, None])
        self.assertEqual(dynamic["robot_vel"][0], [None, None])
        np.testing.assert_allclose(dynamic["object_velocity"][1], [.1, .2, 1.0], atol=1e-12)
        np.testing.assert_allclose(dynamic["object_velocity"][2], [.2, -.1, .2], atol=1e-12)
        np.testing.assert_allclose(dynamic["robot_vel"][1], [.1, -.05], atol=1e-12)
        np.testing.assert_allclose(dynamic["robot_vel"][2], [.2, .05], atol=1e-12)
        np.testing.assert_allclose(dynamic["qvel"][1][-4:], [.1, .2, 1.0, .02], atol=1e-12)
        np.testing.assert_allclose(dynamic["qvel"][2][-4:], [.2, -.1, .2, .02], atol=1e-12)
        self.assertEqual(dynamic["qvel"][0], steps[0]["robot_v"] + [None] * 4)
        for index in (3, 4):
            self.assertEqual(dynamic["object_velocity"][index], [None] * 3)
            self.assertEqual(dynamic["robot_vel"][index], [None] * 2)
            self.assertEqual(dynamic["qvel"][index], steps[index]["robot_v"] + [None] * 4)

    def test_measured_efforts_are_separate_from_unrecorded_controls(self):
        cfg, summary, steps, rows = self.fixture()
        del steps[2]["robot_v"]
        del steps[2]["robot_u"]
        with tempfile.TemporaryDirectory() as tmp:
            dynamic = self.project(tmp, cfg, summary, steps, rows)["dynamic"]
        self.assertEqual(dynamic["robot_control"], [[None] * 5 for _ in range(4)])
        self.assertEqual(dynamic["compute_time"], [None] * 4)
        self.assertEqual(dynamic["contact_normal_force_z"], [None] * 4)
        self.assertEqual(dynamic["robot_contact_force"], [None] * 4)
        self.assertEqual(dynamic["robot_joint_effort"][0], steps[0]["robot_u"])
        self.assertEqual(dynamic["robot_joint_effort"][2], [None] * 5)
        self.assertEqual(dynamic["qvel"][2][:5], [None] * 5)

    def test_single_logged_state_has_no_invented_intervals(self):
        cfg, summary, steps, rows = self.fixture()
        summary["n_control_steps"] = 1
        summary["sim_time_end"] = rows[0]["sim_time"]
        summary["final_position_error"] = rows[0]["position_error_m"]
        summary["final_orientation_error"] = rows[0]["orientation_error_rad"]
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps[:1], rows[:1])
        self.assertEqual(result["steps_run"], 0)
        self.assertEqual(result["dynamic"]["time"], [10.0])
        self.assertEqual(result["dynamic"]["object_velocity"], [[None] * 3])
        for key in ("robot_control", "compute_time", "contact_normal_force_z", "robot_contact_force",
                    "tip_z", "tip_tilt"):
            self.assertEqual(result["dynamic"][key], [], key)

    def test_pairs_by_control_step_and_rejects_cross_run_or_mismatched_rows(self):
        cfg, summary, steps, rows = self.fixture()
        extra = {"control_step": 999, "sim_time": 99, "robot_q": [0] * 5,
                 "objects": {"OBJECT_other_STATE_SIMULATION": [1, 0, 0, 0, 0, 0, 0]}}
        with tempfile.TemporaryDirectory() as tmp:
            baseline = self.project(tmp, cfg, summary, steps, rows)
            shuffled = self.project(tmp, cfg, summary, [extra, *reversed(steps)], rows)
            for key in ("time", "qpos", "qvel", "object_pose_3d"):
                self.assertEqual(shuffled["dynamic"][key], baseline["dynamic"][key])
            cases = []
            for field, value in (("run_id", "other_run"), ("scenario", "shelf_gap"),
                                 ("control_step", 999), ("sim_time", 99.0), ("goal_x", 999.0)):
                bad_rows = deepcopy(rows)
                bad_rows[1][field] = value
                cases.append((field, steps, bad_rows))
            cases.append(("missing raw step", steps[1:], rows))
            for label, bad_steps, bad_rows in cases:
                with self.subTest(case=label), self.assertRaises(ValueError):
                    self.project(tmp, cfg, summary, bad_steps, bad_rows)

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
            (out / f"{run_id}_eval_metrics.png").write_bytes(b"existing plot must be untouched")
            (out / "video.mp4").write_bytes(b"existing video must be untouched")
            (out / "recorder.log").write_text('FINAL {"pos_tol": 99, "ang_tol": 99}\n')
            before = {path.relative_to(out): (path.read_bytes(), path.stat().st_mtime_ns)
                      for path in out.rglob("*") if path.is_file() and path != result_file}
            argv = ["postprocess_run.py", "--run-dir", str(out), "--scene", "open_task",
                    "--run-id", run_id, "--scene-config", str(config), "--export-only"]
            with patch("sys.argv", argv), patch.object(P, "build_fk", side_effect=AssertionError("FK forbidden")) as fk, \
                    patch("matplotlib.pyplot.subplots", side_effect=AssertionError("Plotting forbidden")), \
                    redirect_stdout(io.StringIO()):
                P.main()
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

    def test_metadata_resolves_relocated_docker_snapshot_and_keeps_native_parameters(self):
        cfg, summary, steps, rows = self.fixture()
        docker_config = "/home/dairlib/dairlib/results/synthetic/config"
        native_options = {"N": 7, "admm_iter": 9, "rho_scale": 3,
                          "planning_dt_position": .125, "q_vector": [1, 2, 3]}
        sampling = {"sampling_strategy": 8, "num_additional_samples_repos": 3,
                    "num_additional_samples_c3": 4, "z_height": -.012}
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            controller = {"sampling_c3_options_file": f"{docker_config}/repository/profiles/options.yaml",
                          "sampling_params_file": f"{docker_config}/repository/profiles/sampling.yaml",
                          "sim_params_file": f"{docker_config}/simulation.yaml",
                          "goal_params_file": f"{docker_config}/goal.yaml",
                          "reposition_params_file": f"{docker_config}/repository/profiles/reposition.yaml"}
            mappings = {"config/controller.yaml": controller,
                        "config/simulation.yaml": {"dt": .002, "q_init_franka": [0] * 5},
                        "config/goal.yaml": {"position_success_threshold": .02,
                                             "orientation_success_threshold": .1},
                        "config/repository/profiles/options.yaml": native_options,
                        "config/repository/profiles/sampling.yaml": sampling,
                        "config/repository/profiles/reposition.yaml": {"saved_parameter": 123}}
            for relative, mapping in mappings.items():
                path = out / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(yaml.safe_dump(mapping))
            (out / "runtime_status.json").write_text(json.dumps({
                "run_id": summary["run_id"], "object_name": "banana", "seed": 123,
                "start": 2, "goal_index": 3, "obstacle_cost": "relu", "wall_cap_seconds": 321,
                "controller_params_file": f"{docker_config}/controller.yaml",
                "object_profile": {"physics": {"mass_kg": .1}},
                "sampler_settings": {"SAMPLING_C3_SEED": "123"}}))
            before = {path.relative_to(out): path.read_bytes() for path in out.rglob("*") if path.is_file()}
            result = self.project(out, cfg, summary, steps, rows, pos_tol=.0123, ang_tol=.0456)
            self.assertEqual({path.relative_to(out): path.read_bytes()
                              for path in out.rglob("*") if path.is_file()}, before)
            hyper = result["hyperparameters"]
            self.assertEqual(hyper["horizon"], 7)
            self.assertEqual(hyper["n_admm"], 9)
            self.assertEqual(hyper["goal_pos_tol"], .0123)
            self.assertEqual(hyper["goal_theta_tol"], .0456)
            self.assertIsNone(hyper["control_dt"])
            self.assertEqual(hyper["c3plus"]["configurations"]["config/repository/profiles/options.yaml"],
                             native_options)
            self.assertEqual(hyper["c3plus"]["configurations"]["config/repository/profiles/sampling.yaml"],
                             sampling)
            self.assertEqual(hyper["c3plus"]["configurations"]["config/repository/profiles/reposition.yaml"],
                             {"saved_parameter": 123})
            self.assertEqual(hyper["c3plus"]["sampler_environment"], {"SAMPLING_C3_SEED": "123"})
            self.assertEqual(result["run"]["seed"], 123)
            self.assertEqual(result["run"]["start_index"], "2")
            self.assertEqual(result["run"]["goal_index"], "3")
            self.assertEqual(result["static"]["sim_timestep"], .002)
            sources = {source["path"]: source["sha256"]
                       for source in result["provenance"]["metadata_sources"]}
            self.assertEqual(sources, {str(path): hashlib.sha256(data).hexdigest()
                                       for path, data in before.items()})

    def test_missing_recorded_metadata_stays_unknown(self):
        cfg, summary, steps, rows = self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            # This valid repository path is deliberately not snapshotted. The
            # exporter must not fill historical fields using today's tuning.
            controller = out / "config/controller.yaml"
            controller.parent.mkdir()
            controller.write_text(yaml.safe_dump({"sampling_c3_options_file":
                str(S.REPO / "examples/sampling_c3/shared_parameters/profiles/mesh_objects/sampling_c3plus_options.yaml")}))
            result = self.project(out, cfg, summary, steps, rows, pos_tol=None, ang_tol=None)
        hyper = result["hyperparameters"]
        for key in ("horizon", "n_admm", "rho", "rho_torque", "temperature", "samples",
                    "iterations", "goal_pos_tol", "goal_theta_tol", "costs"):
            self.assertIsNone(hyper[key], key)
        self.assertIsNone(result["run"]["seed"])
        self.assertIsNone(result["static"]["sim_timestep"])
        self.assertIsNone(result["static"]["object_limit_surface_d"])
        self.assertIsNone(result["static"]["object_wrench_limit"])
        self.assertTrue(result["provenance"]["missing_optional_metadata"])


if __name__ == "__main__":
    unittest.main()
