import math
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from c3plus import configs as S
from c3plus.experiments import run as R

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_checkout_paths(self):
        self.assertEqual(S.REPO, Path(__file__).resolve().parents[2])


    def test_controller_goal_is_selected_demo_goal(self):
        plan = R.plan_run("open_task", "exponential", 2, 3, "/unused/test")
        self.assertEqual(plan["demo"], "matched_open_table_xarm6_s2g3")
        self.assertEqual(plan["controller_goal"], S.load_controller_goal(S.demo_name("open_task", 3, 3))[1])
        self.assertEqual(plan["evaluation_goal"], plan["controller_goal"])
        with self.assertRaisesRegex(ValueError, "does not match"):
            R.plan_run("open_task", "exponential", 1, 1, "/unused/test", goal_pose=[0, 0, 0])


    def test_yaw_override_validation_preserves_indexed_position(self):
        base = dict(scene="single_obstacle", obstacle_cost="relu", start=2, goal=2, out="/unused/yaw")
        original = R.plan_run(**base)
        self.assertNotIn("goal_yaw_degrees", original)
        self.assertEqual(original["run_id"], "relu_single_obstacle_s02g02_seed42")
        for value in (45, 180, float("nan"), float("inf"), True, "90"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                R.plan_run(**base, goal_yaw_degrees=value)
        for value in (90, 0, -90):
            changed = R.plan_run(**base, goal_yaw_degrees=value)
            self.assertEqual(changed["controller_goal"][:2], original["controller_goal"][:2])
            self.assertAlmostEqual(changed["controller_goal"][2], math.radians(value))
            with self.assertRaisesRegex(ValueError, "does not match"):
                R.plan_run(**base, goal_yaw_degrees=value, goal_pose=[0, 0, math.radians(value)])


    def test_settings_isolated(self):
        with patch.dict(R.os.environ, {"SAMPLING_C3_UNKNOWN": "old",
                                      "SAMPLING_C3_RANK_OBS_MODE": "old"}):
            exponential = R.environment("exponential")
            relu = R.environment("relu")
        self.assertNotIn("SAMPLING_C3_UNKNOWN", exponential)
        self.assertNotIn("SAMPLING_C3_RANK_OBS_MODE", exponential)
        self.assertEqual(exponential["SAMPLING_C3_SEED"], "42")
        self.assertEqual(relu["SAMPLING_C3_OBS_RELU_W"], "200")


    def test_all_configs_and_goals(self):
        for scene in R.SCENES:
            config = R.yaml.safe_load((S.CONFIG_DIR / f"{scene}.yaml").read_text())
            S.planner_environment(config)
            for m in range(1, 6):
                for n in range(1, 6):
                    demo_name = S.demo_name(scene, m, n)
                    configs = S.load_demo_configs(demo_name)
                    source, pose = S.load_controller_goal(demo_name)
                    self.assertIn(source, configs)
                    self.assertEqual(source.name, "experiments.yaml")
                    resolved = S.compose_demo_configs(demo_name)
                    self.assertEqual(resolved["goal"]["fixed_target_position"][:2], list(pose[:2]))
                    self.assertEqual(len(S.load_controller_goal(demo_name)[1]), 3)


    def test_shared_demo_config_graph_rejects_missing_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            demo = "matched_single_obstacle_xarm6_t2"
            self.copy_demo_configs(demo, repo)
            configs = S.load_demo_configs(demo, repo=repo)
            nested = next(path for path in configs if path.name == "progress_params_c3plus.yaml")
            nested.unlink()
            with self.assertRaises(FileNotFoundError):
                S.load_demo_configs(demo, repo=repo)


    def test_start_goal_and_orientation_compose_independently(self):
        for scene in S.SCENES:
            original = S.compose_demo_configs(S.demo_name(scene, 2, 2))
            other_start = S.compose_demo_configs(S.demo_name(scene, 3, 2))
            other_goal = S.compose_demo_configs(S.demo_name(scene, 2, 3))
            self.assertEqual(original["goal"], other_start["goal"])
            self.assertEqual(original["simulation"], other_goal["simulation"])
            self.assertNotEqual(original["simulation"]["q_init_object"][4:],
                                other_start["simulation"]["q_init_object"][4:])
            self.assertNotEqual(original["goal"]["fixed_target_position"],
                                other_goal["goal"]["fixed_target_position"])
            for yaw in (90, 0, -90):
                rotated = S.compose_demo_configs(S.demo_name(scene, 2, 2), goal_yaw_degrees=yaw)
                self.assertEqual(rotated["simulation"], original["simulation"])
                self.assertEqual(rotated["controller"], original["controller"])
                goal = dict(rotated["goal"])
                quaternion = goal.pop("fixed_target_orientation")
                self.assertEqual(goal.pop("fixed_target_orientations"), [quaternion])
                expected = {k: v for k, v in original["goal"].items()
                            if k not in ("fixed_target_orientation", "fixed_target_orientations")}
                self.assertEqual(goal, expected)
                self.assertAlmostEqual(2 * math.atan2(quaternion[3], quaternion[0]), math.radians(yaw))


    def test_composition_preserves_scene_exceptions(self):
        def resolve(scene, start=2, goal=2):
            return S.compose_demo_configs(S.demo_name(scene, start, goal))
        self.assertTrue(resolve("open_task", 1, 1)["simulation"]["visualize_drake_sim"])
        self.assertFalse(resolve("open_task", 1, 2)["simulation"]["visualize_drake_sim"])
        self.assertEqual(resolve("slalom", 4)["simulation"]["q_init_object"][4:], [0.361, 0.37, 0.0008])
        self.assertEqual(resolve("slalom", goal=4)["goal"]["fixed_target_position"], [0.367, -0.38, 0.0008])
        self.assertEqual(resolve("shelf_gap", goal=3)["goal"]["fixed_target_position"], [0.363, -0.41, 0.0008])
        self.assertEqual(resolve("ycb_clutter")["goal"]["fixed_target_orientation"], [-0.049184, 0, 0, 0.99879])
        self.assertEqual(resolve("single_obstacle")["goal"]["fixed_target_orientation"], [-0.0491838, 0, 0, 0.99879])
        self.assertEqual(resolve("icra_sign", 4)["simulation"]["q_init_franka"],
                         [0.972422, 0.310762, -1.230368, 0.06679, 0.916921])


    def test_resolved_yaml_snapshot_is_complete_and_independent_of_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "source checkout"
            demo = S.demo_name("shelf_gap", 2, 2)
            self.copy_demo_configs(demo, repo)
            out = Path(tmp) / "saved run"
            controller_path = S.write_demo_configs(demo, out, repo=repo, goal_yaw_degrees=-90)
            def resolve(value):
                if isinstance(value, dict):
                    return {k: resolve(v) for k, v in value.items()}
                if isinstance(value, list):
                    return [resolve(v) for v in value]
                if isinstance(value, str) and value.endswith(".yaml"):
                    path = Path(value)
                    self.assertTrue(path.is_absolute())
                    self.assertTrue(path.is_relative_to(out))
                    return resolve(R.yaml.safe_load(path.read_text()))
                return value
            controller = R.yaml.safe_load(controller_path.read_text())
            resolved = resolve(controller)
            expected = S.compose_demo_configs(demo, repo=repo, goal_yaw_degrees=-90)
            self.assertEqual(resolved["goal_params_file"], expected["goal"])
            self.assertEqual(resolved["sim_params_file"], expected["simulation"])
            shutil.rmtree(repo)
            self.assertEqual(resolve(controller), resolved)
            with self.assertRaises(FileExistsError):
                S.write_demo_configs(demo, out)


    def test_configuration_digest_tracks_only_selected_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            demo = S.demo_name("single_obstacle", 2, 2)
            self.copy_demo_configs(demo, repo)
            source = repo / S.EXPERIMENTS_FILE
            data = R.yaml.safe_load(source.read_text())
            original = S.demo_config_digest(demo, repo=repo)
            data["start_positions"]["icra_sign_s01"][0] += 0.01
            source.write_text(R.yaml.safe_dump(data))
            self.assertEqual(S.demo_config_digest(demo, repo=repo), original)
            data["start_positions"]["t_shape_s02"][0] += 0.01
            source.write_text(R.yaml.safe_dump(data))
            changed = S.demo_config_digest(demo, repo=repo)
            self.assertNotEqual(changed, original)
            dependency = next(path for path in S.load_demo_configs(demo, repo=repo)
                              if path.name == "progress_params_c3plus.yaml")
            progress = R.yaml.safe_load(dependency.read_text())
            progress["num_control_loops_to_wait"] += 1
            dependency.write_text(R.yaml.safe_dump(progress))
            self.assertNotEqual(S.demo_config_digest(demo, repo=repo), changed)


    def test_catalogue_orientation_selection_and_invalid_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            demo = S.demo_name("single_obstacle", 2, 2)
            self.copy_demo_configs(demo, repo)
            source = repo / S.EXPERIMENTS_FILE
            data = R.yaml.safe_load(source.read_text())
            data["scenes"]["single_obstacle"]["start_orientations"] = {2: "start_s01"}
            source.write_text(R.yaml.safe_dump(data))
            resolved = S.compose_demo_configs(demo, repo=repo)
            self.assertEqual(resolved["simulation"]["q_init_object"], [1, 0, 0, 0, 0.366, 0.431, 0.0008])
            self.assertEqual(resolved["goal"]["fixed_target_position"], [0.397, -0.431, 0.0008])
            for invalid in ([0, 0, 0, 0], [1, 0], [float("nan"), 0, 0, 0]):
                data["orientations"]["start_s01"] = invalid
                source.write_text(R.yaml.safe_dump(data))
                with self.assertRaises(ValueError):
                    S.compose_demo_configs(demo, repo=repo)
            data["orientations"]["start_s01"] = [1, 0, 0, 0]
            data["defaults"]["goal"]["goal_mode"] = 1
            source.write_text(R.yaml.safe_dump(data))
            with self.assertRaisesRegex(ValueError, "fixed"):
                S.compose_demo_configs(demo, repo=repo, goal_yaw_degrees=90)
