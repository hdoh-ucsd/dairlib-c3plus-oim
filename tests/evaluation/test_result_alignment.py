from copy import deepcopy
import tempfile
import unittest
import numpy as np


from tests.fixtures.results import ProjectionFixtures

class ResultProjectionTests(ProjectionFixtures, unittest.TestCase):
    def test_physical_execution_uses_exact_states_and_preserves_all_snapshots(self):
        cfg, summary, steps, rows = self.fixture()
        native, updates = self.physical_fixture()
        with tempfile.TemporaryDirectory() as tmp:
            before = self.project(tmp, cfg, summary, steps, rows)
            after = self.project(tmp, cfg, summary, steps, rows,
                                 execution_native=native, planning_updates=updates)
        self.assertEqual(after["recording"]["snapshot_dynamic"], before["dynamic"])
        self.assertEqual(after["recording"]["n_snapshots"], 5)
        self.assertEqual(after["planning"]["n_updates"], 4)
        self.assertEqual(after["execution"]["n_steps_executed"], 2)
        self.assertEqual(after["execution"]["mode"], ["reposition", "c3"])
        self.assertEqual(after["dynamic"]["object_pose"][0], [.2, -.4, 0.0])
        self.assertEqual(after["dynamic"]["time"], [7.1, 7.2, 7.35])
        self.assertEqual(len(after["dynamic"]["object_pose"]), 3)
        self.assertEqual(after["dynamic"]["compute_time"], after["execution"]["step_wall_time"])
        self.assertAlmostEqual(after["execution"]["frequency_hz"], 25.0)
        self.assertEqual(after["hyperparameters"]["steps"], 2)
        self.assertIsNone(after["hyperparameters"]["control_dt"])
        for key in ("success", "t_success", "steps_run", "evaluation", "native_controller"):
            self.assertEqual(after[key], before[key])


    def test_physical_execution_rejects_unproven_or_inconsistent_alignment(self):
        cfg, summary, steps, rows = self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            for mutation in ("missing_terminal", "duplicate_wall", "duplicate_sim", "unknown_policy", "duplicate_plan", "budget"):
                native, updates = self.physical_fixture()
                if mutation == "missing_terminal": native["terminals"] = []
                if mutation == "duplicate_wall": native["boundaries"][1]["wall_time"] = 0
                if mutation == "duplicate_sim": native["boundaries"][1]["sim_time"] = 7.1
                if mutation == "unknown_policy": native["boundaries"][1]["plan_utime"] = 999
                if mutation == "duplicate_plan": updates[-1]["utime"] = updates[0]["utime"]
                if mutation == "budget": native["headers"][0]["step_budget"] = 1
                with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                    self.project(tmp, cfg, summary, steps, rows,
                                 execution_native=native, planning_updates=updates)


    def test_physical_execution_unlimited_budget_stays_null(self):
        cfg, summary, steps, rows = self.fixture()
        native, updates = self.physical_fixture()
        native["headers"][0]["step_budget"] = None
        native["terminals"][0]["reason"] = "shutdown"
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows,
                                  execution_native=native, planning_updates=updates)
        self.assertIsNone(result["execution"]["step_budget"])
        self.assertIsNone(result["hyperparameters"]["steps"])


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
        self.assertEqual(result["n_snapshots"], 5)
        self.assertEqual(result["n_recorded_intervals"], 4)
        for key in ("n_control_steps", "steps_run", "success", "t_success", "first_success_t"):
            self.assertIn(key, result["schema"]["legacy_fields"])
        dynamic = result["dynamic"]
        for key in ("time", "object_pose", "object_velocity", "robot_pos", "robot_vel",
                    "qpos", "qvel", "object_pose_3d", "tip_z_state", "tip_tilt_state",
                    "robot_joint_effort"):
            self.assertEqual(len(dynamic[key]), 5, key)
        for key in ("robot_control", "contact_normal_force_z", "robot_contact_force",
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
        self.assertNotIn("compute_time", dynamic)
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
        self.assertIsNone(result["hyperparameters"]["control_dt"])
        self.assertEqual(result["dynamic"]["time"], [10.0])
        self.assertEqual(result["dynamic"]["object_velocity"], [[None] * 3])
        for key in ("robot_control", "contact_normal_force_z", "robot_contact_force",
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
