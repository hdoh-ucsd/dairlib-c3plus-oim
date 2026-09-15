from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np


from tests.fixtures.results import ProjectionFixtures
from c3plus.evaluation import exporter as P

class ResultProjectionTests(ProjectionFixtures, unittest.TestCase):
    def test_native_goal_stop_marks_success_and_keeps_exact_terminal_pose(self):
        cfg, summary, steps, rows = self.fixture()
        native, updates = self.physical_fixture()
        native['headers'][0].update(goal=cfg['goal'], goal_pos_tol=.05, goal_ang_tol=.1)
        native['terminals'][0]['reason'] = 'goal_reached'
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows,
                                  execution_native=native, planning_updates=updates)
        self.assertTrue(result['success'])
        self.assertEqual(result['t_success'], 7.35)
        self.assertEqual(result['first_success_t'], 7.35)
        self.assertEqual(result['execution']['terminal_reason'], 'goal_reached')
        self.assertEqual(result['dynamic']['object_pose'][-1], cfg['goal'])
        self.assertEqual(result['dynamic']['object_pose_3d'][-1],
                         native['terminals'][0]['objects'][0]['q'])
        bad = deepcopy(native)
        bad['terminals'][0]['objects'][0]['q'][4] = 5
        with tempfile.TemporaryDirectory() as tmp, self.assertRaisesRegex(ValueError, 'goal stop'):
            self.project(tmp, cfg, summary, steps, rows, execution_native=bad, planning_updates=updates)

    def test_execution_quaternion_drift_only_normalizes_derived_yaw(self):
        cfg, summary, steps, rows = self.fixture()
        for scale in (1.000059, .999941, -1.000059, -.999941):
            with self.subTest(scale=scale), tempfile.TemporaryDirectory() as tmp:
                native, updates = self.physical_fixture()
                states = [*native["boundaries"], *native["terminals"]]
                angles = [.4, -.7, 1.2]
                for state, angle in zip(states, angles):
                    state["objects"][0]["q"][:4] = [
                        math.cos(angle / 2) * scale, 0, 0,
                        math.sin(angle / 2) * scale]
                    state["objects"][0]["v"] = [.1, -.2, .3, -.4, .5, -.6]
                original = deepcopy((native, updates, cfg, summary, steps, rows))
                snapshots = self.project(tmp, cfg, summary, steps, rows)
                result = self.project(tmp, cfg, summary, steps, rows,
                                      execution_native=native, planning_updates=updates)
                self.assertEqual((native, updates, cfg, summary, steps, rows), original)
                self.assertEqual(result["recording"]["execution_native"], native)
                self.assertEqual(result["recording"]["snapshot_dynamic"], snapshots["dynamic"])
                self.assertEqual(result["dynamic"]["object_pose_3d"],
                                 [state["objects"][0]["q"] for state in states])
                self.assertEqual(result["dynamic"]["object_spatial_velocity"],
                                 [state["objects"][0]["v"] for state in states])
                self.assertEqual(result["dynamic"]["robot_joint_positions"],
                                 [state["robot_q"] for state in states])
                self.assertEqual(result["dynamic"]["robot_joint_velocities"],
                                 [state["robot_v"] for state in states])
                np.testing.assert_allclose(
                    [pose[2] for pose in result["dynamic"]["object_pose"]], angles, atol=1e-14)
                self.assertEqual(result["dynamic"]["time"], [7.1, 7.2, 7.35])
                self.assertEqual(result["execution"]["wall_time"], [0.0, .035, .08])
                self.assertEqual(result["dynamic"]["compute_time"], [.035, .08 - .035])
                self.assertEqual(result["execution"]["frequency_hz"], 25.0)
                for key in ("success", "t_success", "steps_run", "evaluation", "native_controller"):
                    self.assertEqual(result[key], snapshots[key], key)


    def test_previously_accepted_quaternion_drift_preserves_exact_legacy_yaw(self):
        cfg, summary, steps, rows = self.fixture()
        native, updates = self.physical_fixture()
        scale, angle = 1.000002, 1.2
        q = [math.cos(angle / 2) * scale, 0, 0, math.sin(angle / 2) * scale]
        self.assertTrue(math.isclose(sum(x*x for x in q), 1.0, abs_tol=1e-5))
        expected = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                              1 - 2 * (q[2] * q[2] + q[3] * q[3]))
        self.assertNotEqual(expected, angle)
        for state in [*native["boundaries"], *native["terminals"]]:
            state["objects"][0]["q"][:4] = q
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows,
                                  execution_native=native, planning_updates=updates)
        self.assertEqual([pose[2] for pose in result["dynamic"]["object_pose"]], [expected] * 3)


    def test_execution_quaternion_drift_fix_still_rejects_invalid_states(self):
        cfg, summary, steps, rows = self.fixture()
        invalid = {
            "zero quaternion": [0, 0, 0, 0, .2, -.4, -.02],
            "nan quaternion": [float("nan"), 0, 0, 0, .2, -.4, -.02],
            "infinite quaternion": [float("inf"), 0, 0, 0, .2, -.4, -.02],
            "nonfinite translation": [1, 0, 0, 0, float("inf"), -.4, -.02],
            "truncated state": [1, 0, 0, 0, .2, -.4],
            "non-list state": None,
            "boolean quaternion": [True, 0, 0, 0, .2, -.4, -.02],
            "string quaternion": ["1", 0, 0, 0, .2, -.4, -.02],
        }
        with tempfile.TemporaryDirectory() as tmp:
            for label, q in invalid.items():
                native, updates = self.physical_fixture()
                native["boundaries"][0]["objects"][0]["q"] = q
                with self.subTest(state=label), self.assertRaises(ValueError):
                    self.project(tmp, cfg, summary, steps, rows,
                                 execution_native=native, planning_updates=updates)


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
        self.assertAlmostEqual(after["hyperparameters"]["control_dt"], .125)
        self.assertEqual(after["hyperparameters"]["control_dt_source"], "mean_physical_execution_interval")
        for key in ("success", "t_success", "steps_run", "evaluation", "native_controller"):
            self.assertEqual(after[key], before[key])


    def test_execution_control_dt_uses_mean_simulation_interval_not_wall_time(self):
        cfg, summary, steps, rows = self.fixture()
        native, updates = self.physical_fixture()
        # Deliberately different, nonuniform simulation and wall intervals.
        native["boundaries"][0]["sim_time"] = 12.0
        native["boundaries"][1]["sim_time"] = 12.4
        native["terminals"][0]["sim_time"] = 14.0
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows,
                                  execution_native=native, planning_updates=updates)
        self.assertEqual(result["hyperparameters"]["control_dt"], 1.0)
        self.assertEqual(result["execution"]["sim_time"], [12.0, 12.4, 14.0])
        self.assertEqual(result["dynamic"]["time"], result["execution"]["sim_time"])
        self.assertAlmostEqual(result["execution"]["frequency_hz"], 25.0)
        self.assertEqual(result["dynamic"]["compute_time"], [.035, .08-.035])
        self.assertIsNone(result["planning"]["solve_time_s"])


    def test_one_applied_policy_control_dt_includes_terminal_interval(self):
        cfg, summary, steps, rows = self.fixture()
        native, updates = self.physical_fixture()
        native["headers"][0]["step_budget"] = 1
        native["boundaries"] = native["boundaries"][:1]
        native["terminals"][0].update(boundary_step=1, plan_utime=updates[0]["utime"])
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows,
                                  execution_native=native, planning_updates=updates)
        self.assertEqual(result["execution"]["n_steps_executed"], 1)
        self.assertEqual(result["dynamic"]["time"], [7.1, 7.35])
        self.assertAlmostEqual(result["hyperparameters"]["control_dt"], .25)
        self.assertAlmostEqual(result["execution"]["frequency_hz"], 12.5)


    def test_zero_execution_helper_retains_unavailable_control_dt(self):
        result = {"execution": {"alignment": "physical_policy_boundaries_v1",
                                "n_steps_executed": 0, "sim_time": [7.1]},
                  "dynamic": {"time": [7.1]}, "hyperparameters": {},
                  "schema": {"semantics_version": 4}, "provenance": {}}
        recorded = deepcopy(result["execution"])
        P.add_execution_control_dt(result)
        self.assertIsNone(result["hyperparameters"]["control_dt"])
        self.assertEqual(result["hyperparameters"]["control_dt_source"], "mean_physical_execution_interval")
        self.assertEqual(result["execution"], recorded)
        self.assertEqual(result["dynamic"]["time"], [7.1])


    def test_serialized_native_timing_supports_external_evaluator_scalar_contract(self):
        cfg, summary, steps, rows = self.fixture()
        native, updates = self.physical_fixture()
        native["headers"][0]["step_budget"] = None
        native["terminals"][0]["reason"] = "shutdown"
        with tempfile.TemporaryDirectory() as tmp:
            result = self.project(tmp, cfg, summary, steps, rows,
                                  execution_native=native, planning_updates=updates)
            path = Path(tmp) / "result.json"
            P.write_result_json(path, result)
            saved = json.loads(path.read_text())
        hp, dynamic = saved["hyperparameters"], saved["dynamic"]
        n = len(dynamic["object_pose"]) - 1
        self.assertNotEqual(n, saved["steps_run"], "Legacy steps_run counts recorder intervals")
        self.assertNotEqual(dynamic["time"][1]-dynamic["time"][0],
                            dynamic["time"][2]-dynamic["time"][1])
        # Exercise the downloaded OIM evaluator's scalar-period conversion
        # without importing its external checkout or substituting snapshot N.
        control_dt = float(hp["control_dt"])
        self.assertTrue(math.isfinite(control_dt) and control_dt > 0)
        self.assertAlmostEqual(n*control_dt, dynamic["time"][-1]-dynamic["time"][0])
        self.assertIsNone(hp["steps"])
        self.assertEqual(int(hp.get("steps") or n), n)
        frequency = 1.0/(sum(dynamic["compute_time"])/len(dynamic["compute_time"]))
        self.assertAlmostEqual(frequency, saved["execution"]["frequency_hz"])
        self.assertNotAlmostEqual(frequency, 1.0/control_dt)


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
        self.assertAlmostEqual(result["hyperparameters"]["control_dt"], .125)
        self.assertIsNone(result["planning"]["solve_time_s"])
        snapshots = result["recording"]["snapshot_dynamic"]
        self.assertEqual(snapshots["object_velocity"][0], [None]*3)
        self.assertEqual(snapshots["robot_control"], [[None]*5]*4)


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
