from copy import deepcopy
import unittest

from c3plus.evaluation import exporter as P

from tests.fixtures.results import ProjectionFixtures

class ResultProjectionTests(ProjectionFixtures, unittest.TestCase):
    def test_control_dt_is_mean_observed_interval_without_resampling(self):
        for times, expected in (([10., 10.1, 10.4, 10.8, 11.], .25),
                                ([10., 10.2, 10.4, 10.6, 10.8], .2),
                                ([10., 10., 10.4, 10.8, 11.], None),
                                ([10., 10.1, 10.4, 10.3, 11.], None)):
            with self.subTest(times=times):
                result = self.semantic_result()
                result["dynamic"]["time"] = times
                original = deepcopy(result["dynamic"])
                P.add_result_semantics(result)
                self.assertEqual(result["dynamic"], original)
                actual = result["hyperparameters"]["control_dt"]
                self.assertEqual(result["hyperparameters"]["control_dt_source"], "mean_observed_state_interval")
                if expected is None:
                    self.assertIsNone(actual)
                else:
                    self.assertAlmostEqual(actual, expected)
                    self.assertAlmostEqual(result["steps_run"] * actual, times[-1] - times[0])


    def test_unavailable_timing_is_omitted_but_real_timing_is_preserved(self):
        result = self.semantic_result()
        result["dynamic"]["compute_time"] = [None] * result["steps_run"]
        P.add_result_semantics(result)
        self.assertNotIn("compute_time", result["dynamic"])
        self.assertNotIn("compute_time", result["schema"]["interval_arrays"])
        self.assertIn("compute_time", result["schema"]["missing"])
        timing = [.014, None, .019, .021]
        result["dynamic"]["compute_time"] = timing.copy()
        P.add_result_semantics(result)
        self.assertEqual(result["dynamic"]["compute_time"], timing)
        self.assertIn("compute_time", result["schema"]["interval_arrays"])


    def test_common_success_uses_simultaneous_strict_thresholds(self):
        cases = (
            # Equality at either threshold fails; only index 3 succeeds.
            ([.05, .04, .051, .049, .02], [.01, .1, .01, .099, .2], True, False, 10.4),
            # Separate position and orientation successes must not be combined.
            ([.04, .06, .04, .06, .04], [.2, .05, .2, .05, .2], False, False, None),
            # Preserve recorded order even when the last timestamp goes backward.
            ([.06, .06, .04, .04, .04], [.2, .2, .05, .05, .05], True, True, 10.4),
        )
        for position, orientation, ever, final, first_time in cases:
            with self.subTest(position=position, orientation=orientation):
                result = self.semantic_result()
                result["dynamic"]["position_error_m"] = position
                result["dynamic"]["orientation_error_rad"] = orientation
                result["hyperparameters"].update(goal_pos_tol=.05, goal_theta_tol=.1)
                result.update(success=True, t_success=10.4, first_success_t=10.5)
                before = deepcopy(result)
                self.assertIs(P.add_result_semantics(result), result)
                evaluation = result["evaluation"]
                self.assertEqual(evaluation["thresholds"], {"position_m": .05, "orientation_rad": .1})
                self.assertIs(evaluation["ever_success"], ever)
                self.assertIs(evaluation["final_success"], final)
                self.assertEqual(evaluation["first_success_t"], first_time)
                self.assertEqual(result["dynamic"], before["dynamic"])
                for key in ("success", "t_success", "first_success_t", "n_control_steps", "steps_run",
                            "best_pos_err", "best_ang_err", "final_position_error", "final_orientation_error"):
                    self.assertEqual(result[key], before[key], key)
                self.assertEqual(result["n_snapshots"], len(before["dynamic"]["time"]))
                self.assertEqual(result["n_recorded_intervals"], len(before["dynamic"]["time"]) - 1)


    def test_drill_common_success_does_not_invent_native_completion(self):
        result = self.semantic_result()
        result["dynamic"]["position_error_m"] = [.2, .06, .04, .029853535315141556, .02985378167664127]
        result["dynamic"]["orientation_error_rad"] = [.2, .2, .09, .09, .08950720751676533]
        configs = result["provenance"]["c3plus"].setdefault("configurations", {})
        configs["config/goal.yaml"] = {"position_success_threshold": .02,
                                       "orientation_success_threshold": .1,
                                       "other_saved_native_setting": 987}
        before = deepcopy(result)
        P.add_result_semantics(result)
        self.assertTrue(result["evaluation"]["ever_success"])
        self.assertTrue(result["evaluation"]["final_success"])
        self.assertGreater(min(result["dynamic"]["position_error_m"]), .02)
        self.assertIsNone(result["native_controller"]["success"])
        self.assertEqual(result["native_controller"]["success_thresholds"],
                         {"position_m": .02, "orientation_rad": .1})
        self.assertEqual(configs, before["provenance"]["c3plus"].get("configurations", {}))
        self.assertEqual(result["dynamic"], before["dynamic"])
        # A saved threshold change must be reflected, without a hardcoded 0.02.
        configs["config/goal.yaml"].update(position_success_threshold=.0137,
                                          orientation_success_threshold=.0678)
        P.add_result_semantics(result)
        self.assertEqual(result["native_controller"]["success_thresholds"],
                         {"position_m": .0137, "orientation_rad": .0678})
        self.assertEqual(result["evaluation"]["thresholds"], {"position_m": .05, "orientation_rad": .1})
        self.assertIsNone(result["native_controller"]["success"])


    def test_diagnostic_aliases_preserve_native_settings_and_unavailable_measurements(self):
        result = self.semantic_result()
        weights = {"q_pos": 3.0, "q_theta": 1.2, "w_obstacle": 4.5}
        result["evaluation"]["weights"] = weights
        configs = {"config/controller.yaml": {"native_only": True},
                   "config/options.yaml": {"Q": [1, 2], "R": [3], "admm_iter": 7}}
        result["provenance"]["c3plus"]["configurations"] = deepcopy(configs)
        before = deepcopy(result)
        P.add_result_semantics(result)
        evaluation = result["evaluation"]
        self.assertEqual(evaluation["weights"], weights)
        self.assertEqual(evaluation["costs"]["components"], before["dynamic"]["evaluation_costs"])
        self.assertEqual(evaluation["costs"]["total"], before["dynamic"]["evaluation_total"])
        self.assertNotIn("costs", result["hyperparameters"])
        self.assertEqual(result["provenance"]["c3plus"].setdefault("configurations", {}), configs)
        self.assertEqual(result["dynamic"], before["dynamic"])
        for key in ("robot_control", "contact_normal_force_z", "robot_contact_force"):
            self.assertEqual(result["dynamic"][key], before["dynamic"][key])
            self.assertIn(key, result["schema"]["missing"])
        self.assertEqual(result["dynamic"]["robot_control"], [[None] * 5] * 4)
        self.assertTrue(any(value is not None for row in result["dynamic"]["robot_joint_effort"] for value in row))
        self.assertIsNone(result["static"]["object_limit_surface_d"])
        self.assertIsNone(result["static"]["object_wrench_limit"])
        self.assertFalse(any("plan" in key or "wrench" in key or "consensus" in key
                             for key in result["dynamic"]))
        result["evaluation"]["weights"] = None
        result["provenance"]["c3plus"]["configurations"] = {}
        P.add_result_semantics(result)
        self.assertIsNone(result["evaluation"]["weights"])
        self.assertIsNone(result["native_controller"]["success"])
        self.assertEqual(result["native_controller"]["success_thresholds"],
                         {"position_m": None, "orientation_rad": None})
