"""Metric definitions at genuine execution boundaries, independent of a simulator."""

from copy import deepcopy
import math
import unittest

from c3plus.evaluation.metrics import aggregate_metrics, trial_metrics
from c3plus.evaluation.schema import EXECUTION_SEMANTICS_VERSION


def run_fixture():
    return {
        "schema": {"version": "c3plus-reference-projection-v1",
                   "semantics_version": EXECUTION_SEMANTICS_VERSION},
        "run": {"algorithm": "c3plus", "task": "open_table", "object": "T_shape",
                "run_id": "one", "start_index": "2", "goal_index": "2", "seed": 42},
        "hyperparameters": {"steps": 10, "goal_pos_tol": .05, "goal_theta_tol": .1,
                            "object": "T_shape", "horizon": 5, "n_admm": 3,
                            "obstacle_cost": "exponential", "control_dt": 999},
        "static": {"goal": [0, 0, 0]},
        "dynamic": {"time": [10, 11, 13, 16],
                    "object_pose": [[0, 0, 0], [.2, 0, .3], [.03, 0, .02], [5, 0, 1]],
                    "compute_time": [.1, .2, .4]},
        "execution": {"alignment": "physical_policy_boundaries_v1",
                      "n_steps_executed": 3, "step_budget": 10,
                      "sim_time": [10, 11, 13, 16], "wall_time": [0, .1, .3, .7],
                      "step_wall_time": [.1, .2, .4], "frequency_hz": 3/.7},
        "evaluation": {"thresholds": {"position_m": .05, "orientation_rad": .1},
                       "ever_success": False},
        "planning": {"n_updates": 17}, "recording": {"n_snapshots": 25},
    }


class TrialMetricsTests(unittest.TestCase):
    def test_first_crossing_excludes_initial_and_later_settling(self):
        run = run_fixture()
        before = deepcopy(run)
        result = trial_metrics(run)
        self.assertEqual(run, before)
        self.assertTrue(result["success"])
        self.assertEqual(result["steps_to_goal"], 2)
        self.assertEqual(result["execution_time"], 3)
        self.assertAlmostEqual(result["eps_d"], .03)
        self.assertAlmostEqual(result["eps_o"], .02)
        self.assertAlmostEqual(result["trajectory_mean_position_error"], .115)
        self.assertAlmostEqual(result["trajectory_mean_orientation_error"], .16)
        self.assertAlmostEqual(result["theta"], .16)
        self.assertAlmostEqual(result["frequency_hz"], 3/.7)
        self.assertEqual(result["execution_steps_recorded"], 3)
        self.assertEqual(result["planning_updates"], 17)
        self.assertEqual(result["recorder_snapshots"], 25)

    def test_simultaneous_strict_tolerances_not_separate_crossings(self):
        run = run_fixture()
        run["dynamic"]["object_pose"] = [[0, 0, 0], [.05, 0, 0], [.01, 0, .3], [.2, 0, .01]]
        result = trial_metrics(run)
        self.assertFalse(result["success"])
        self.assertEqual(result["steps_to_goal"], 10)
        self.assertEqual(result["execution_time"], 6)
        self.assertAlmostEqual(result["eps_d"], .2)
        self.assertAlmostEqual(result["eps_o"], .01)
        self.assertAlmostEqual(result["trajectory_mean_position_error"], (.05+.01+.2)/3)
        self.assertAlmostEqual(result["trajectory_mean_orientation_error"], (.0+.3+.01)/3)
        self.assertEqual(result["theta"], result["trajectory_mean_orientation_error"])

    def test_exact_yaw_threshold_is_not_success(self):
        run = run_fixture()
        run["hyperparameters"]["goal_theta_tol"] = .125
        run["evaluation"]["thresholds"]["orientation_rad"] = .125
        run["dynamic"]["object_pose"] = [[0, 0, 0]] + [[0, 0, .125]] * 3
        self.assertFalse(trial_metrics(run)["success"])

    def test_yaw_is_wrapped_and_position_is_euclidean(self):
        run = run_fixture()
        run["static"]["goal"] = [1, 2, math.pi-.01]
        run["dynamic"]["object_pose"][1] = [1.03, 2.02, -math.pi+.02]
        result = trial_metrics(run)
        self.assertEqual(result["steps_to_goal"], 1)
        self.assertAlmostEqual(result["eps_d"], math.hypot(.03, .02))
        self.assertAlmostEqual(result["eps_o"], .03)
        self.assertAlmostEqual(result["theta"], .03)

    def test_failure_without_budget_never_substitutes_recorded_steps(self):
        run = run_fixture()
        run["dynamic"]["object_pose"][2] = [1, 0, 0]
        run["execution"]["step_budget"] = run["hyperparameters"]["steps"] = None
        result = trial_metrics(run)
        self.assertFalse(result["success"])
        self.assertIsNone(result["steps_to_goal"])
        self.assertIsNone(result["execution_step_budget"])
        self.assertEqual(result["execution_time"], 6)
        self.assertEqual(result["eps_d"], 5)
        self.assertEqual(result["eps_o"], 1)
        self.assertAlmostEqual(result["trajectory_mean_position_error"], (.2+1+5)/3)
        self.assertAlmostEqual(result["trajectory_mean_orientation_error"], (.3+0+1)/3)
        self.assertIn("no configured execution-step budget", " ".join(result["warnings"]))

    def test_missing_frequency_never_uses_dt_or_stored_frequency(self):
        run = run_fixture()
        for key in ("wall_time", "step_wall_time"):
            del run["execution"][key]
        del run["dynamic"]["compute_time"]
        result = trial_metrics(run)
        self.assertIsNone(result["frequency_hz"])
        self.assertEqual(result["steps_to_goal"], 2)
        self.assertTrue(result["warnings"])

    def test_documented_compute_time_alias_can_supply_frequency(self):
        run = run_fixture()
        for key in ("wall_time", "step_wall_time", "frequency_hz"):
            del run["execution"][key]
        self.assertAlmostEqual(trial_metrics(run)["frequency_hz"], 3/.7)

    def test_wall_boundaries_can_supply_frequency_without_duration_aliases(self):
        run = run_fixture()
        del run["execution"]["step_wall_time"]
        del run["dynamic"]["compute_time"]
        self.assertAlmostEqual(trial_metrics(run)["frequency_hz"], 3/.7)

    def test_missing_execution_alignment_is_rejected_even_with_matching_lengths(self):
        for alignment in (None, "C3_DEBUG_CURR", "planner_updates"):
            run = run_fixture()
            run["execution"]["alignment"] = alignment
            with self.subTest(alignment=alignment), self.assertRaisesRegex(ValueError, "alignment"):
                trial_metrics(run)

    def test_wrong_schema_or_identity_is_rejected(self):
        for section, field, value in (("schema", "version", "summary"),
                ("schema", "semantics_version", 3), ("run", "algorithm", "mppi"),
                ("run", "task", ""), ("run", "object", None), ("run", "run_id", "")):
            run = run_fixture()
            run[section][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                trial_metrics(run)

    def test_malformed_core_telemetry_and_aliases_are_rejected(self):
        cases = (("execution", "n_steps_executed", True),
                 ("execution", "n_steps_executed", 3.0),
                 ("execution", "step_budget", 2), ("hyperparameters", "steps", 99),
                 ("dynamic", "time", [10, 11, 13, 15]),
                 ("execution", "sim_time", [10, 11, 13]),
                 ("dynamic", "object_pose", [[0, 0, 0]] * 3),
                 ("dynamic", "object_pose", [[0, 0, float("nan")]] * 4),
                 ("dynamic", "object_pose", [[0, 0, False]] * 4),
                 ("static", "goal", [0, 0]),
                 ("hyperparameters", "goal_pos_tol", .2),
                 ("hyperparameters", "object", "banana"))
        for section, field, value in cases:
            run = run_fixture()
            run[section][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                trial_metrics(run)

    def test_repeated_or_negative_sim_time_is_rejected(self):
        for times in ([10, 10, 13, 16], [-1, 10, 13, 16]):
            run = run_fixture()
            run["execution"]["sim_time"] = run["dynamic"]["time"] = times
            with self.subTest(times=times), self.assertRaises(ValueError):
                trial_metrics(run)

    def test_malformed_wall_timing_is_not_silently_dropped_or_repaired(self):
        for section, key, value in (("execution", "wall_time", [0, .1, .1, .7]),
                ("execution", "wall_time", [10, 10.1, 10.3, 10.7]),
                ("execution", "step_wall_time", [.1, .2]),
                ("execution", "step_wall_time", [.1, -.2, .4]),
                ("execution", "step_wall_time", [.1, float("inf"), .4]),
                ("execution", "step_wall_time", [.1, "0.2", .4]),
                ("execution", "step_wall_time", [.1, .2, .5]),
                ("dynamic", "compute_time", [.1, .3, .4]),
                ("execution", "frequency_hz", 1000)):
            run = run_fixture()
            run[section][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                trial_metrics(run)

    def test_settings_exclude_ids_outcomes_and_timing(self):
        run = run_fixture()
        run["hyperparameters"].update(steps_run=999, frequency_hz=1000,
            control_dt_source="wrong_source", provenance={"large": list(range(100))})
        settings = trial_metrics(run)["settings"]
        for key in ("run_id", "steps_run", "frequency_hz", "control_dt", "control_dt_source", "provenance"):
            self.assertNotIn(key, settings)
        self.assertEqual({key: settings[key] for key in ("object", "start", "goal", "seed", "horizon", "n_admm")},
                         {"object": "T_shape", "start": 2, "goal": 2, "seed": 42, "horizon": 5, "n_admm": 3})

    def test_conflicting_setting_aliases_rejected(self):
        for key, value in (("seed", 43), ("start", 3)):
            run = run_fixture()
            run["hyperparameters"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                trial_metrics(run)

    def test_zero_executions_have_no_fabricated_errors_or_frequency(self):
        run = run_fixture()
        run["execution"].update(n_steps_executed=0, sim_time=[10], wall_time=[0],
                                step_wall_time=[], frequency_hz=None)
        run["dynamic"].update(time=[10], object_pose=[[0, 0, 0]], compute_time=[])
        result = trial_metrics(run)
        self.assertFalse(result["success"])
        self.assertEqual(result["steps_to_goal"], 10)
        self.assertEqual(result["execution_time"], 0)
        for key in ("eps_d", "eps_o", "theta", "trajectory_mean_position_error",
                    "trajectory_mean_orientation_error", "frequency_hz"):
            self.assertIsNone(result[key])


class AggregateMetricsTests(unittest.TestCase):
    def test_means_preserve_censoring_and_average_per_run_frequency(self):
        first = trial_metrics(run_fixture())
        other = run_fixture()
        other["dynamic"]["object_pose"][2] = [1, 0, 0]
        other["execution"]["wall_time"] = [0, .2, .6, 1.4]
        other["execution"]["step_wall_time"] = other["dynamic"]["compute_time"] = [.2, .4, .8]
        other["execution"]["frequency_hz"] = 3/1.4
        second = trial_metrics(other)
        result = aggregate_metrics([first, second])
        self.assertEqual(result["n"], 2)
        self.assertEqual(result["SR"], .5)
        self.assertEqual(result["steps"], 6)
        self.assertEqual(result["execution_time"], 4.5)
        self.assertAlmostEqual(result["eps_d"], (first["eps_d"]+second["eps_d"])/2)
        self.assertEqual(result["eps_d_success"], first["eps_d"])
        self.assertAlmostEqual(result["eps_o"], (first["eps_o"]+second["eps_o"])/2)
        self.assertEqual(result["eps_o_success"], first["eps_o"])
        for key in ("trajectory_mean_position_error", "trajectory_mean_orientation_error"):
            self.assertAlmostEqual(result[key], (first[key]+second[key])/2)
            self.assertEqual(result[key+"_success"], first[key])
            self.assertEqual(result["available"][key], 2)
            self.assertEqual(result["available"][key+"_success"], 1)
        self.assertAlmostEqual(result["theta"], (first["theta"]+second["theta"])/2)
        self.assertAlmostEqual(result["frequency_hz"], (3/.7+3/1.4)/2)
        self.assertEqual(result["available"]["steps"], 2)
        self.assertEqual(result["available"]["eps_d_success"], 1)
        self.assertEqual(result["available"]["eps_o_success"], 1)

    def test_successful_residuals_obey_tolerances_despite_larger_trajectory_errors(self):
        first = trial_metrics(run_fixture())
        second_run = run_fixture()
        second_run["dynamic"]["object_pose"][2] = [.02, 0, .08]
        second = trial_metrics(second_run)
        failed_run = run_fixture()
        failed_run["dynamic"]["object_pose"][2] = [1, 0, 0]
        failed = trial_metrics(failed_run)
        result = aggregate_metrics([first, second, failed])
        self.assertEqual(result["SR"], 2/3)
        self.assertAlmostEqual(result["eps_d_success"], .025)
        self.assertAlmostEqual(result["eps_o_success"], .05)
        self.assertLess(result["eps_d_success"], .05)
        self.assertLess(result["eps_o_success"], .1)
        self.assertGreater(result["trajectory_mean_position_error_success"], .05)
        self.assertGreater(result["trajectory_mean_orientation_error_success"], .1)
        self.assertEqual(result["available"]["eps_o"], 3)
        self.assertEqual(result["available"]["eps_o_success"], 2)
        self.assertEqual(result["theta"], result["trajectory_mean_orientation_error"])

    def test_partial_means_explicitly_report_availability(self):
        first = trial_metrics(run_fixture())
        second = dict(first, success=False, steps_to_goal=None, frequency_hz=None)
        result = aggregate_metrics([first, second])
        self.assertEqual(result["steps"], 2)
        self.assertEqual(result["available"]["steps"], 1)
        self.assertEqual(result["available"]["frequency_hz"], 1)
        self.assertEqual(len(result["warnings"]), 2)
        self.assertIn("1/2", result["warnings"][0])

    def test_no_success_or_available_values_remain_none(self):
        result = aggregate_metrics([{"success": False}])
        self.assertEqual(result["SR"], 0)
        for key in ("eps_d", "eps_d_success", "eps_o", "eps_o_success",
                    "trajectory_mean_position_error", "trajectory_mean_position_error_success",
                    "trajectory_mean_orientation_error", "trajectory_mean_orientation_error_success",
                    "theta", "steps", "frequency_hz", "execution_time"):
            self.assertIsNone(result[key])
            self.assertEqual(result["available"][key], 0)

    def test_empty_aggregation_rejected(self):
        with self.assertRaises(ValueError):
            aggregate_metrics([])


if __name__ == "__main__":
    unittest.main()
