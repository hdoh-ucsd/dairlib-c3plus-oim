import copy
import unittest

from c3plus import configs as S
from c3plus.experiments import run as R

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_planner_environment_serialization(self):
        config = {
            "obstacles": {"polygons": [
                [[0, 0], [2, 0], [2, 2], [0, 2]],
                [[3, 0], [4, 0], [3, 1]],
            ]},
            "planner": {"box_polygons": [0], "polygon_overrides": {2: 1}},
        }
        self.assertEqual(S.planner_environment(config), {
            "SAMPLING_C3_OBS_BOXES": "1,1,1,1",
            "SAMPLING_C3_OBS_POLYS": "2|3,0|4,0|3,1",
        })
        self.assertEqual(S.planner_environment({}), {})


    def test_planner_environment_rejects_invalid_geometry(self):
        valid = {"obstacles": {"polygons": [[[0, 0], [1, 0], [1, 1], [0, 1]]]},
                 "planner": {"box_polygons": [0]}}
        cases = []
        bad_reference = copy.deepcopy(valid)
        bad_reference["planner"]["box_polygons"] = [1]
        cases.append(bad_reference)
        triangle = copy.deepcopy(valid)
        triangle["obstacles"]["polygons"][0].pop()
        cases.append(triangle)
        nonfinite = copy.deepcopy(valid)
        nonfinite["obstacles"]["polygons"][0][0][0] = float("nan")
        cases.append(nonfinite)
        overlapping_slots = copy.deepcopy(valid)
        overlapping_slots["planner"]["polygon_overrides"] = {0: 0}
        cases.append(overlapping_slots)
        for config in cases:
            with self.subTest(config=config), self.assertRaises(ValueError):
                S.planner_environment(config)


    def test_planner_environment_preserves_scene_indices(self):
        def env(scene):
            return S.planner_environment(R.yaml.safe_load((S.CONFIG_DIR / f"{scene}.yaml").read_text()))
        shelf = env("shelf_gap")["SAMPLING_C3_OBS_BOXES"].split(";")
        self.assertEqual(shelf, ["0.62,0,0.13,0.08"] * 3 + ["0.24,0,0.05,0.08"] * 2)
        ycb = env("ycb_clutter")
        self.assertEqual([p.split("|")[0] for p in ycb["SAMPLING_C3_OBS_POLYS"].split(";")], ["2", "3"])
        icra = env("icra_sign")
        self.assertEqual(icra["SAMPLING_C3_OBJECT_FOOTPRINT"], "c_glyph")
        self.assertEqual(icra["SAMPLING_C3_OBS_TOP_Z"], "0.046")
        self.assertEqual([p.split("|")[0] for p in icra["SAMPLING_C3_OBS_POLYS"].split(";")], list(map(str, range(7))))
