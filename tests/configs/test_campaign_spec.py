"""The campaign selection spec: parsing, validation and CLI precedence."""
import io
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from c3plus.configs import campaign_spec as S
from c3plus.utils import campaign as G


def _spec(tmp, data):
    path = Path(tmp) / "spec.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


class CampaignSpecTests(unittest.TestCase):
    def test_default_spec_in_the_checkout_parses(self):
        # The shipped campaign.yaml must stay loadable; it is the default input.
        spec = S.load_campaign_spec()
        self.assertTrue(set(spec) <= {"scenes", "objects", "pairs", "obstacle_cost",
                                      "cap", "steps", "record", "video"})

    def test_only_declared_keys_are_returned(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = S.load_campaign_spec(_spec(tmp, {"tasks": ["open_table"]}))
            self.assertEqual(spec, {"scenes": ["open_table"]})

    def test_aliases_and_explicit_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = S.load_campaign_spec(_spec(tmp, {
                "scenes": ["open_task"], "objects": ["T_block"], "pairs": [[2, 2], [1, 3]]}))
            self.assertEqual(spec["scenes"], ["open_table"])
            self.assertEqual(spec["objects"], ["T_shape"])
            self.assertEqual(spec["pairs"], [(2, 2), (1, 3)])

    def test_invalid_specs_fail_loudly(self):
        # Each of these would otherwise run a different campaign than the file reads as.
        for data in ({"objectss": ["T_shape"]},
                     {"tasks": ["open_table"], "scenes": ["open_table"]},
                     {"tasks": "open_table"},
                     {"tasks": []},
                     {"pairs": "everything"},
                     {"pairs": [[2]]},
                     {"pairs": [[0, 2]]},
                     {"obstacle_cost": "quadratic"},
                     {"cap": 0},
                     {"steps": 0},
                     {"steps": "many"},
                     {"video": "no"},
                     {"record": 1},
                     {"record": False, "video": True},
                     {"seed": 7}):
            with self.subTest(data=data), tempfile.TemporaryDirectory() as tmp:
                with self.assertRaises(ValueError):
                    S.load_campaign_spec(_spec(tmp, data))


class CampaignSpecBudgetTests(unittest.TestCase):
    def test_budget_and_artifact_keys_parse(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = S.load_campaign_spec(_spec(tmp, {"steps": 2671, "video": False, "record": True}))
            self.assertEqual(spec, {"steps": 2671, "video": False, "record": True})

    def test_no_recording_forces_no_rendering(self):
        # There is no recorded trajectory left to replay.
        with tempfile.TemporaryDirectory() as tmp:
            spec = S.load_campaign_spec(_spec(tmp, {"record": False}))
            self.assertEqual(spec, {"record": False, "video": False})


class CampaignSpecPrecedenceTests(unittest.TestCase):
    def _run_count(self, extra, spec_data=None):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            argv = ["--out", str(out), "--dry-run", *extra]
            if spec_data is not None:
                argv += ["--spec", str(_spec(tmp, spec_data))]
            stream = io.StringIO()
            with redirect_stdout(stream):
                G.main(argv)
            return stream.getvalue()

    def test_spec_selects_when_no_selection_flag_is_given(self):
        output = self._run_count([], {"tasks": ["open_table"], "objects": ["T_shape"],
                                      "pairs": [[2, 2]], "obstacle_cost": "exponential"})
        self.assertIn("selection from", output)
        self.assertIn('"run_count": 1', output)

    def test_command_line_selectors_bypass_the_spec_entirely(self):
        # A one-off run must never require editing the file.
        output = self._run_count(["--tasks", "shelf_gap", "--objects", "hammer",
                                  "--pairs", "smoke", "--obstacle_cost", "exponential"],
                                 {"tasks": ["open_table"], "objects": ["T_shape"],
                                  "pairs": [[2, 2], [1, 3], [4, 4]]})
        self.assertNotIn("selection from", output)
        self.assertIn('"run_count": 1', output)

    def test_explicit_cap_overrides_the_spec_cap(self):
        output = self._run_count(["--cap", "77"], {"tasks": ["open_table"], "objects": ["T_shape"],
                                                   "pairs": [[2, 2]], "cap": 300})
        self.assertIn('"simulation_cap_seconds": 77', output)

    def test_spec_supplies_the_step_budget_and_the_flag_overrides_it(self):
        base = {"tasks": ["open_table"], "objects": ["T_shape"], "pairs": [[2, 2]]}
        self.assertIn('"execution_step_budget": 2671',
                      self._run_count([], {**base, "steps": 2671}))
        self.assertIn('"execution_step_budget": 40',
                      self._run_count(["--steps", "40"], {**base, "steps": 2671}))

    def test_spec_can_switch_rendering_off(self):
        base = {"tasks": ["open_table"], "objects": ["T_shape"], "pairs": [[2, 2]]}
        # The decision is recorded in the plan, so a resume cannot mix artifact sets.
        self.assertIn('"video": false', self._run_count([], {**base, "video": False}))
        self.assertNotIn('"video": false', self._run_count([], base))
        self.assertIn('"video": false', self._run_count(["--no-video"], base))
        unrecorded = self._run_count([], {**base, "record": False})
        self.assertIn('"record": false', unrecorded)
        self.assertIn('"video": false', unrecorded)

    def test_unrecorded_campaigns_refuse_resume_and_export(self):
        # Neither has anything to work from: no result JSON is ever written.
        for extra in (["--resume"], ["--oim-out", "oim"]):
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                self._run_count(["--no-record", *extra],
                                {"tasks": ["open_table"], "objects": ["T_shape"], "pairs": [[2, 2]]})

    def test_spec_cannot_be_combined_with_suite_or_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            path = _spec(tmp, {"tasks": ["open_table"]})
            with self.assertRaises(SystemExit):
                G.main(["--out", str(out), "--suite", "full", "--spec", str(path), "--dry-run"])


if __name__ == "__main__":
    unittest.main()
