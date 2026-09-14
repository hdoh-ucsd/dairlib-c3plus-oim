"""Planning telemetry remains independent of policy execution and snapshots."""
import json
from pathlib import Path
import tempfile
import unittest

from c3plus.recording.planning import read_planning_updates


class PlanningReaderTests(unittest.TestCase):
    def test_reader_preserves_planning_only_updates_without_other_event_types(self):
        updates = [{"update": index, "utime": 100000 + index,
                    "mode": "c3" if index else "reposition",
                    "unrecognized_diagnostic": {"preserve": index}}
                   for index in range(3)]
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "planner.log"
            self.assertIsNone(read_planning_updates(tmp))
            noise = ("ordinary output\n[C3_EXECUTION_STEP] {\"execution_step\": 0}\n"
                     "[C3_EXECUTION_BOUNDARY] {\"boundary_step\": 0}\n")
            log.write_text(noise)
            self.assertEqual(read_planning_updates(tmp), [])
            log.write_text(noise + "".join("[C3_PLANNING_UPDATE] " + json.dumps(item) + "\n"
                                           for item in updates))
            self.assertEqual(read_planning_updates(tmp), updates)
            log.write_text("[C3_PLANNING_UPDATE] {truncated\n")
            with self.assertRaises(json.JSONDecodeError):
                read_planning_updates(tmp)
