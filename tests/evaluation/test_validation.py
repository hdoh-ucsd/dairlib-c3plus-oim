from c3plus.evaluation import validation as Validation
from c3plus.evaluation import postprocess as Postprocess
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from c3plus.evaluation import exporter as P

from tests.fixtures.results import ArtifactFixtures, RUN_ID

class RunArtifactTests(ArtifactFixtures, unittest.TestCase):
    def test_drifted_native_quaternions_survive_regeneration_and_reject_tampering(self):
        from tests.fixtures import results as fixtures
        for scale in (1.000059, .999941):
            with self.subTest(scale=scale), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "run"
                original = self.make_run(root)
                native, updates = fixtures.ProjectionFixtures().physical_fixture()
                native["headers"][0]["step_budget"] = None
                native["terminals"][0]["reason"] = "shutdown"
                for state in [*native["boundaries"], *native["terminals"]]:
                    state["objects"][0]["q"][:4] = [
                        math.cos(.6) * scale, 0, 0, math.sin(.6) * scale]
                with (root / "sim.log").open("a") as stream:
                    for key, prefix in (("headers", "C3_EXECUTION_LOGGING"),
                                        ("boundaries", "C3_EXECUTION_BOUNDARY"),
                                        ("terminals", "C3_EXECUTION_TERMINAL")):
                        for record in native[key]:
                            stream.write(f"[{prefix}] " + json.dumps(record) + "\n")
                with (root / "planner.log").open("a") as stream:
                    for update in updates:
                        stream.write("[C3_PLANNING_UPDATE] " + json.dumps(update) + "\n")
                args = SimpleNamespace(run_dir=str(root), scene="open_task", run_id=RUN_ID)
                path = root / f"{RUN_ID}_result.json"
                with redirect_stdout(io.StringIO()):
                    Postprocess.export_existing_result(args, None)
                exported = json.loads(path.read_text())
                compacted = self.compact(root)
                self.assertEqual(compacted["dynamic"], exported["dynamic"])
                self.assertEqual(compacted["recording"]["execution_native"], native)
                self.assertEqual(compacted["recording"]["steps_raw"], original["steps"])
                self.assertEqual(compacted["recording"]["state_trace"], original["trace"])
                self.assertEqual({p.name for p in root.iterdir()}, {path.name, f"{RUN_ID}.mp4"})
                cfg = compacted["provenance"]["evaluation_scene_config"]
                Validation._validate(compacted, compacted["recording"], cfg, root)
                with redirect_stdout(io.StringIO()):
                    Postprocess.export_existing_result(args, None)
                self.assertEqual(json.loads(path.read_text()), compacted)
                for location in ("raw quaternion", "projected quaternion", "projected yaw"):
                    bad = deepcopy(compacted)
                    if location == "raw quaternion":
                        bad["recording"]["execution_native"]["boundaries"][0]["objects"][0]["q"][0] += .05
                    elif location == "projected quaternion":
                        bad["dynamic"]["object_pose_3d"][0][0] += .05
                    else:
                        bad["dynamic"]["object_pose"][0][2] += .05
                    with self.subTest(tampered=location), self.assertRaises(ValueError):
                        Validation._validate(bad, bad["recording"], cfg, root)


    def test_execution_events_survive_compaction_and_reject_timing_tampering(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            self.make_run(root)
            # Four execution handoffs, including both modes, versus three snapshots.
            events = [{"execution_step": i, "execution_wall_time": t,
                       "utime": 100_000 + i, "mode": "c3" if i > 1 else "reposition"}
                      for i, t in enumerate([0.0, .03, .08, .11])]
            with (root / "planner.log").open("a") as stream:
                for event in events:
                    stream.write("[C3_EXECUTION_STEP] " + json.dumps(event) + "\n")
            path = root / f"{RUN_ID}_result.json"
            result = json.loads(path.read_text())
            P.add_execution_timing(result, events)
            P.write_result_json(path, result)
            compacted = self.compact(root)
            self.assertEqual(compacted["recording"]["execution_steps"], events)
            self.assertEqual(compacted["dynamic"]["execution_wall_time"], [0.0, .03, .08, .11])
            self.assertAlmostEqual(compacted["execution_timing"]["frequency_hz"], 3 / .11)
            cfg = compacted["provenance"]["evaluation_scene_config"]
            Validation._validate(compacted, compacted["recording"], cfg, root)
            for field in ("frequency_hz", "n_steps"):
                changed = deepcopy(compacted)
                changed["execution_timing"][field] += 1
                with self.subTest(field=field), self.assertRaises(ValueError):
                    Validation._validate(changed, changed["recording"], cfg, root)
            changed = deepcopy(compacted)
            changed["dynamic"]["execution_wall_time"][1] += .001
            with self.assertRaises(ValueError):
                Validation._validate(changed, changed["recording"], cfg, root)
            self.assertEqual(set(p.name for p in root.iterdir()), {path.name, f"{RUN_ID}.mp4"})


    def test_physical_execution_survives_packaging_with_independent_snapshot_data(self):
        from tests.fixtures import results as fixtures
        native, updates = fixtures.ProjectionFixtures().physical_fixture()
        native["headers"][0]["step_budget"] = None
        native["terminals"][0]["reason"] = "shutdown"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "run"
            original = self.make_run(root)
            with (root / "sim.log").open("a") as stream:
                for key, prefix in (("headers", "C3_EXECUTION_LOGGING"),
                                    ("boundaries", "C3_EXECUTION_BOUNDARY"),
                                    ("terminals", "C3_EXECUTION_TERMINAL")):
                    for record in native[key]:
                        stream.write(f"[{prefix}] " + json.dumps(record) + "\n")
            with (root / "planner.log").open("a") as stream:
                for update in updates:
                    stream.write("[C3_PLANNING_UPDATE] " + json.dumps(update) + "\n")
            path = root / f"{RUN_ID}_result.json"
            result = json.loads(path.read_text())
            cfg = result["provenance"]["evaluation_scene_config"]
            P.add_execution_projection(result, cfg, native, updates)
            P.write_result_json(path, result)
            # Physical telemetry is present before compaction, but raw CSV and
            # observations still live in sidecars at this point.
            with redirect_stdout(io.StringIO()):
                Postprocess.export_existing_result(SimpleNamespace(run_dir=str(root), scene="open_task", run_id=RUN_ID), None)
            self.assertEqual(json.loads(path.read_text())["execution"], result["execution"])
            compacted = self.compact(root)
            self.assertEqual(compacted["recording"]["execution_native"], native)
            self.assertEqual(compacted["recording"]["planning_updates"], updates)
            self.assertEqual(compacted["recording"]["steps_raw"], original["steps"])
            self.assertEqual(compacted["recording"]["state_trace"], original["trace"])
            self.assertEqual(compacted["dynamic"], result["dynamic"])
            self.assertEqual(compacted["execution"]["n_steps_executed"], 2)
            Validation._validate(compacted, compacted["recording"], cfg, root)
            for key in ("frequency_hz", "n_steps_executed"):
                bad = deepcopy(compacted)
                bad["execution"][key] += 1
                with self.subTest(key=key), self.assertRaises(ValueError):
                    Validation._validate(bad, bad["recording"], cfg, root)
            self.assertEqual({p.name for p in root.iterdir()}, {path.name, f"{RUN_ID}.mp4"})
            with redirect_stdout(io.StringIO()):
                Postprocess.export_existing_result(SimpleNamespace(run_dir=str(root), scene="open_task", run_id=RUN_ID), None)
            self.assertEqual(json.loads(path.read_text()), compacted)


    def test_invalid_inputs_never_delete_or_replace_original_artifacts(self):
        def invalid_json(root):
            (root / f"{RUN_ID}_result.json").write_text("{broken JSON")

        def wrong_run_id(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["run_id"] = "a different run"
            path.write_text(json.dumps(value))

        def changed_projected_trajectory(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["dynamic"]["object_pose"][1][0] += 1
            path.write_text(json.dumps(value))

        def changed_observed_dt(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["hyperparameters"]["control_dt"] = .0001
            path.write_text(json.dumps(value))

        def invented_compute_time(root):
            path = root / f"{RUN_ID}_result.json"
            value = json.loads(path.read_text())
            value["dynamic"]["compute_time"] = [.001] * value["steps_run"]
            path.write_text(json.dumps(value))

        def incomplete_phase(root):
            path = root / "runtime_status.json"
            status = json.loads(path.read_text())
            del status["render_rc"]
            path.write_text(json.dumps(status))

        def failed_phase(root):
            path = root / "runtime_status.json"
            status = json.loads(path.read_text())
            status["postprocess_rc"] = 17
            path.write_text(json.dumps(status))

        mutations = {
            "invalid result JSON": invalid_json,
            "wrong run identity": wrong_run_id,
            "projection changed": changed_projected_trajectory,
            "physics dt substituted for observed interval": changed_observed_dt,
            "compute time invented without a recorded signal": invented_compute_time,
            "missing packaging phase": incomplete_phase,
            "failed packaging phase": failed_phase,
            "missing completion marker": lambda p: (p / "RUN_COMPLETE").unlink(),
            "invalid raw JSONL": lambda p: (p / "steps_raw.jsonl").write_text("not JSON\n"),
            "empty replay trace": lambda p: (p / "state_trace.jsonl").write_text(""),
            "missing metrics rows": lambda p: (p / f"{RUN_ID}_metrics.csv").write_text("control_step,sim_time\n"),
            "configuration changed after recording": lambda p: (p / "config/sampling.yaml").write_text(
                "sample_random_seed: 99\nsampling_strategy: 8\n"),
            "unknown personal file": lambda p: (p / "notes.txt").write_text("Keep these notes"),
            "untrusted symlink": lambda p: (p / "config/external.yaml").symlink_to(p / "config/goal.yaml"),
        }
        for description, mutate in mutations.items():
            with self.subTest(description=description), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / "run"
                self.make_run(root)
                mutate(root)
                self.assert_rejected_without_changes(root)
