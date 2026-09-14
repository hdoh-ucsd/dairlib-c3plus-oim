from c3plus.evaluation import validation as Validation
from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml

from c3plus import configs as S
from c3plus.runtime import provenance as Provenance
from c3plus.evaluation import exporter as P, package as A

from tests.fixtures.results import ProjectionFixtures, ArtifactFixtures, RUN_ID

class ResultProjectionTests(ProjectionFixtures, unittest.TestCase):
    def test_legacy_native_metadata_and_weights_relocated_without_losing_only_copy(self):
        result = self.semantic_result()
        weights = {"q_pos": 3., "q_theta": 1.2}
        result["evaluation"].pop("weights")
        result["hyperparameters"]["costs"] = weights
        legacy = result["provenance"].pop("c3plus")
        legacy.update(obstacle_cost="exponential", wall_cap_seconds=600,
                      configurations={"config/options.yaml": {"N": 5, "admm_iter": 3}},
                      sampler_environment={"SAMPLING_C3_SEED": "42"})
        result["hyperparameters"].pop("obstacle_cost")
        result["hyperparameters"]["c3plus"] = deepcopy(legacy)
        P.add_result_semantics(result)
        self.assertEqual(result["provenance"]["c3plus"], legacy)
        self.assertEqual(result["evaluation"]["weights"], weights)
        self.assertEqual(result["hyperparameters"]["obstacle_cost"], "exponential")
        self.assertNotIn("c3plus", result["hyperparameters"])
        self.assertNotIn("costs", result["hyperparameters"])
        before = deepcopy(result)
        P.add_result_semantics(result)
        self.assertEqual(result, before)


    def test_native_duplicate_removed_only_after_exact_snapshot_validation(self):
        result = self.semantic_result()
        configs = {"config/goal.yaml": {"position_success_threshold": .02,
                                        "orientation_success_threshold": .1}}
        result["provenance"]["c3plus"]["configurations"] = deepcopy(configs)
        text = yaml.safe_dump(configs["config/goal.yaml"])
        result["provenance"]["configuration"] = {"files": {"config/goal.yaml": {
            "text": text, "data": deepcopy(configs["config/goal.yaml"]),
            "size_bytes": len(text.encode()), "sha256": hashlib.sha256(text.encode()).hexdigest()}}}
        before = deepcopy(result)
        P.add_result_semantics(result)
        self.assertNotIn("configurations", result["provenance"]["c3plus"])
        self.assertEqual(result["provenance"]["configuration"], before["provenance"]["configuration"])
        self.assertEqual(result["native_controller"]["success_thresholds"],
                         {"position_m": .02, "orientation_rad": .1})
        bad = deepcopy(before)
        bad["provenance"]["c3plus"]["configurations"]["config/goal.yaml"]["position_success_threshold"] = .03
        with self.assertRaisesRegex(ValueError, "disagrees"):
            P.add_result_semantics(bad)
        bad = deepcopy(before)
        bad["provenance"]["configuration"]["files"]["config/goal.yaml"]["text"] += "# changed\n"
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            P.add_result_semantics(bad)


    def test_t_identity_comes_from_saved_catalogue_without_filename_inference(self):
        result = self.semantic_result()
        result.pop("object_name", None)
        result["run"]["object"] = None
        result["hyperparameters"]["object"] = None
        result["static"]["object_name"] = None
        saved_evaluation = result["provenance"].get("evaluation_scene_config", {})
        saved_evaluation.pop("object_name", None)
        saved_evaluation.update(object_body_name="vertical_link", object_channel_substring="G_shape_video")
        result["runtime_status"] = {"scene": "open_task", "seed": 42, "seed_verified": True,
                                    "sampler_settings": {"SAMPLING_C3_SEED": "42"}}
        catalogue = {"scenes": {"open_task": {"object_profile": "Tblock"}},
                     "object_profiles": {"Tblock": {"object_body_name": "vertical_link"}}}
        text = yaml.safe_dump(catalogue)
        result["provenance"]["configuration"] = {"files": {
            "config/source_experiments.yaml": {"data": catalogue, "text": text,
                "sha256": hashlib.sha256(text.encode()).hexdigest(), "size_bytes": len(text.encode())}}}
        # The legacy run ID deliberately still mentions banana. The saved selection wins.
        runtime = deepcopy(result["runtime_status"])
        native = deepcopy(result["provenance"]["c3plus"].setdefault("configurations", {}))
        P.add_result_semantics(result)
        self.assertEqual(result["object_name"], "T_block")
        self.assertEqual(result["run"]["object"], "T_block")
        self.assertEqual(result["hyperparameters"]["object"], "T_block")
        self.assertEqual(result["static"]["object_name"], "T_block")
        self.assertEqual(result["runtime_status"], runtime)
        self.assertEqual(result["provenance"]["c3plus"].setdefault("configurations", {}), native)
        self.assertEqual(result["provenance"]["configuration"]["files"]["config/source_experiments.yaml"]["text"], text)


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
            self.assertEqual(result["provenance"]["c3plus"]["configurations"]["config/repository/profiles/options.yaml"],
                             native_options)
            self.assertEqual(result["provenance"]["c3plus"]["configurations"]["config/repository/profiles/sampling.yaml"],
                             sampling)
            self.assertEqual(result["provenance"]["c3plus"]["configurations"]["config/repository/profiles/reposition.yaml"],
                             {"saved_parameter": 123})
            self.assertEqual(result["provenance"]["c3plus"]["sampler_environment"], {"SAMPLING_C3_SEED": "123"})
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
                    "iterations", "goal_pos_tol", "goal_theta_tol"):
            self.assertIsNone(hyper[key], key)
        self.assertIsNone(result["run"]["seed"])
        self.assertIsNone(result["static"]["sim_timestep"])
        self.assertIsNone(result["static"]["object_limit_surface_d"])
        self.assertIsNone(result["static"]["object_wrench_limit"])
        self.assertTrue(result["provenance"]["missing_optional_metadata"])


class RunArtifactTests(ArtifactFixtures, unittest.TestCase):
    def test_captured_source_state_survives_projection_and_compaction(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "source"
            repo.mkdir()

            def git(*arguments):
                return subprocess.check_output(["git", *arguments], cwd=repo,
                                               stderr=subprocess.PIPE)

            git("init", "-q")
            tracked = repo / "controller.py"
            tracked.write_text("version = 'committed'\n")
            git("add", "controller.py")
            git("-c", "user.name=Fixture", "-c", "user.email=fixture@example.invalid",
                "commit", "-qm", "fixture baseline")
            tracked.write_text("version = 'staged'\n")
            git("add", "controller.py")
            tracked.write_text("version = 'launch-time worktree'\n")
            (repo / "untracked_source.bin").write_bytes(b"\xffsource bytes\x00")
            captured = Provenance.capture_source_state(repo)
            self.assertTrue(captured["worktree_dirty"])
            self.assertIn("untracked_source.bin", captured["untracked_files"])

            root = Path(tmp) / "run"
            original = self.make_run(root)
            raw = (json.dumps(captured, indent=2, allow_nan=False) + "\n").encode("utf-8")
            source_path = root / "config/source_state.json"
            source_path.write_bytes(raw)
            descriptor = {"path": "config/source_state.json",
                          "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw),
                          "base_commit": captured["base_commit"], "scope": captured["scope"]}
            status = {**original["status"], "commit": captured["base_commit"],
                      "worktree_dirty": captured["worktree_dirty"], "source_state": descriptor}
            (root / "runtime_status.json").write_text(json.dumps(status))
            with (root / f"{RUN_ID}_metrics.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            projected = P.project_result(root, "open_task", RUN_ID, original["cfg"],
                                         original["result"], original["steps"], rows,
                                         evaluation_costs=original["manifest"]["evaluation"]["costs"])
            self.assertEqual(projected["provenance"]["source_state"]["status"], "recorded")
            self.assertIsNone(projected["provenance"]["source_state"]["embedded_json_pointer"])
            P.write_result_json(root / f"{RUN_ID}_result.json", projected)

            # The captured hash must reject changes before any run evidence is deleted.
            source_path.write_bytes(raw + b"\n")
            self.assert_rejected_without_changes(root)
            source_path.write_bytes(raw)
            result = self.compact(root)
            self.assertEqual({path.name for path in root.iterdir()},
                             {f"{RUN_ID}_result.json", f"{RUN_ID}.mp4"})
            embedded = result["provenance"]["configuration"]["files"]["config/source_state.json"]
            self.assertEqual(embedded["text"].encode("utf-8"), raw)
            self.assertEqual(embedded["data"], captured)
            self.assertEqual(embedded["sha256"], descriptor["sha256"])
            self.assertEqual(embedded["size_bytes"], descriptor["size_bytes"])
            self.assertEqual(result["runtime_status"], status)
            provenance = result["provenance"]["source_state"]
            self.assertEqual({key: provenance[key] for key in descriptor}, descriptor)
            target = result
            for token in provenance["embedded_json_pointer"].lstrip("/").split("/"):
                target = target[token.replace("~1", "/").replace("~0", "~")]
            self.assertEqual(target, captured)
            self.assertIsNone(result["native_controller"]["success"])
            self.assertEqual(result["dynamic"], P._json_values(projected["dynamic"]))
            self.assertEqual(result["recording"]["steps_raw"], original["steps"])
            self.assertEqual((root / f"{RUN_ID}.mp4").read_bytes(), original["video"])
            # Validation and semantic refresh use embedded evidence after config/ is gone.
            Validation._validate(result, result["recording"], original["cfg"], root)
            self.assertEqual(P.add_result_semantics(deepcopy(result), root), result)
            self.assertTrue(A.completion(root, RUN_ID))
