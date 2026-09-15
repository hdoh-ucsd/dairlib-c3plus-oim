import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

import yaml

from c3plus.configs.paths import REPO
from c3plus.configs.poses import (PROVENANCE_FILE, load_pose_catalogue, pose_ids,
                                 pose_catalogue_snapshot, pose_provenance,
                                 pose_source_files, resolve_pose)


class CanonicalPoseTests(unittest.TestCase):
    def copy_catalogue(self, directory):
        repo = Path(directory)
        for source in pose_source_files():
            target = repo / source.relative_to(REPO)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        return repo

    def replace_pose_file(self, repo, text):
        path = repo / "examples/poses/open_table.yaml"
        path.write_text(text)
        provenance = repo / PROVENANCE_FILE
        data = json.loads(provenance.read_text())
        data["files"][path.name]["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        provenance.write_text(json.dumps(data))

    def test_source_identity_and_exact_stored_values(self):
        provenance = pose_provenance()
        self.assertEqual(provenance["source_commit"],
                         "a31203d8e9a347ba7bf373954a4b7e4059d1e350")
        catalogue = load_pose_catalogue()
        self.assertEqual(len(catalogue), 6)
        self.assertNotIn("clutter", catalogue)
        for task, poses in catalogue.items():
            source = REPO / "examples/poses" / f"{task}.yaml"
            self.assertEqual(poses, yaml.safe_load(source.read_text()))
            self.assertEqual(len(poses["starts"]) * len(poses["goals"]), 25)
        self.assertEqual(resolve_pose("open_table", "goal", 1), [0.381, -0.4, 3.1416])

    def test_task_specific_poses_and_order_are_retained(self):
        self.assertEqual(pose_ids("start"), ("1", "2", "3", "4", "5"))
        self.assertEqual(pose_ids("goal", "icra_sign"), pose_ids("goal"))
        self.assertEqual(resolve_pose("icra_sign", "goal", "1"), [0.5, -0.4, 1.5708])
        self.assertEqual(resolve_pose("shelf_gap", "goal", 2), [0.404, -0.393, 2.7924])
        self.assertEqual(resolve_pose("open_table", "goal", 2), [0.397, -0.431, 3.24])
        self.assertEqual(resolve_pose("slalom", "start", 4), [0.361, 0.37, 0.3282])
        self.assertEqual(resolve_pose("open_table", "start", 4), [0.361, 0.357, 0.3282])

    def test_relocated_catalogue_has_same_hash_and_no_external_dependency(self):
        with tempfile.TemporaryDirectory() as directory:
            repo = self.copy_catalogue(directory)
            self.assertEqual(load_pose_catalogue(repo), load_pose_catalogue())
            self.assertEqual(pose_provenance(repo), pose_provenance())
            self.assertEqual(len(pose_source_files(repo)), 7)

    def test_unknown_selection_never_falls_back(self):
        for task, kind, index in [("missing", "start", 1), ("open_table", "initial", 1),
                                  ("open_table", "goal", 0), ("open_table", "goal", True),
                                  ("open_table", "goal", "random")]:
            with self.subTest(task=task, kind=kind, index=index), self.assertRaises(ValueError):
                resolve_pose(task, kind, index)

    def test_missing_and_tampered_source_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            repo = self.copy_catalogue(directory)
            path = repo / "examples/poses/open_table.yaml"
            path.write_text(path.read_text().replace("0.3810", "0.3811"))
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                load_pose_catalogue(repo)
            path.unlink()
            with self.assertRaises(FileNotFoundError):
                pose_source_files(repo)

    def test_bad_pose_content_rejected_even_with_matching_hash(self):
        cases = [
            'starts: {"1": [0, 0, .nan]}\ngoals: {"1": [0, 0, 0]}',
            'starts: {"1": [0, true, 0]}\ngoals: {"1": [0, 0, 0]}',
            'starts: {"1": [0, "0", 0]}\ngoals: {"1": [0, 0, 0]}',
            'starts: {"1": [0, 0]}\ngoals: {"1": [0, 0, 0]}',
            'starts: {1: [0, 0, 0]}\ngoals: {"1": [0, 0, 0]}',
            'starts: {"1": [0, 0, 0], "1": [1, 1, 1]}\ngoals: {"1": [0, 0, 0]}',
            'starts: {}\ngoals: {"1": [0, 0, 0]}',
        ]
        with tempfile.TemporaryDirectory() as directory:
            repo = self.copy_catalogue(directory)
            for text in cases:
                with self.subTest(text=text):
                    self.replace_pose_file(repo, text)
                    with self.assertRaises(ValueError):
                        load_pose_catalogue(repo)

    def test_common_ids_require_same_source_order(self):
        with tempfile.TemporaryDirectory() as directory:
            repo = self.copy_catalogue(directory)
            path = repo / "examples/poses/open_table.yaml"
            data = yaml.safe_load(path.read_text())
            data["goals"] = dict(reversed(list(data["goals"].items())))
            self.replace_pose_file(repo, yaml.safe_dump(data, sort_keys=False))
            self.assertEqual(pose_ids("goal", "open_table", repo), ("5", "4", "3", "2", "1"))
            with self.assertRaisesRegex(ValueError, "ordering differs"):
                pose_ids("goal", repo=repo)

    def test_snapshot_is_scoped_and_returns_independent_values(self):
        with tempfile.TemporaryDirectory() as directory:
            repo = self.copy_catalogue(directory)
            original = load_pose_catalogue(repo)
            with pose_catalogue_snapshot(repo):
                returned = load_pose_catalogue(repo)
                returned["open_table"]["starts"]["1"][0] = 100
                self.assertEqual(load_pose_catalogue(repo), original)
                (repo / "examples/poses/open_table.yaml").write_text("changed")
                with pose_catalogue_snapshot(repo):
                    self.assertEqual(load_pose_catalogue(repo), original)
                self.assertEqual(load_pose_catalogue(repo), original)
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                load_pose_catalogue(repo)

    def test_snapshot_releases_on_error(self):
        with tempfile.TemporaryDirectory() as directory:
            repo = self.copy_catalogue(directory)
            with self.assertRaisesRegex(RuntimeError, "interrupted"):
                with pose_catalogue_snapshot(repo):
                    (repo / "examples/poses/open_table.yaml").unlink()
                    raise RuntimeError("interrupted")
            with self.assertRaises(FileNotFoundError):
                load_pose_catalogue(repo)


if __name__ == "__main__":
    unittest.main()
