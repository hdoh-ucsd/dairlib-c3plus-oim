import argparse
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


from tests.fixtures.results import WorkflowFixtures, ArtifactFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_renderer_removes_own_temporary_files_on_failure(self):
        renderer = self.renderer()
        created = []

        def fail(args):
            created.extend([Path(args.assets_tmp), Path(args.frames_dir)])
            for path in created:
                (path / "generated-file").write_text("test")
            raise RuntimeError("encoding failed")

        with patch.object(renderer, "_render_trace", side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, "encoding failed"):
                renderer.render_trace(argparse.Namespace(assets_tmp=None, frames_dir=None))
        self.assertEqual(len(created), 2)
        self.assertTrue(all(not path.exists() for path in created))


    def test_renderer_preserves_requested_temporary_files(self):
        renderer = self.renderer()
        with tempfile.TemporaryDirectory() as tmp:
            args = argparse.Namespace(assets_tmp=tmp, frames_dir=tmp)
            with patch.object(renderer, "_render_trace"):
                renderer.render_trace(args)
            self.assertTrue(Path(tmp).is_dir())


class RunArtifactTests(ArtifactFixtures, unittest.TestCase):
    def test_render_obstacle_fallback_reads_new_and_legacy_native_snapshots(self):
        try:
            from c3plus.visualization import render as renderer
        except ModuleNotFoundError as exc:
            if exc.name == "pydrake":
                self.skipTest("Drake unavailable for importing the renderer")
            raise
        configurations = {"config/simulation.yaml": {"scenario_name": "saved-scene",
                                                     "obstacle_model": "saved-obstacle.sdf"}}
        variants = ({"hyperparameters": {"c3plus": {"configurations": configurations}}},
                    {"provenance": {"c3plus": {"configurations": configurations}}},
                    {"provenance": {"configuration": {"files": {
                        name: {"data": value} for name, value in configurations.items()}}}})
        trace = [{"t": 1., "q": [0.] * 5, "obj": [1., 0., 0., 0., .4, .1, .03]}]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "result.json"
            for variant in variants:
                with self.subTest(layout=variant):
                    path.write_text(json.dumps(variant))
                    args = SimpleNamespace(result=str(path), object_sdf="object.sdf")
                    with patch.object(renderer, "load_result_trace", return_value=trace), \
                            patch.object(renderer, "resolve_result_model", return_value="resolved-obstacle.sdf") as resolve:
                        rows, obj, obstacle, goal = renderer.load_render_input(args)
                    self.assertEqual((rows, obj, obstacle, goal), (trace, "object.sdf", "resolved-obstacle.sdf", None))
                    resolve.assert_called_once_with("saved-obstacle.sdf", str(path), variant)
