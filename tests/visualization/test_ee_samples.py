import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np

from c3plus.visualization import objects as V

from tests.fixtures.results import MeshPreviewFixtures

class MeshPreviewTests(MeshPreviewFixtures, unittest.TestCase):
    def test_ee_candidate_clearance_and_coordinate_transforms(self):
        try:
            import pydrake.common
            import numpy as np
            from pydrake.math import RollPitchYaw
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")

        def box_distance(points, center, half_size):
            offset = np.abs(points - center) - half_size
            return np.linalg.norm(np.maximum(offset, 0), axis=1) + np.minimum(offset.max(axis=1), 0)

        for scene in ("open_task", "icra_sign"):
            for yaw in (90, 0, -90):
                with self.subTest(scene=scene, yaw=yaw):
                    settings = self.settings("--scene", scene, "--pose", "goal", "--goal", "2",
                                             "--goal-yaw-degrees", str(yaw), "--ee-samples", "32")
                    first = V.sample_ee_candidates(settings, 32, 42)
                    self.assertEqual(first, V.sample_ee_candidates(settings, 32, 42))
                    world, local = np.array(first["points_world"]), np.array(first["points_object"])
                    rotation = RollPitchYaw(0, 0, math.radians(yaw)).ToRotationMatrix().matrix()
                    np.testing.assert_allclose(local @ rotation.T + settings["position_m"], world, atol=1e-12)
                    self.assertEqual(world.shape, (32, 3))
                    height = 0.005 if scene == "open_task" else -0.012
                    np.testing.assert_allclose(world[:, 2], height, atol=1e-12)
                    self.assertTrue(np.all((world[:, 0] >= 0.17) & (world[:, 0] <= 0.73)))
                    self.assertTrue(np.all((world[:, 1] >= -0.58) & (world[:, 1] <= 0.58)))
                    radius = np.linalg.norm(world[:, :2], axis=1)
                    self.assertTrue(np.all((radius >= 0.27) & (radius <= 0.68)))
                    if scene == "open_task":
                        distances = np.minimum(
                            box_distance(local, [0, 0.0099, 0], [0.0445, 0.0099, 0.0298]),
                            box_distance(local, [0, -0.0397, 0], [0.0099, 0.0397, 0.0298]))
                        self.assertTrue(np.all(distances - 0.00555 > 0.019 - 1e-12))
                    else:
                        distances = np.minimum.reduce([
                            box_distance(local, [-0.0323, 0, 0], [0.016, 0.0515, 0.0125]),
                            box_distance(local, [0, 0.0355, 0], [0.0483, 0.016, 0.0125]),
                            box_distance(local, [0, -0.0355, 0], [0.0483, 0.016, 0.0125]),
                        ])
                        self.assertTrue(np.all(distances > 0.027))
                    self.assertNotEqual(first["points_world"], V.sample_ee_candidates(settings, 32, 43)["points_world"])


    def test_raw_mesh_ee_samples_preserve_concavity(self):
        try:
            import numpy as np
            import trimesh
            import scipy
        except ImportError as exc:
            self.skipTest(f"Mesh runtime unavailable: {exc}")

        def box_distance(points, center, half_size):
            offset = np.abs(points - center) - half_size
            return np.linalg.norm(np.maximum(offset, 0), axis=1) + np.minimum(offset.max(axis=1), 0)

        # Build a closed concave prism without optional polygon triangulators.
        outline = np.array([[-0.0483, -0.0515], [-0.0163, -0.0515], [0.0483, -0.0515],
                            [0.0483, -0.0195], [-0.0163, -0.0195], [-0.0163, 0.0195],
                            [0.0483, 0.0195], [0.0483, 0.0515], [-0.0163, 0.0515], [-0.0483, 0.0515]])
        triangles = np.array([[0, 1, 4], [0, 4, 5], [0, 5, 8], [0, 8, 9],
                              [1, 2, 3], [1, 3, 4], [5, 6, 7], [5, 7, 8]])
        fixture = trimesh.creation.extrude_triangulation(outline, triangles, 0.025)
        fixture.apply_translation([0, 0, -0.0125])
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        mesh = str(Path(directory.name) / "concave.obj")
        fixture.export(mesh)
        for yaw in (0, 90, -90):
            with self.subTest(yaw=yaw):
                # A doubled C has a cavity wide enough to hold candidate tip
                # spheres; convex-hull distance would incorrectly reject them.
                settings = self.settings("--mesh", mesh, "--mesh-scale", "2", "--ee-samples", "128",
                                         "--position", "0.4", "0", "-0.004", "--rpy-degrees", "0", "0", str(yaw))
                report = V.sample_ee_candidates(settings, 128, 42)
                self.assertEqual(report, V.sample_ee_candidates(settings, 128, 42))
                world, local = np.array(report["points_world"]), np.array(report["points_object"])
                angle = math.radians(yaw)
                rotation = np.array([[math.cos(angle), -math.sin(angle), 0],
                                     [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
                np.testing.assert_allclose(local @ rotation.T + settings["position_m"], world, atol=1e-12)
                np.testing.assert_allclose(world[:, 2], -0.012, atol=1e-12)
                distances = np.minimum.reduce([
                    box_distance(local, [-0.0646, 0, 0], [0.032, 0.103, 0.025]),
                    box_distance(local, [0, 0.071, 0], [0.0966, 0.032, 0.025]),
                    box_distance(local, [0, -0.071, 0], [0.0966, 0.032, 0.025]),
                ])
                self.assertTrue(np.all(distances >= 0.027 - 1e-10))
                self.assertTrue(np.any((np.abs(local[:, 0]) < 0.0966) & (np.abs(local[:, 1]) < 0.103)))
                self.assertTrue(np.all(world[:, 2] - 0.00555 >= -0.029))
                self.assertTrue(np.all((world[:, 0] >= 0.17) & (world[:, 0] <= 0.73)))
                radial = np.linalg.norm(world[:, :2], axis=1)
                self.assertTrue(np.all((radial >= 0.27) & (radial <= 0.68)))
                self.assertNotEqual(report["points_world"], V.sample_ee_candidates(settings, 128, 43)["points_world"])


    def test_ee_overlay_and_json_sidecar(self):
        try:
            import pydrake.common
            import numpy as np
            from PIL import Image
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        with tempfile.TemporaryDirectory() as tmp:
            for scene in ("open_task", "icra_sign"):
                for geometry in ("visual", "collision"):
                    with self.subTest(scene=scene, geometry=geometry):
                        output = Path(tmp) / f"{scene}_{geometry}.png"
                        settings = self.settings("--scene", scene, "--view", "top", "--geometry", geometry,
                                                 "--ee-samples", "24", "--hide-robot", "--output", str(output),
                                                 "--width", "640", "--height", "480")
                        V.render_image(settings)
                        report = json.loads(output.with_suffix(".ee_samples.json").read_text())
                        self.assertEqual(report["count"], 24)
                        self.assertEqual(len(report["points_world"]), 24)
                        self.assertEqual(report["object_name"], settings["object_name"])
                        with Image.open(output) as image:
                            pixels = np.asarray(image)[80:]
                            blue = (pixels[:, :, 2] > 120) & (pixels[:, :, 1] > 70) & (pixels[:, :, 0] < 90)
                            self.assertGreater(int(blue.sum()), 10)
