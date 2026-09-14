from contextlib import redirect_stderr
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from c3plus import configs as S
from tools import __main__ as cli
from c3plus.visualization import objects as V

from tests.fixtures.results import MeshPreviewFixtures

class MeshPreviewTests(MeshPreviewFixtures, unittest.TestCase):
    def test_help_without_drake(self):
        result = subprocess.run(
            [sys.executable, "-S", "-m", "tools.visualize", "--help"],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--mesh", result.stdout)


    def test_renamed_cli_dry_run_creates_no_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = Path(tmp) / "new" / "object.png"
            result = subprocess.run(
                [sys.executable, "-m", "tools", "visualize",
                 "--output", str(image), "--view", "top", "--ee-samples", "--dry-run"],
                cwd=S.REPO, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            settings = json.loads(result.stdout)
            self.assertEqual(settings["output"], str(image))
            self.assertEqual(settings["view"], "top")
            self.assertEqual(settings["ee_sampling"]["count"], 64)
            self.assertEqual(settings["ee_sampling"]["seed"], 42)
            self.assertFalse(image.parent.exists())
            self.assertNotIn("visualize_mesh", cli.COMMANDS)


    def test_independent_start_and_goal_placements(self):
        start = self.settings("--scene", "open_task", "--start", "2", "--goal", "5")
        self.assertEqual(start["position_m"], [0.366, 0.431, 0.0008])
        goal = self.settings("--scene", "open_task", "--start", "5", "--pose", "goal",
                             "--goal", "2", "--goal-yaw-degrees", "-90")
        self.assertEqual(goal["position_m"][:2], [0.397, -0.431])
        self.assertAlmostEqual(goal["quaternion_wxyz"][0], math.sqrt(0.5))
        self.assertAlmostEqual(goal["quaternion_wxyz"][3], -math.sqrt(0.5))
        self.assertTrue(self.settings("--scene", "icra_sign")["object_file"].endswith("push_c_glyph.sdf"))


    def test_invalid_preview_options_fail_before_viewer(self):
        mesh = str(S.REPO / "examples/sampling_c3/urdf/c_glyph_base/c_glyph_base.obj")
        cases = (("--mesh-scale", "0.001"), ("--mesh", mesh, "--mesh-scale", "0"),
                 ("--mesh", mesh, "--mesh-scale", "nan"), ("--position", "nan", "0", "0"),
                 ("--goal-yaw-degrees", "90"), ("--width", "0"), ("--height", "-1"),
                 ("--output", "image.jpg"), ("--mesh", mesh, "--geometry", "collision"),
                 ("--ee-samples", "0"), ("--ee-samples", "10001"),
                 ("--sample-seed", "42"), ("--ee-samples", "--sample-seed", "-1"),
                 ("--sample-height", "-0.012"),
                 ("--mesh", mesh, "--ee-samples", "--position", "0.4", "0", "0", "--sample-height", "nan"),
                 ("--mesh", mesh, "--ee-samples"),
                 ("--model", str(S.REPO / "examples/sampling_c3/urdf/push_c_glyph.sdf"), "--ee-samples"),
                 ("--model", mesh), ("--mesh", "/missing/object.obj"),
                 ("--pose", "goal", "--goal-yaw-degrees", "90", "--rpy-degrees", "0", "0", "0"))
        for flags in cases:
            with self.subTest(flags=flags), patch.object(V, "render_image") as viewer, \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                V.main(list(flags))
            self.assertEqual(error.exception.code, 1)
            viewer.assert_not_called()


    def test_geometry_and_camera_poses_without_dynamics(self):
        try:
            import numpy as np
            from pydrake.common.eigen_geometry import Quaternion
            from pydrake.math import RollPitchYaw, RotationMatrix
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        with tempfile.TemporaryDirectory() as tmp:
            # An open mesh needs no volume for a visual preview. Its unused
            # normal declaration must not prevent repairing face normals.
            mesh = Path(tmp) / 'open & "quoted" mesh.obj'
            mesh.write_text("v 0 0 0\nv 0.01 0 0\nv 0 0.01 0\nvn 0 0 1\nf 1 2 3\n")
            mjcf = Path(tmp) / "object.xml"
            mjcf.write_text('<mujoco model="preview_test"><worldbody><body name="test_object">'
                            '<freejoint/><geom type="box" size=".01 .02 .03"/>'
                            '</body></worldbody></mujoco>')
            cases = [(self.settings("--scene", scene, "--start", "2"),
                      "c_glyph_base" if scene == "icra_sign" else "vertical_link")
                     for scene in S.SCENES]
            for scene, body in (("open_task", "vertical_link"), ("icra_sign", "c_glyph_base")):
                for yaw in (90, 0, -90):
                    cases.append((self.settings("--scene", scene, "--pose", "goal", "--goal", "2",
                                                "--goal-yaw-degrees", str(yaw)), body))
            cases.extend([
                (self.settings("--mesh", str(mesh), "--mesh-scale", "0.001",
                               "--position", "20", "-11", "0.0008", "--rpy-degrees", "10", "20", "30"), "object"),
                (self.settings("--model", str(mjcf)), "test_object"),
            ])
            for settings, body_name in cases:
                settings.update(width=320, height=240, frames=True)
                with self.subTest(scene=settings["scene"], pose=settings["pose"], model=body_name), \
                        tempfile.TemporaryDirectory(dir=tmp) as assets:
                    diagram, plant, sensor = V.build_preview(settings, assets)
                    context = diagram.CreateDefaultContext()
                    plant_context = plant.GetMyMutableContextFromRoot(context)
                    body = plant.GetBodyByName(body_name)
                    pose = plant.EvalBodyPoseInWorld(plant_context, body)
                    np.testing.assert_allclose(pose.translation(), settings["position_m"], atol=1e-8, rtol=0)
                    if settings["rpy_degrees"] is None:
                        quat = np.asarray(settings["quaternion_wxyz"])
                        expected = RotationMatrix(Quaternion(quat / np.linalg.norm(quat)))
                    else:
                        expected = RollPitchYaw(np.radians(settings["rpy_degrees"])).ToRotationMatrix()
                    np.testing.assert_allclose(pose.rotation().matrix(), expected.matrix(), atol=5e-8, rtol=0)
                    for index, angle in enumerate(settings["robot_joints_rad"], 1):
                        self.assertAlmostEqual(plant.GetJointByName(f"xarm6_joint{index}").get_angle(plant_context), angle)
                    for name, height in (("ground", -0.029), ("platform", -0.0145)):
                        position = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName(name)).translation()
                        np.testing.assert_allclose(position, [0, 0, height], atol=1e-12)
                    self.assertEqual(context.get_time(), 0)
                    pixels = sensor.color_image_output_port().Eval(sensor.GetMyContextFromRoot(context)).data
                    self.assertEqual(pixels.shape, (240, 320, 4))
                    self.assertGreater(np.unique(pixels[:, :, :3].reshape(-1, 3), axis=0).shape[0], 10)


    def test_png_views_and_collision_output(self):
        try:
            import pydrake.common
            import numpy as np
            from PIL import Image
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        with tempfile.TemporaryDirectory() as tmp:
            for scene, view, geometry in (("open_task", "scene", "visual"),
                                          ("icra_sign", "object", "visual"),
                                          ("ycb_clutter", "top", "collision")):
                with self.subTest(scene=scene, view=view, geometry=geometry):
                    output = Path(tmp) / "images" / f"{scene}.png"
                    settings = self.settings("--scene", scene, "--view", view, "--geometry", geometry,
                                             "--frames", "--width", "320", "--height", "240",
                                             "--output", str(output))
                    self.assertEqual(V.render_image(settings), output)
                    with Image.open(output) as image:
                        self.assertEqual(image.format, "PNG")
                        self.assertEqual(image.size, (320, 240))
                        colors = np.asarray(image)[48:, :, :]
                        self.assertGreater(np.unique(colors.reshape(-1, 3), axis=0).shape[0], 10)
