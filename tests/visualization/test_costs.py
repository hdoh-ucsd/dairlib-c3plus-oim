from pathlib import Path
import tempfile
import unittest
import numpy as np
import yaml

from c3plus.visualization import costs as C

from tests.fixtures.results import ObjectFixtures

class ObjectRunTests(ObjectFixtures, unittest.TestCase):
    def test_cost_reconstruction_uses_saved_footprint(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "scene.yaml"
            config.write_text(yaml.safe_dump({"footprint": [[-.1, -.1], [.1, -.1], [.1, .1], [-.1, .1]],
                                              "obstacles": {"discs": [[.1, .1, .01]]}}))
            zeros = np.array([0.0])
            native_scene, native_has_obstacle = C.obs_curve("relu", "open_task", zeros, zeros, zeros)
            selected, selected_has_obstacle = C.obs_curve("relu", "open_task", zeros, zeros, zeros, config)
            self.assertFalse(native_has_obstacle)
            self.assertEqual(native_scene[0], 0)
            self.assertTrue(selected_has_obstacle)
            self.assertGreater(selected[0], 0)
