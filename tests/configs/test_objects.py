import json
from pathlib import Path
import shutil
import tempfile
import unittest
import yaml

from c3plus import configs as S

from tests.fixtures.results import ObjectFixtures

class ObjectRunTests(ObjectFixtures, unittest.TestCase):
    def test_catalog_rejects_old_t_name_and_incompatible_canonical_profiles(self):
        self.assertIn("T_block", S.OBJECTS)
        self.assertNotIn("Tblock", S.OBJECTS)
        for scene, name in (("open_task", "Tblock"), ("icra_sign", "T_block"), ("open_task", "Cblock")):
            with self.subTest(scene=scene, object=name), self.assertRaises(ValueError):
                S.resolve_object_profile(scene, name)


    def test_model_snapshots_are_self_contained_and_keep_contact_properties(self):
        try:
            from pydrake.geometry import Convex, Sphere
            from pydrake.multibody.parsing import Parser
            from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
            from pydrake.systems.framework import DiagramBuilder
        except ImportError as exc:
            self.skipTest(f"Drake runtime unavailable: {exc}")
        for name in S.MESH_OBJECTS:
            with self.subTest(object=name), tempfile.TemporaryDirectory() as tmp:
                directory = Path(tmp) / "config"
                demo = S.demo_name("open_task", 2, 2, name)
                profile = S.resolve_object_profile("open_task", name)
                controller_file = S.write_demo_configs(demo, directory)
                controller = yaml.safe_load(controller_file.read_text())
                simulation = yaml.safe_load((directory / "simulation.yaml").read_text())
                channels = yaml.safe_load(Path(controller["lcm_channels_simulation_file"]).read_text())
                self.assertEqual(channels["object_state_channels"][0], f"OBJECT_{name}_base_STATE_SIMULATION")
                self.assertTrue(Path(controller["sampling_mesh_files"][0]).is_relative_to(directory))
                for role, config in (("simulation", simulation), ("controller", controller)):
                    model_file = Path(config["object_models"][0])
                    self.assertTrue(model_file.is_relative_to(directory))
                    self.assertIn("drake:declare_convex", model_file.read_text())
                    builder = DiagramBuilder()
                    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
                    instance, = Parser(plant).AddModels(str(model_file))
                    plant.Finalize()
                    context = plant.CreateDefaultContext()
                    body = plant.GetBodyByName(profile["object_body_name"], instance)
                    self.assertAlmostEqual(body.get_mass(context), 0.1)
                    ids = plant.GetCollisionGeometriesForBody(body)
                    inspector = scene_graph.model_inspector()
                    pieces = profile["physics"]["collision_piece_count"]
                    self.assertEqual(len(ids), pieces + (3 if role == "controller" else 0))
                    self.assertTrue(all(isinstance(inspector.GetShape(g), Convex) for g in ids[:pieces]))
                    for gid in ids[:pieces]:
                        friction = inspector.GetProximityProperties(gid).GetProperty("material", "coulomb_friction")
                        self.assertAlmostEqual(friction.static_friction(), 0.3)
                    if role == "controller":
                        self.assertTrue(all(isinstance(inspector.GetShape(g), Sphere) for g in ids[-3:]))
                    self.assertEqual(context.get_time(), 0)


    def test_selected_geometry_and_physics_change_the_configuration_digest(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            name = "banana"
            demo = S.demo_name("open_task", 2, 2, name)
            resolved = S.compose_demo_configs(demo)
            profile = S.resolve_object_profile("open_task", name)
            for source in [*S.load_demo_configs(demo), *S.model_assets(resolved),
                           S.REPO / profile["physics_metadata_file"]]:
                target = repo / source.relative_to(S.REPO)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
            original = S.demo_config_digest(demo, repo)
            mesh = repo / resolved["controller"]["sampling_mesh_files"][0]
            mesh.write_text(mesh.read_text() + "\n# Changed source revision\n")
            changed_mesh = S.demo_config_digest(demo, repo)
            self.assertNotEqual(original, changed_mesh)
            metadata_file = repo / profile["physics_metadata_file"]
            metadata = json.loads(metadata_file.read_text())
            metadata["mass_kg"] = 0.2
            metadata_file.write_text(json.dumps(metadata))
            self.assertNotEqual(changed_mesh, S.demo_config_digest(demo, repo))
