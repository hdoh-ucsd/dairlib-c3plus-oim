"""Static native collision checks; never construct or advance a Simulator."""
from copy import deepcopy
import importlib.util
import math
import unittest
from unittest.mock import patch

from c3plus.configs import geometry as G
from c3plus.configs.catalog import TASKS, OBJECTS, demo_name
from c3plus.configs.poses import resolve_pose
from c3plus.configs.resolver import compose_demo_configs


def resolved(task='open_table', name='T_shape', start=1, goal=1):
    return compose_demo_configs(demo_name(task, start, goal, name), object_name=name)


class GeometryResolutionTests(unittest.TestCase):
    def test_full_endpoint_expansion_uses_canonical_resolved_values(self):
        instances = []
        class Recorder:
            def __init__(self, config, repo):
                self.poses = []
                instances.append(self)
            def check(self, pose, robot_joints):
                self.poses.append((deepcopy(pose), deepcopy(robot_joints)))
                return []
        with patch.object(G, '_Geometry', Recorder):
            report = G.validate_suite_geometry()
        self.assertTrue(report['valid'])
        self.assertEqual(report['expected_cells'], len(TASKS) * len(OBJECTS))
        self.assertEqual(len(instances), report['expected_cells'])
        self.assertEqual(report['endpoints_checked'], 300)
        self.assertEqual(report['start_poses_checked'], 150)
        self.assertEqual(report['goal_poses_checked'], 150)
        for instance in instances:
            self.assertEqual(len(instance.poses), 10)
            self.assertTrue(all(joints is not None for _, joints in instance.poses[:5]))
            self.assertTrue(all(joints is None for _, joints in instance.poses[5:]))

    def test_failed_cell_is_reported_before_any_launch(self):
        with patch.object(G, '_Geometry', side_effect=ValueError('missing selected asset')):
            report = G.validate_suite_geometry(tasks=['icra_sign'], objects=['banana'])
        self.assertFalse(report['valid'])
        self.assertEqual(report['endpoints_checked'], 0)
        self.assertEqual(report['errors'], [{'task': 'icra_sign', 'object': 'banana',
            'kind': 'geometry_setup', 'message': 'missing selected asset'}])

    def test_pose_substitution_is_rejected(self):
        source = resolve_pose('icra_sign', 'start', '1')
        pose = resolved('icra_sign')['simulation']['q_init_object']
        G._matches_canonical(pose, source, 'pose')
        wrong = list(pose); wrong[4] += .001
        with self.assertRaisesRegex(ValueError, 'XY differs'):
            G._matches_canonical(wrong, source, 'pose')
        wrong = [math.cos(.1), 0, 0, math.sin(.1), *pose[4:]]
        with self.assertRaisesRegex(ValueError, 'orientation differs'):
            G._matches_canonical(wrong, source, 'pose')


@unittest.skipUnless(importlib.util.find_spec('pydrake'), 'Drake required for native geometry queries')
class NativeGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.open_config = resolved()
        cls.icra_config = resolved('icra_sign', 'sugar_box', start=3)
        with G._quiet_parser():
            cls.open_geometry = G._Geometry(cls.open_config)
            cls.icra_geometry = G._Geometry(cls.icra_config)

    def test_source_pose_is_clear_but_shared_alternative_has_real_collision(self):
        native = self.icra_config['simulation']
        self.assertEqual(self.icra_geometry.check(native['q_init_object'], native['q_init_franka']), [])
        x, y, yaw = resolve_pose('open_table', 'start', '3')
        wrong = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2), x, y, native['q_init_object'][-1]]
        errors = self.icra_geometry.check(wrong, native['q_init_franka'])
        collisions = [error for error in errors if error['kind'] == 'obstacle_collision']
        self.assertTrue(collisions)
        self.assertGreater(max(error['depth_m'] for error in collisions), .039)
        self.assertTrue(any('glyph_2b' in error['geometry_a'] + error['geometry_b'] for error in collisions))

    def test_table_bounds_height_and_penetration_are_checked(self):
        simulation = self.open_config['simulation']
        pose, joints = simulation['q_init_object'], simulation['q_init_franka']
        self.assertEqual(self.open_geometry.check(pose, joints), [])
        for axis, delta, expected in ((6, .01, 'table_support'), (6, -.01, 'table_penetration'),
                                       (4, 1., 'table_support')):
            wrong = list(pose); wrong[axis] += delta
            with self.subTest(axis=axis, delta=delta):
                errors = self.open_geometry.check(wrong, joints)
                self.assertIn(expected, {error['kind'] for error in errors})

    def test_start_robot_collision_is_not_applied_to_an_undefined_goal_robot_pose(self):
        geometry = self.open_geometry
        simulation = self.open_config['simulation']
        geometry.check(simulation['q_init_object'], simulation['q_init_franka'])
        tip = geometry.plant.GetBodyByName('end_effector_tip')
        xyz = geometry.plant.EvalBodyPoseInWorld(geometry.plant_context, tip).translation()
        # Place the center of the T crossbar over the actual native tip sphere.
        pose = [1., 0., 0., 0., float(xyz[0]), float(xyz[1]) - .0099, float(xyz[2])]
        errors = geometry.check(pose, simulation['q_init_franka'])
        self.assertIn('initial_robot_collision', {error['kind'] for error in errors})
        goal_errors = geometry.check(pose)
        self.assertNotIn('initial_robot_collision', {error['kind'] for error in goal_errors})


if __name__ == '__main__':
    unittest.main()
