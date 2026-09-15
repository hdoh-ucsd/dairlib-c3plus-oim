"""Vectorized obstacle diagnostics must preserve the original scalar geometry."""

import math
import unittest

import numpy as np

from c3plus.configs.catalog import evaluation_config
from c3plus.evaluation.postprocess import Obstacles, compute_row, sample_boundary


class ScalarBatchAdapter:
    """Drive the batch caller through the retained, independent scalar oracle."""

    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.batch_calls = 0

    def empty(self):
        return self.obstacles.empty()

    def sdf(self, point):
        return self.obstacles.sdf(point)

    def sdf_batch(self, points):
        self.batch_calls += 1
        return np.asarray([self.obstacles.sdf(point) for point in points])


class ObstacleDistanceTests(unittest.TestCase):
    def assert_scalar_equivalent(self, cfg, points):
        obstacles = Obstacles(cfg)
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        before = points.copy()
        expected = np.asarray([obstacles.sdf(point) for point in points])
        actual = obstacles.sdf_batch(points)
        self.assertEqual(actual.shape, (len(points),))
        np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
        np.testing.assert_array_equal(points, before)
        return actual

    def test_random_concave_and_convex_polygons_in_both_winding_orders(self):
        rng = np.random.default_rng(42)
        points = rng.uniform(-2, 2, (150, 2))
        for n_vertices in (3, 6, 11):
            angles = np.linspace(0, 2*math.pi, n_vertices, endpoint=False) + .173
            for radii in (np.ones(n_vertices), np.where(np.arange(n_vertices) % 2, .35, 1.0)):
                polygon = np.column_stack((radii*np.cos(angles), radii*np.sin(angles)))
                for winding in (polygon, polygon[::-1]):
                    with self.subTest(vertices=n_vertices, winding=winding.tolist()):
                        self.assert_scalar_equivalent({"obstacles": {"polygons": [winding.tolist()]}}, points)

    def test_concave_notch_preserves_inside_and_outside_signs(self):
        # A C-shaped polygon: the notch is outside despite lying in its hull.
        polygon = [[0, 0], [3, 0], [3, 1], [1, 1], [1, 2], [3, 2], [3, 3], [0, 3]]
        points = [[.5, 1.5], [2, 1.5], [2, .5], [2, 2.5], [4, 1.5]]
        values = self.assert_scalar_equivalent({"obstacles": {"polygons": [polygon]}}, points)
        np.testing.assert_array_equal(np.sign(values), [-1, 1, -1, -1, 1])

    def test_exact_vertices_edges_and_horizontal_ray_crossings(self):
        polygon = [[0, 0], [2, 0], [2, 1], [1, 1], [1, 2], [0, 2]]
        vertices = np.asarray(polygon, dtype=float)
        midpoints = (vertices + np.roll(vertices, -1, axis=0))/2
        # Include points exactly on vertex y-coordinates, where > versus >=
        # determines the parity of horizontal-ray intersections.
        points = np.vstack((vertices, midpoints,
                            [[-1, 0], [-1, 1], [-1, 2], [.5, 1], [1.5, 1],
                             [np.nextafter(1., 0.), 1.5], [np.nextafter(1., 2.), 1.5]]))
        values = self.assert_scalar_equivalent({"obstacles": {"polygons": [polygon]}}, points)
        np.testing.assert_array_equal(values[:len(vertices)*2], 0)

    def test_repeated_vertices_and_zero_length_edges(self):
        rng = np.random.default_rng(7)
        points = np.vstack((rng.normal(size=(100, 2)), [[0, 0], [0, 1], [1, 0], [.5, 0]]))
        for polygon in (
                [[0, 0], [0, 0], [1, 0], [1, 1], [1, 1], [0, 1], [0, 0]],
                [[0, 0], [0, 1], [0, 1], [0, 0]],
                [[.1, .2], [.1, .2], [.1, .2]],
                [[0, 0], [1e-10, 0], [1, 1], [0, 1]]):
            with self.subTest(polygon=polygon):
                self.assert_scalar_equivalent({"obstacles": {"polygons": [polygon]}}, points)

    def test_discs_include_center_boundary_and_zero_radius(self):
        cfg = {"obstacles": {"discs": [[1, 2, .5], [-1, -2, 0]]}}
        values = self.assert_scalar_equivalent(cfg, [[1, 2], [1.5, 2], [2, 2], [-1, -2], [-1, -1]])
        np.testing.assert_allclose(values, [-.5, 0, .5, 0, 1])

    def test_mixed_overlapping_obstacles_take_minimum_signed_distance(self):
        cfg = {"obstacles": {"polygons": [[[0, 0], [2, 0], [2, 2], [0, 2]],
                                           [[1, 1], [3, 1], [3, 3], [1, 3]]],
                             "discs": [[1, 1, .8], [4, 4, .2]]}}
        points = np.random.default_rng(123).uniform(-1, 5, (180, 2))
        self.assert_scalar_equivalent(cfg, points)

    def test_empty_obstacles_and_empty_queries(self):
        for cfg in ({}, {"obstacles": {}}, {"obstacles": {"polygons": [], "discs": []}}):
            with self.subTest(cfg=cfg):
                values = self.assert_scalar_equivalent(cfg, [[0, 0], [1, -1]])
                self.assertTrue(np.isposinf(values).all())
                self.assert_scalar_equivalent(cfg, [])
        self.assert_scalar_equivalent({"obstacles": {"polygons": [[[0, 0], [1, 0], [0, 1]]],
                                                     "discs": [[0, 0, .2]]}}, [])

    def test_checked_in_icra_sign_glyphs(self):
        cfg = evaluation_config("icra_sign", "T_shape")
        points = np.random.default_rng(2026).uniform([-.15, -.7], [.7, .75], (150, 2))
        vertices = np.concatenate([np.asarray(polygon) for polygon in cfg["obstacles"]["polygons"]])
        self.assert_scalar_equivalent(cfg, np.vstack((points, vertices)))


class ComputeRowObstacleTests(unittest.TestCase):
    def assert_row_equivalent(self, cfg, x, y, yaw, k, *, table=None, previous=True):
        obstacles = Obstacles(cfg)
        adapter = ScalarBatchAdapter(obstacles)
        geo = {"footprint": cfg["footprint"],
               "boundary": sample_boundary(cfg["footprint"], cfg["boundary_sample_spacing"]),
               "obstacles": obstacles, "table": table}
        args = (k, x, y, yaw, x-.075, y+.012, cfg["tip_target_z"]+.004,
                np.diag([1., -1., -1.]), cfg["goal"], cfg)
        q = [.1, -.2, .3, -.4, .5, -.6]
        q_prev = [value-.001 for value in q] if previous else None
        actual = compute_row(*args, geo, q, q_prev, .013)
        expected = compute_row(*args, {**geo, "obstacles": adapter}, q, q_prev, .013)
        self.assertEqual(adapter.batch_calls, 0 if obstacles.empty() else 1)
        self.assertEqual(actual[2].keys(), expected[2].keys())
        for key in actual[2]:
            with self.subTest(cost=key):
                np.testing.assert_allclose(actual[2][key], expected[2][key], rtol=1e-12, atol=1e-12)
        for index in (0, 1, 3, 4, 5, 6):
            with self.subTest(return_index=index):
                np.testing.assert_allclose(actual[index], expected[index], rtol=1e-12, atol=1e-12)

    def test_icra_sign_full_diagnostic_rows_match_scalar_oracle(self):
        cfg = evaluation_config("icra_sign", "T_shape")
        for k, (x, y, yaw) in enumerate(((.5, -.55, 0.), (.35, -.35, .7), (.5, .28, -1.2))):
            with self.subTest(x=x, y=y, yaw=yaw):
                self.assert_row_equivalent(cfg, x, y, yaw, 10+k,
                                          table=([.45, 0], [.35, .8]), previous=bool(k))

    def test_open_table_keeps_zero_obstacle_cost_and_missing_clearance(self):
        cfg = evaluation_config("open_table", "T_shape")
        cfg["obstacles"] = {}
        self.assert_row_equivalent(cfg, .3, -.3, .2, 4, previous=False)


if __name__ == "__main__":
    unittest.main()
