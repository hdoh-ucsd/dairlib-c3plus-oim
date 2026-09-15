"""Check prescribed experiment poses against native geometry without stepping physics."""
from __future__ import annotations

import math
import logging
from contextlib import contextmanager
from pathlib import Path

from .paths import REPO

# Numerical tolerance for geometry contact queries, not an evaluation threshold.
GEOMETRY_TOLERANCE_M = 1e-6


@contextmanager
def _quiet_parser():
    # MJCF warnings about ignored actuator/force settings do not affect this
    # geometry-only check. Restore the caller's logging level immediately.
    logger = logging.getLogger("drake")
    previous = logger.level
    def errors_only(record):
        return record.levelno >= logging.ERROR
    logger.setLevel(logging.ERROR)
    logger.addFilter(errors_only)
    try:
        yield
    finally:
        logger.removeFilter(errors_only)
        logger.setLevel(previous)


def _finite_vector(value, size, label):
    if (not isinstance(value, (list, tuple)) or len(value) != size
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                   for v in value)):
        raise ValueError(f"{label} must contain {size} finite numbers")
    return value


def _matches_canonical(pose, canonical, label):
    """Check the actual resolved pose, including full quaternion orientation."""
    _finite_vector(pose, 7, label)
    _finite_vector(canonical, 3, "Canonical pose")
    if list(pose[4:6]) != list(canonical[:2]):
        raise ValueError(f"{label} XY differs from the local canonical pose")
    norm = math.sqrt(sum(v * v for v in pose[:4]))
    if norm == 0:
        raise ValueError(f"{label} has a zero quaternion")
    expected = [math.cos(canonical[2] / 2), 0., 0., math.sin(canonical[2] / 2)]
    q = [v / norm for v in pose[:4]]
    if min(sum((a - sign * b) ** 2 for a, b in zip(q, expected)) for sign in (-1, 1)) > 1e-24:
        raise ValueError(f"{label} orientation differs from the local canonical pose")


class _Geometry:
    """One native task/object plant reused for its start and goal contexts."""

    def __init__(self, resolved, repo=REPO):
        # Help, catalogue inspection and manifest construction do not import Drake.
        import numpy as np
        import yaml
        from pydrake.geometry import Box, Convex, Mesh
        from pydrake.math import RigidTransform
        from pydrake.multibody.parsing import Parser
        from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
        from pydrake.systems.framework import DiagramBuilder

        self.np = np
        self.Box = Box
        self.RigidTransform = RigidTransform
        repo = Path(repo)
        controller, simulation = resolved["controller"], resolved["simulation"]
        scenario = yaml.safe_load((repo / controller["scenario_params_file"]).read_text())
        options = yaml.safe_load((repo / controller["sampling_c3_options_file"]).read_text())
        if options.get("include_walls"):
            raise ValueError("Geometry preflight requires an explicit model for include_walls=true")
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=float(simulation["dt"]))
        parser = Parser(self.plant, self.scene_graph)
        urdf = repo / "examples/sampling_c3/urdf"
        # Geometry/welds match native AddXarm6ToPlant (sampling_c3_utils.cc).
        self.arm, = parser.AddModels(str(urdf / "oim_xarm6_tabletop/xarm6/xarm6_policyport.xml"))
        tool, = parser.AddModels(str(urdf / "end_effector_xarm6_stick.urdf"))
        self.plant.WeldFrames(self.plant.GetFrameByName("xarm6_link6", self.arm),
                              self.plant.GetFrameByName("end_effector_flange", tool), RigidTransform())
        ground, = parser.AddModels(str(urdf / "ground_oim_xarm6.urdf"))
        platform, = parser.AddModels(str(urdf / "platform.urdf"))
        for model, frame, z in ((ground, "ground", -.029), (platform, "platform", -.0145)):
            self.plant.WeldFrames(self.plant.GetFrameByName("xarm6_link_base", self.arm),
                                  self.plant.GetFrameByName(frame, model), RigidTransform([0, 0, z]))
        if scenario.get("obstacle_model"):
            parser.AddModels(str(repo / scenario["obstacle_model"]))
        model, = parser.AddModels(str(repo / simulation["object_model"]))
        self.body = self.plant.GetBodyByName(controller["object_body_name"], model)
        self.plant.Finalize()
        self.diagram = builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyMutableContextFromRoot(self.context)
        self.inspector = self.scene_graph.model_inspector()

        def ids(instance):
            return {gid for index in self.plant.GetBodyIndices(instance)
                    for gid in self.plant.GetCollisionGeometriesForBody(self.plant.get_body(index))}

        self.object_ids = ids(model)
        self.robot_ids = ids(self.arm) | ids(tool)
        self.table_ids = ids(ground)
        if not self.object_ids or len(self.table_ids) != 1:
            raise ValueError("Expected object collision geometry and one native tabletop box")
        self.vertices = {}
        for gid in self.object_ids:
            shape = self.inspector.GetShape(gid)
            if isinstance(shape, Box):
                values = self._box_vertices(shape)
            elif isinstance(shape, (Convex, Mesh)):
                hull = shape.GetConvexHull()
                values = np.asarray([hull.vertex(i) for i in range(hull.num_vertices())])
            else:
                raise ValueError(f"Unsupported object collision shape: {type(shape).__name__}")
            self.vertices[gid] = values
        table_id = next(iter(self.table_ids))
        table_shape = self.inspector.GetShape(table_id)
        if not isinstance(table_shape, Box):
            raise ValueError("Native table collision geometry must be a box")
        query = self._query()
        corners = (query.GetPoseInWorld(table_id) @ self._box_vertices(table_shape).T).T
        self.table_min, self.table_max = corners.min(axis=0), corners.max(axis=0)

    def _box_vertices(self, box):
        from itertools import product
        return self.np.asarray(list(product((-box.width() / 2, box.width() / 2),
                                            (-box.depth() / 2, box.depth() / 2),
                                            (-box.height() / 2, box.height() / 2))))

    def _query(self):
        return self.scene_graph.get_query_output_port().Eval(
            self.scene_graph.GetMyContextFromRoot(self.context))

    def check(self, pose, robot_joints=None):
        """Return errors for an object pose; robot checks apply only to starts."""
        from pydrake.common.eigen_geometry import Quaternion
        _finite_vector(pose, 7, "Object pose")
        q = self.np.asarray(pose[:4], dtype=float)
        norm = self.np.linalg.norm(q)
        if norm == 0:
            raise ValueError("Object pose has a zero quaternion")
        self.plant.SetFreeBodyPose(self.plant_context, self.body,
                                  self.RigidTransform(Quaternion(q / norm), pose[4:]))
        if robot_joints is not None:
            _finite_vector(robot_joints, 5, "Robot initial configuration")
            self.plant.SetPositions(self.plant_context, self.arm, self.np.asarray(robot_joints))
        query = self._query()
        errors = []
        for pair in query.ComputePointPairPenetration():
            if pair.depth <= GEOMETRY_TOLERANCE_M:
                continue
            a_object, b_object = pair.id_A in self.object_ids, pair.id_B in self.object_ids
            if a_object == b_object:
                continue
            other = pair.id_B if a_object else pair.id_A
            if other in self.robot_ids and robot_joints is None:
                continue  # No robot goal configuration is defined by the pose catalogue.
            kind = ("table_penetration" if other in self.table_ids else
                    "initial_robot_collision" if other in self.robot_ids else "obstacle_collision")
            errors.append({"kind": kind, "geometry_a": self.inspector.GetName(pair.id_A),
                           "geometry_b": self.inspector.GetName(pair.id_B), "depth_m": float(pair.depth),
                           "message": f"{kind}: native collision geometries penetrate by {pair.depth:.9g} m"})
        world = self.np.concatenate([(query.GetPoseInWorld(gid) @ vertices.T).T
                                    for gid, vertices in self.vertices.items()])
        low, high = world.min(axis=0), world.max(axis=0)
        outside = any(low[i] < self.table_min[i] - GEOMETRY_TOLERANCE_M
                      or high[i] > self.table_max[i] + GEOMETRY_TOLERANCE_M for i in (0, 1))
        if outside or abs(low[2] - self.table_max[2]) > GEOMETRY_TOLERANCE_M:
            errors.append({"kind": "table_support", "bounds_min": low.tolist(), "bounds_max": high.tolist(),
                           "table_min": self.table_min.tolist(), "table_max": self.table_max.tolist(),
                           "message": "Object collision geometry must lie within the tabletop and rest on its support plane"})
        return errors


def validate_suite_geometry(tasks=None, objects=None, *, repo=REPO):
    """Validate all unique endpoints before launch; return a JSON-compatible report.

    Every task/object plant is built once and reused for all endpoint checks.
    No files, dynamics, random samples, or planner state are created. Failures
    name the exact task, object, pose role/ID and native collision geometry.
    """
    from .catalog import configuration_snapshot
    with configuration_snapshot(repo=repo):
        return _validate_suite_geometry(tasks, objects, repo=repo)


def _validate_suite_geometry(tasks, objects, *, repo):
    from .catalog import TASKS, OBJECTS, canonical_task, canonical_object, demo_name
    from .poses import load_pose_catalogue
    from .resolver import compose_demo_configs

    tasks = tuple(canonical_task(task) for task in (TASKS if tasks is None else tasks))
    objects = tuple(canonical_object(name) for name in (OBJECTS if objects is None else objects))
    if len(set(tasks)) != len(tasks) or len(set(objects)) != len(objects):
        raise ValueError("Geometry selections must not contain duplicates")
    catalogues = load_pose_catalogue(repo=repo)
    report = {"valid": True, "task_count": len(tasks), "object_count": len(objects),
              "expected_cells": len(tasks) * len(objects), "cells_checked": 0,
              "endpoints_checked": 0, "start_poses_checked": 0, "goal_poses_checked": 0,
              "numerical_tolerance_m": GEOMETRY_TOLERANCE_M, "cells": [], "errors": []}
    for task in tasks:
        poses = catalogues[task]
        starts = sorted(poses["starts"], key=int)
        goals = sorted(poses["goals"], key=int)
        for name in objects:
            cell = {"task": task, "object": name, "starts_checked": 0, "goals_checked": 0, "valid": True}
            try:
                first = compose_demo_configs(demo_name(task, int(starts[0]), int(goals[0]), name),
                                             repo=repo, object_name=name)
                configurations = {(starts[0], goals[0]): first}
                with _quiet_parser():
                    geometry = _Geometry(first, repo)
                for role, indices in (("starts", starts), ("goals", goals)):
                    for index in indices:
                        start, goal = (index, goals[0]) if role == "starts" else (starts[0], index)
                        key = (start, goal)
                        if key not in configurations:
                            configurations[key] = compose_demo_configs(demo_name(task, int(start), int(goal), name),
                                                                        repo=repo, object_name=name)
                        resolved = configurations[key]
                        pose = (resolved["simulation"]["q_init_object"] if role == "starts" else
                                [*resolved["goal"]["fixed_target_orientation"], *resolved["goal"]["fixed_target_position"]])
                        _matches_canonical(pose, poses[role][index], f"{task}/{name}/{role}/{index}")
                        joints = resolved["simulation"]["q_init_franka"] if role == "starts" else None
                        errors = geometry.check(pose, joints)
                        for error in errors:
                            report["errors"].append({"task": task, "object": name, "role": role,
                                                     "pose_id": index, "pose": list(poses[role][index]), **error})
                        cell[role + "_checked"] += 1
                        report[("start" if role == "starts" else "goal") + "_poses_checked"] += 1
                        report["endpoints_checked"] += 1
                        if errors:
                            cell["valid"] = False
            except (ImportError, RuntimeError, ValueError, OSError, KeyError, TypeError) as exc:
                cell["valid"] = False
                report["errors"].append({"task": task, "object": name, "kind": "geometry_setup",
                                         "message": str(exc)})
            report["cells_checked"] += 1
            report["cells"].append(cell)
    report["valid"] = not report["errors"]
    return report
