#!/usr/bin/env python3
"""Save a PNG of an object in simulation coordinates using Drake.

Loads the configured robot, tool, table, platform, obstacles, and object at its
start or goal pose. Optional model/OBJ overrides are previews; experiment
configurations and mesh files are never rewritten. No dynamics are advanced
and no browser or server is needed.
"""
import argparse
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

if __package__:
    from .catalog import EXPERIMENTS_FILE, REPO, SCENES, compose_demo_configs, demo_name, load_demo_configs
else:
    from catalog import EXPERIMENTS_FILE, REPO, SCENES, compose_demo_configs, demo_name, load_demo_configs


def sample_ee_candidates(settings, count, seed):
    """Sample the active T/C geometric proposals at a supplied object pose.

    Follows examples/sampling_c3/generate_samples.cc:314-389 (T perimeter),
    :581-773 (C mesh normals), :790-803 (workspace), :836-900 (projection),
    systems/controllers/sampling_based_c3_controller.cc:1526-1584 (mesh faces),
    and franka_sampling_c3_controller.cc:259-298 (T EE/object contact pairs).
    The NumPy RNG is a repeatable preview stream, not the native C++ stream.
    Object geometry and workspace checks are reproduced; runtime history,
    predicted states, mode switching, reachability and cost selection are not.
    """
    if settings["raw_mesh"]:
        return sample_raw_mesh_ee_candidates(settings, count, seed)
    import numpy as np
    import yaml
    from pydrake.common.eigen_geometry import Quaternion
    from pydrake.geometry import Box, ReadObjToTriangleSurfaceMesh
    from pydrake.math import RigidTransform, RollPitchYaw
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.framework import DiagramBuilder

    if isinstance(count, bool) or int(count) != count or not 1 <= count <= 10000:
        raise ValueError("EE sample count must be an integer between 1 and 10000")
    if isinstance(seed, bool) or int(seed) != seed or seed < 0:
        raise ValueError("EE sample seed must be a nonnegative integer")
    count, seed = int(count), int(seed)
    config = settings["ee_sampling"]
    with Path(config["params_file"]).open() as stream:
        params = yaml.safe_load(stream)
    with Path(config["options_file"]).open() as stream:
        options = yaml.safe_load(stream)
    strategy = int(params["sampling_strategy"])
    if strategy not in (4, 7):
        raise ValueError("EE previews support the active perimeter (4) and mesh-normal (7) samplers")
    if options["include_walls"]:
        raise ValueError("EE previews currently require include_walls=false")
    if strategy == 7 and not params["gen_planar_samples"]:
        raise ValueError("Mesh-normal EE previews currently require gen_planar_samples=true")
    clearance = float(params["sample_projection_clearance"])
    radius = float(config["ee_radius"])
    position = np.asarray(settings["position_m"], dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Object position must contain three finite coordinates")
    if settings["rpy_degrees"] is not None:
        X_WO = RigidTransform(RollPitchYaw(np.radians(settings["rpy_degrees"])), position)
    else:
        quaternion = np.asarray(settings["quaternion_wxyz"], dtype=float)
        if (quaternion.shape != (4,) or not np.all(np.isfinite(quaternion))
                or np.linalg.norm(quaternion) == 0):
            raise ValueError("Object quaternion must be finite and nonzero")
        X_WO = RigidTransform(Quaternion(quaternion / np.linalg.norm(quaternion)), position)
    rotation = X_WO.rotation().matrix()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    models = Parser(plant).AddModels(str(config["controller_model"]))
    if len(models) != 1:
        raise ValueError("EE preview requires one controller object model")
    model = models[0]
    children = {plant.get_joint(j).child_body().index() for j in plant.GetJointIndices(model)}
    roots = [plant.get_body(b) for b in plant.GetBodyIndices(model) if b not in children]
    if len(roots) != 1:
        raise ValueError("EE preview requires one free object root")
    plant.SetDefaultFloatingBaseBodyPose(roots[0], X_WO)
    plant.Finalize()
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    query = scene_graph.get_query_output_port().Eval(scene_graph.GetMyContextFromRoot(context))
    inspector = query.inspector()
    object_geometries = [gid for b in plant.GetBodyIndices(model)
                         for gid in plant.GetCollisionGeometriesForBody(plant.get_body(b))]
    if strategy == 4:
        # Exactly the native order: stem first, crossbar second. The three
        # small ground-witness spheres are not EE/object contact pairs for T.
        contact_geometries = [plant.GetCollisionGeometriesForBody(
            plant.GetBodyByName(name, model))[0] for name in ("horizontal_link", "vertical_link")]
        if not all(isinstance(inspector.GetShape(gid), Box) for gid in contact_geometries):
            raise ValueError("Perimeter EE preview requires the configured T collision boxes")
        height = float(params["sampling_height"])
    else:
        # Native C's point query includes all controller-object geometries,
        # including the three ground-witness spheres, but excludes EE/ground.
        contact_geometries = object_geometries
        height = float(params["z_height"])
        if not config["mesh_file"]:
            raise ValueError("Mesh-normal EE preview requires the native sampling OBJ")
        mesh = ReadObjToTriangleSurfaceMesh(Path(config["mesh_file"]), scale=1.0)
        faces, normals, areas = [], [], []
        threshold = float(params["buffer_distance"]) ** 2 - clearance ** 2 + 0.04
        for triangle in mesh.triangles():
            vertices = np.array([mesh.vertex(triangle.vertex(i)) for i in range(3)])
            cross = np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0])
            length = np.linalg.norm(cross)
            if length == 0:
                continue
            normal = cross / length
            if normal[2] ** 2 < threshold:
                faces.append(vertices)
                normals.append(normal)
                areas.append(0.5 * length)
        if not faces:
            raise ValueError("Native sampling mesh has no eligible side faces")
        faces = np.asarray(faces)
        normals = np.asarray(normals)
        cumulative_area = np.cumsum(areas)

    def distances(point):
        results = {result.id_G: result for result in query.ComputeSignedDistanceToPoint(point)}
        return [results[gid] for gid in contact_geometries]

    workspace = np.asarray(options["workspace_limits"], dtype=float)
    margin = float(options["workspace_margins"])
    radial_min, radial_max = options["robot_radius_limits"]

    def in_workspace(point):
        return (workspace[0, 3] + margin <= point[0] <= workspace[0, 4] - margin
                and workspace[1, 3] + margin <= point[1] <= workspace[1, 4] - margin
                and workspace[2, 3] <= point[2] <= workspace[2, 4]
                and radial_min + margin <= np.linalg.norm(point[:2]) <= radial_max - margin)

    rng = np.random.default_rng(seed)
    points = []
    stats = {"draws": 0, "rejected_not_overlapping": 0, "rejected_clearance": 0,
             "rejected_height": 0, "rejected_workspace": 0}
    # This is a preview-wide bound, not the native per-call failure counter.
    max_draws = min(1000000, max(10000, count * 1000))
    while len(points) < count and stats["draws"] < max_draws:
        stats["draws"] += 1
        if strategy == 4:
            local = [rng.uniform(*params["grid_x_limits"]), rng.uniform(*params["grid_y_limits"]), 0]
            candidate = rotation @ local + position
            candidate[2] = height
            closest = min(distances(candidate), key=lambda result: result.distance)
            # Sphere/box separation equals point/box distance minus EE radius.
            if closest.distance - radius > -0.001:
                stats["rejected_not_overlapping"] += 1
                continue
            witness = query.GetPoseInWorld(closest.id_G) @ closest.p_GN
            candidate = witness + (radius + clearance) * closest.grad_W
            if min(result.distance - radius for result in distances(candidate)) <= clearance - 0.001:
                stats["rejected_clearance"] += 1
                continue
            if abs(candidate[2] - height) > 0.001:
                stats["rejected_height"] += 1
                continue
        else:
            index = int(np.searchsorted(cumulative_area, rng.uniform(0, cumulative_area[-1]), side="right"))
            a, b = rng.uniform(size=2)
            if a + b > 1:
                a, b = 1 - a, 1 - b
            local = (1 - a - b) * faces[index, 0] + a * faces[index, 1] + b * faces[index, 2]
            candidate = rotation @ (local + float(params["buffer_distance"]) * normals[index]) + position
            candidate[2] = height
            if min(result.distance for result in distances(candidate)) <= clearance:
                stats["rejected_clearance"] += 1
                continue
        if params["filter_samples_for_safety"] and not in_workspace(candidate):
            stats["rejected_workspace"] += 1
            continue
        points.append(candidate.tolist())
    if len(points) != count:
        raise ValueError(f"EE preview exhausted {max_draws} geometric draws: accepted {len(points)}/{count}; "
                         "check object pose, clearance, and workspace limits")
    world = np.asarray(points)
    report = {
        "kind": "static_geometric_EE_candidates", "count": count, "seed": seed,
        "rng": "numpy.default_rng; independent preview stream, not native C++ draws",
        "strategy": strategy, "strategy_name": "random_perimeter" if strategy == 4 else "mesh_normal_multiobject",
        "points_world": points, "points_object": ((world - position) @ rotation).tolist(),
        "sampling_height_world_m": height, "configured_z_height_m": float(params["z_height"]),
        "ee_radius_m": radius, "projection_clearance_m": clearance,
        "clearance_reference": "EE surface to object" if strategy == 4 else "EE center to object",
        "normal_offset_m": radius + clearance if strategy == 4 else float(params["buffer_distance"]),
        "native_fresh_samples": {"reposition": int(params["num_additional_samples_repos"]),
                                 "c3": int(params["num_additional_samples_c3"])},
        "sources": {key: str(config[key]) for key in ("params_file", "options_file", "controller_model", "ee_model_file")},
        "workspace_filter_applied": bool(params["filter_samples_for_safety"]),
        "stats": {**stats, "accepted": count, "preview_draw_limit": max_draws},
        "runtime_omissions": ["native RNG history and predicted object state", "unsuccessful-sample history",
            "current EE, previous target and retained buffer candidates", "IK/dynamic feasibility and C3 cost ranking",
            "scene-obstacle pusher and swept-reposition-path filters", "goal-reached stop behavior"],
        "note": "Geometric preview candidates at the selected pose; not recorded or controller-selected runtime samples.",
    }
    if strategy == 4:
        report["body_frame_draw_rectangle_m"] = {"x": params["grid_x_limits"], "y": params["grid_y_limits"]}
        report["ground_witness_spheres_in_distance_check"] = False
        report["native_height_tolerance_m"] = 0.001
        report["native_clearance_tolerance_m"] = 0.001
    else:
        report["sources"]["mesh_file"] = str(config["mesh_file"])
        report["eligible_mesh_faces"] = len(faces)
        report["eligible_mesh_area_m2"] = float(cumulative_area[-1])
        report["ground_witness_spheres_in_distance_check"] = True
        report["face_selection"] = "area-weighted eligible triangles; uniform reflected barycentric coordinates"
    return report


def sample_raw_mesh_ee_candidates(settings, count, seed):
    """Preview EE sphere centres beside an actual horizontal mesh section.

    This intentionally differs from generate_samples.cc's native C proposal:
    that proposal samples side faces then overwrites world z. For a varying-
    height raw mesh, intersecting at the requested height first avoids moving
    the surface sample onto unrelated geometry. Every accepted centre is outside
    the closed original triangle surface and has the requested 3D clearance.
    No convex hull, rtree, ray accelerator, controller, or simulation is used.
    """
    from hashlib import sha256
    from pathlib import Path
    import numpy as np
    import trimesh
    import yaml
    from scipy.spatial import cKDTree
    from scipy.spatial.transform import Rotation

    if isinstance(count, bool) or int(count) != count or not 1 <= count <= 10000:
        raise ValueError("EE sample count must be an integer between 1 and 10000")
    if isinstance(seed, bool) or int(seed) != seed or seed < 0:
        raise ValueError("EE sample seed must be a nonnegative integer")
    count, seed = int(count), int(seed)
    config = settings["ee_sampling"]
    mesh_file = Path(config["mesh_file"]).resolve()
    mesh = trimesh.load(mesh_file, force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces):
        raise ValueError("Raw EE preview requires a triangle mesh")
    if not np.all(np.isfinite(mesh.vertices)) or np.any(mesh.area_faces <= 1e-18):
        raise ValueError("Raw EE preview requires finite, nondegenerate mesh triangles")
    # A closed oriented triangle chain also permits pinched edges shared by
    # four faces, provided their directed contributions cancel. Its solid
    # angle remains valid without deforming the source into a manifold mesh.
    directed_edges = mesh.edges
    unique_edges, edge_inverse, edge_counts = np.unique(
        np.sort(directed_edges, axis=1), axis=0, return_inverse=True, return_counts=True)
    edge_balance = np.bincount(edge_inverse, weights=np.where(
        directed_edges[:, 0] < directed_edges[:, 1], 1, -1), minlength=len(unique_edges))
    nonmanifold_edges = int(np.count_nonzero(edge_counts != 2))
    if np.any(edge_balance != 0) or not mesh.is_winding_consistent:
        raise ValueError("Raw EE preview requires a closed, consistently oriented triangle surface; "
                         "clean the imported geometry before sampling")
    components = mesh.split(only_watertight=False, repair=False)
    if any(component.volume <= 0 for component in components):
        raise ValueError("Raw EE preview requires outward-facing mesh components")
    scale = float(settings.get("mesh_scale", 1.0))
    position = np.asarray(settings["position_m"], dtype=float)
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("Mesh scale must be finite and positive")
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Object position must contain three finite coordinates")
    if settings.get("rpy_degrees") is not None:
        rpy = np.asarray(settings["rpy_degrees"], dtype=float)
        if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
            raise ValueError("Object RPY must contain three finite angles")
        rotation = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    else:
        quaternion = np.asarray(settings["quaternion_wxyz"], dtype=float)
        if (quaternion.shape != (4,) or not np.all(np.isfinite(quaternion))
                or np.linalg.norm(quaternion) == 0):
            raise ValueError("Object quaternion must be finite and nonzero")
        rotation = Rotation.from_quat(quaternion[[1, 2, 3, 0]]).as_matrix()
    radius = float(config["ee_radius"])
    height = float(config.get("height", -0.012))
    clearance = float(config.get("clearance", 0.027))
    offset = float(config.get("offset", 0.035))
    table_height = float(config.get("table_height", -0.029))
    if (not np.all(np.isfinite([radius, height, clearance, offset, table_height]))
            or radius <= 0 or clearance < radius or offset < clearance):
        raise ValueError("EE radius, centre clearance, and offset must be finite and "
                         "satisfy 0 < radius <= clearance <= offset")
    if height - radius < table_height:
        raise ValueError("The requested EE sample sphere intersects the tabletop")
    with Path(config["options_file"]).open() as stream:
        options = yaml.safe_load(stream)
    workspace = np.asarray(options["workspace_limits"], dtype=float)
    margin = float(options["workspace_margins"])
    radial_min, radial_max = map(float, options["robot_radius_limits"])
    if options.get("include_walls", False):
        raise ValueError("Raw mesh EE previews require include_walls=false")
    if (workspace.shape != (3, 5) or not np.all(np.isfinite(workspace))
            or not np.all(np.isfinite([margin, radial_min, radial_max]))):
        raise ValueError("Invalid EE workspace configuration")

    # All section, normal and clearance calculations use the transformed,
    # uniformly scaled original triangles in world coordinates.
    mesh.vertices = np.asarray(mesh.vertices) * scale @ rotation.T + position
    if not mesh.bounds[0, 2] < height < mesh.bounds[1, 2]:
        raise ValueError("Requested EE sampling height does not cross the object interior")
    segments, face_ids = trimesh.intersections.mesh_plane(
        mesh, plane_normal=[0, 0, 1], plane_origin=[0, 0, height], return_faces=True)
    normals = mesh.face_normals[face_ids].copy()
    normals[:, 2] = 0
    normal_lengths = np.linalg.norm(normals, axis=1)
    lengths = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1)
    usable = (normal_lengths > 1e-12) & (lengths > 1e-12)
    segments, face_ids, lengths = segments[usable], face_ids[usable], lengths[usable]
    normals = normals[usable] / normal_lengths[usable, None]
    if not len(segments):
        raise ValueError("Mesh has no usable perimeter section at the requested height")
    cumulative_length = np.cumsum(lengths)
    triangles = mesh.triangles.copy()
    centroids = triangles.mean(axis=1)
    triangle_radii = np.linalg.norm(triangles - centroids[:, None, :], axis=2).max(axis=1)
    maximum_radius = float(triangle_radii.max())
    tree = cKDTree(centroids)

    def closest_surface(point):
        # A nearest-centroid triangle supplies an upper distance bound. A
        # closer triangle's centroid must lie within upper + its radius, so
        # this radius query cannot omit the true closest triangle. The final
        # point/triangle computation is exact, including concave surfaces.
        _, first = tree.query(point)
        initial = trimesh.triangles.closest_point(triangles[[first]], point[None])[0]
        upper = float(np.linalg.norm(point - initial))
        possible = np.asarray(tree.query_ball_point(point, upper + maximum_radius + 1e-12))
        near = trimesh.triangles.closest_point(
            triangles[possible], np.broadcast_to(point, (len(possible), 3)))
        distances = np.linalg.norm(near - point, axis=1)
        index = int(np.argmin(distances))
        return float(distances[index]), near[index], int(possible[index])

    def winding_number(point):
        # Oriented solid angles of all triangles: closed exterior ~0,
        # interior ~+1. No nearest-normal sign or convex approximation.
        directions = triangles - point
        a, b, c = directions[:, 0], directions[:, 1], directions[:, 2]
        la, lb, lc = np.linalg.norm(directions, axis=2).T
        numerator = np.einsum("ij,ij->i", a, np.cross(b, c))
        denominator = (la * lb * lc + np.einsum("ij,ij->i", a, b) * lc
                       + np.einsum("ij,ij->i", b, c) * la
                       + np.einsum("ij,ij->i", c, a) * lb)
        return float(np.arctan2(numerator, denominator).sum() / (2 * np.pi))

    def in_workspace(point):
        return (workspace[0, 3] + margin <= point[0] <= workspace[0, 4] - margin
                and workspace[1, 3] + margin <= point[1] <= workspace[1, 4] - margin
                and workspace[2, 3] <= point[2] <= workspace[2, 4]
                and radial_min + margin <= np.linalg.norm(point[:2]) <= radial_max - margin)

    rng = np.random.default_rng(seed)
    points, surface_points, outward_normals, closest_points = [], [], [], []
    distances, windings, nearest_faces, sampled_faces = [], [], [], []
    stats = {"draws": 0, "rejected_workspace": 0, "rejected_clearance": 0,
             "rejected_inside_or_ambiguous": 0}
    max_draws = min(1000000, max(10000, count * 1000))
    while len(points) < count and stats["draws"] < max_draws:
        stats["draws"] += 1
        index = int(np.searchsorted(cumulative_length, rng.uniform(0, cumulative_length[-1]), side="right"))
        fraction = rng.uniform()
        surface = (1 - fraction) * segments[index, 0] + fraction * segments[index, 1]
        candidate = surface + offset * normals[index]
        # Remove only roundoff in plane intersection, never reproject a side
        # face sample from an unrelated height.
        candidate[2] = height
        if not in_workspace(candidate):
            stats["rejected_workspace"] += 1
            continue
        distance, closest, nearest_face = closest_surface(candidate)
        if distance < clearance:
            stats["rejected_clearance"] += 1
            continue
        winding = winding_number(candidate)
        if not np.isfinite(winding) or abs(winding) > 1e-5:
            stats["rejected_inside_or_ambiguous"] += 1
            continue
        points.append(candidate.tolist())
        surface_points.append(surface.tolist())
        outward_normals.append(normals[index].tolist())
        closest_points.append(closest.tolist())
        distances.append(distance)
        windings.append(winding)
        nearest_faces.append(nearest_face)
        sampled_faces.append(int(face_ids[index]))
    if len(points) != count:
        raise ValueError(f"Raw mesh EE preview exhausted {max_draws} draws: accepted {len(points)}/{count}; "
                         "check the object pose, sample height, clearance and workspace")
    world = np.asarray(points)
    return {
        "kind": "static_geometric_EE_candidates", "count": count, "seed": seed,
        "rng": "numpy.default_rng; independent static preview stream",
        "strategy": "mesh_section_perimeter", "strategy_name": "mesh_section_perimeter",
        "points_world": points, "points_object": ((world - position) @ rotation).tolist(),
        "points_mesh_unscaled": (((world - position) @ rotation) / scale).tolist(),
        "sampling_height_world_m": height, "configured_z_height_m": height,
        "ee_radius_m": radius, "projection_clearance_m": clearance,
        "clearance_reference": "EE center to actual triangle surface",
        "normal_offset_m": offset, "table_height_world_m": table_height,
        "minimum_table_sphere_clearance_m": height - radius - table_height,
        "sources": {key: str(config[key]) for key in ("mesh_file", "options_file", "ee_model_file")},
        "mesh_sha256": sha256(mesh_file.read_bytes()).hexdigest(),
        "mesh_scale": scale, "mesh_faces": len(mesh.faces), "mesh_components": len(components),
        "mesh_watertight": bool(mesh.is_watertight), "mesh_winding_consistent": True,
        "mesh_oriented_edge_closure": True, "mesh_nonmanifold_edge_count": nonmanifold_edges,
        "section_segments": len(segments), "section_length_m": float(cumulative_length[-1]),
        "section_selection": "length-weighted horizontal mesh/plane segments; uniform point along segment",
        "workspace_filter_applied": True,
        "stats": {**stats, "accepted": count, "preview_draw_limit": max_draws},
        "validation": {
            "method": "exact original-triangle distance plus full closed-mesh solid-angle winding number",
            "center_clearance_m": distances,
            "sphere_surface_clearance_m": [distance - radius for distance in distances],
            "minimum_center_clearance_m": min(distances),
            "minimum_sphere_surface_clearance_m": min(distances) - radius,
            "winding_numbers": windings, "maximum_absolute_winding": max(map(abs, windings)),
            "nearest_points_world": closest_points, "nearest_triangle_ids": nearest_faces,
            "sampled_surface_points_world": surface_points,
            "sampled_surface_triangle_ids": sampled_faces,
            "outward_horizontal_normals_world": outward_normals,
        },
        "runtime_omissions": ["native convex contact model and controller RNG",
            "robot and scene-obstacle collision checks", "swept-reposition-path and platform checks",
            "IK/dynamic feasibility, controller history and C3 cost ranking"],
        "note": "Raw-mesh geometric preview only; native collision pieces and controller selection are not evaluated here.",
    }


def preview_settings(args):
    """Resolve the physical model, world pose, and image output without rendering."""
    if args.width < 1 or args.height < 1:
        raise ValueError("--width and --height must be positive")
    if args.ee_samples is not None and not 1 <= args.ee_samples <= 10000:
        raise ValueError("--ee-samples must be between 1 and 10000")
    if args.sample_seed is not None and args.ee_samples is None:
        raise ValueError("--sample-seed requires --ee-samples")
    if args.sample_seed is not None and args.sample_seed < 0:
        raise ValueError("--sample-seed must be nonnegative")
    if args.sample_height is not None:
        if args.mesh is None or args.ee_samples is None:
            raise ValueError("--sample-height requires --mesh and --ee-samples")
        if not math.isfinite(args.sample_height):
            raise ValueError("--sample-height must be finite")
    if args.goal_yaw_degrees is not None and args.pose != "goal":
        raise ValueError("--goal-yaw-degrees requires --pose goal")
    if args.goal_yaw_degrees is not None and args.rpy_degrees is not None:
        raise ValueError("Choose either --goal-yaw-degrees or --rpy-degrees")
    if args.mesh_scale is not None and args.mesh is None:
        raise ValueError("--mesh-scale requires --mesh")
    scale = 1.0 if args.mesh_scale is None else args.mesh_scale
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("--mesh-scale must be a positive finite number")
    for value in (args.position, args.rpy_degrees):
        if value is not None and not all(math.isfinite(v) for v in value):
            raise ValueError("Position and orientation must contain finite numbers")

    demo = demo_name(args.scene, args.start, args.goal)
    configs = compose_demo_configs(demo, goal_yaw_degrees=args.goal_yaw_degrees)
    sources = load_demo_configs(demo)
    controller, simulation, goal = (configs[key] for key in ("controller", "simulation", "goal"))
    options = sources[REPO / controller["sampling_c3_options_file"]]
    if options.get("include_walls", False):
        raise ValueError("This preview supports the catalogued scenes with include_walls=false")
    scenario = sources[REPO / controller["scenario_params_file"]]
    pose = (simulation["q_init_objects"][0] if args.pose == "start"
            else [*goal["fixed_target_orientation"], *goal["fixed_target_position"]])
    model = (args.mesh or args.model or REPO / simulation["object_models"][0]).resolve()
    allowed = (".obj",) if args.mesh else (".sdf", ".urdf", ".xml")
    if model.suffix.lower() not in allowed:
        raise ValueError("Use --mesh for an OBJ, or --model for an SDF, URDF, or MJCF XML")
    if not model.is_file():
        raise FileNotFoundError(model)
    if args.geometry == "collision" and args.mesh:
        raise ValueError("Raw OBJ previews have no collision geometry; use --model")
    profiles = sources[REPO / EXPERIMENTS_FILE]["object_profiles"]
    model_names = {(REPO / profile[key]).resolve(): name
                   for name, profile in profiles.items()
                   for key in ("simulation_model", "controller_model")}
    object_name = model_names.get(model, model.stem)
    ee_sampling = None
    if args.ee_samples is not None:
        configured_models = {(REPO / simulation["object_models"][0]).resolve(),
                             (REPO / controller["object_models"][0]).resolve()}
        if args.mesh is None and model not in configured_models:
            raise ValueError("--ee-samples requires the selected scene's configured object model")
        if args.mesh is not None and args.position is None:
            raise ValueError("Raw mesh EE previews require --position X Y Z to set the object placement")
        params_file = REPO / controller["sampling_params_file"]
        params = sources[params_file]
        strategy = params["sampling_strategy"]
        if strategy not in (4, 7):
            raise ValueError("EE previews support the configured perimeter and mesh-normal samplers")
        ee_model = REPO / "examples/sampling_c3/urdf/end_effector_simple_model_xarm6.urdf"
        # Drake accepts its extension prefix without an XML namespace declaration.
        ee_xml = ee_model.read_text()
        if "xmlns:drake=" not in ee_xml:
            ee_xml = ee_xml.replace("<robot ", '<robot xmlns:drake="http://drake.mit.edu" ', 1)
        ee_radius = float(ET.fromstring(ee_xml).find(".//collision/geometry/sphere").attrib["radius"])
        base = controller["base_names"][0]
        mesh_file = REPO / "examples/sampling_c3/urdf" / base / f"{base}.obj"
        if strategy == 7 and not mesh_file.is_file():
            raise FileNotFoundError(mesh_file)
        ee_sampling = {
            "params_file": str(params_file),
            "options_file": str(REPO / controller["sampling_c3_options_file"]),
            "controller_model": str(REPO / controller["object_models"][0]),
            "mesh_file": str(mesh_file) if strategy == 7 else None,
            "ee_model_file": str(ee_model), "ee_radius": ee_radius,
            "count": args.ee_samples,
            "seed": 42 if args.sample_seed is None else args.sample_seed,
        }
        if args.mesh is not None:
            ee_sampling = {
                "mode": "mesh_section_perimeter", "mesh_file": str(model),
                "options_file": str(REPO / controller["sampling_c3_options_file"]),
                "ee_model_file": str(ee_model), "ee_radius": ee_radius,
                "height": -0.012 if args.sample_height is None else args.sample_height,
                "clearance": 0.027, "offset": 0.035,
                "count": args.ee_samples,
                "seed": 42 if args.sample_seed is None else args.sample_seed,
            }
    yaw = "" if args.goal_yaw_degrees is None else f"_yaw{args.goal_yaw_degrees:+04d}"
    samples = "" if ee_sampling is None else f'_ee{ee_sampling["count"]}_seed{ee_sampling["seed"]}'
    output = args.output or (REPO / "results/previews" /
        f"{args.scene}_{object_name}_{args.pose}_s{args.start:02d}g{args.goal:02d}{yaw}_{args.view}_{args.geometry}{samples}.png")
    if output.suffix.lower() != ".png":
        raise ValueError("--output must end in .png")
    obstacle = scenario.get("obstacle_model")
    return {
        "scene": args.scene, "demo": demo, "pose": args.pose,
        "object_name": object_name, "object_file": str(model), "raw_mesh": args.mesh is not None,
        "mesh_scale": scale,
        "position_m": args.position if args.position is not None else pose[4:7],
        "quaternion_wxyz": None if args.rpy_degrees is not None else pose[:4],
        "rpy_degrees": args.rpy_degrees,
        "robot_joints_rad": simulation["q_init_franka"],
        "obstacle_file": str(REPO / obstacle) if obstacle else None,
        "output": str(output.resolve()), "view": args.view, "geometry": args.geometry,
        "width": args.width, "height": args.height, "frames": args.frames,
        "hide_robot": args.hide_robot,
        "ee_sampling": ee_sampling,
    }


def mesh_urdf(path, scale):
    """Wrap an OBJ for display without deriving inertia from its mesh volume."""
    robot = ET.Element("robot", name="mesh_preview")
    link = ET.SubElement(robot, "link", name="object")
    # Positive dummy inertia is only for the viewer's movable body. This is
    # not a physics model, and it intentionally defines no collision geometry.
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value="1")
    ET.SubElement(inertial, "inertia", ixx="1", iyy="1", izz="1", ixy="0", ixz="0", iyz="0")
    geometry = ET.SubElement(ET.SubElement(link, "visual"), "geometry")
    # In-memory URDF parsing requires a URI. Drake resolves its path literally,
    # so retain spaces and let ElementTree escape XML-special characters.
    uri = "file://" + str(Path(path).resolve())
    ET.SubElement(geometry, "mesh", filename=uri, scale=" ".join([str(scale)] * 3))
    return ET.tostring(robot, encoding="unicode")


def prepare_render_geometry(scene_graph, source, directory, geometry, overlay_ids, hidden_ids=()):
    """Select camera-visible geometry and prepare temporary OBJ normals."""
    from pydrake.geometry import Convex, Mesh, PerceptionProperties, RenderLabel, Rgba, Role

    inspector = scene_graph.model_inspector()
    prepared = {}
    for index, gid in enumerate(inspector.GetAllGeometryIds()):
        illustration = inspector.GetIllustrationProperties(gid)
        proximity = inspector.GetProximityProperties(gid)
        visible = (illustration is not None if geometry == "visual" else proximity is not None)
        visible = (visible or gid in overlay_ids) and gid not in hidden_ids
        if inspector.GetPerceptionProperties(gid) is not None:
            scene_graph.RemoveRole(source, gid, Role.kPerception)
        if not visible:
            continue
        properties = PerceptionProperties(illustration) if illustration is not None else PerceptionProperties()
        if not properties.HasProperty("phong", "diffuse"):
            properties.AddProperty("phong", "diffuse", Rgba(0.9, 0.45, 0.15, 1))
        if not properties.HasProperty("label", "id"):
            properties.AddProperty("label", "id", RenderLabel.kDontCare)
        shape = inspector.GetShape(gid)
        if (geometry == "collision" and proximity is not None and isinstance(shape, Mesh)
                and not proximity.HasGroup("hydroelastic")):
            # Drake's point-contact solver uses a convex hull for mesh collision.
            scene_graph.ChangeShape(source, gid, Convex(shape.source(), scale3=shape.scale3()))
        elif isinstance(shape, Mesh) and shape.extension() == ".obj" and shape.source().is_path():
            path = Path(shape.source().path())
            if path not in prepared:
                has_normals, faces_have_normals = False, True
                with path.open(errors="replace") as mesh_file:
                    for line in mesh_file:
                        fields = line.split("#", 1)[0].split()
                        if fields and fields[0] == "vn":
                            has_normals = True
                        elif fields and fields[0] == "f":
                            faces_have_normals &= all(
                                len(vertex.split("/")) == 3 and vertex.split("/")[2]
                                for vertex in fields[1:])
                prepared[path] = path
                if not (has_normals and faces_have_normals):
                    import trimesh
                    destination = Path(directory) / f"mesh_{index}"
                    destination.mkdir()
                    prepared[path] = destination / path.name
                    mesh = trimesh.load(path, force="mesh", process=False)
                    mesh.export(prepared[path], include_normals=True)
            if prepared[path] != path:
                scene_graph.ChangeShape(source, gid, Mesh(str(prepared[path]), scale3=shape.scale3()))
        scene_graph.AssignRole(source, gid, properties)


def build_preview(settings, directory):
    """Build a scene and camera without advancing dynamics or opening a server."""
    # Imports stay here so --help and --dry-run work without Drake or a display.
    import numpy as np
    from pydrake.common.eigen_geometry import Quaternion
    from pydrake.geometry import (
        ClippingRange, DepthRange, DepthRenderCamera, LightParameter,
        MakeRenderEngineVtk, RenderCameraCore, RenderEngineVtkParams, Sphere,
    )
    from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.systems.sensors import CameraInfo, RgbdSensor
    from pydrake.visualization import AddFrameTriadIllustration
    if __package__:
        from .render_run_3d import EYE, TARGET, look_at
    else:
        from render_run_3d import EYE, TARGET, look_at

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    urdf = REPO / "examples/sampling_c3/urdf"

    # Geometry and welds match AddXarm6ToPlant in sampling_c3_utils.cc.
    arm, = parser.AddModels(str(urdf / "oim_xarm6_tabletop/xarm6/xarm6_policyport.xml"))
    tool, = parser.AddModels(str(urdf / "end_effector_xarm6_stick.urdf"))
    plant.WeldFrames(plant.GetFrameByName("xarm6_link6", arm),
                     plant.GetFrameByName("end_effector_flange", tool), RigidTransform())
    for filename, frame, z in (("ground_oim_xarm6.urdf", "ground", -0.029),
                                ("platform.urdf", "platform", -0.0145)):
        model, = parser.AddModels(str(urdf / filename))
        plant.WeldFrames(plant.GetFrameByName("xarm6_link_base", arm),
                         plant.GetFrameByName(frame, model), RigidTransform([0, 0, z]))
    for index, angle in enumerate(settings["robot_joints_rad"], 1):
        plant.GetJointByName(f"xarm6_joint{index}", arm).set_default_angle(angle)
    if settings["obstacle_file"]:
        # The scene's static SDF already expresses its geometry in world space.
        parser.AddModels(settings["obstacle_file"])
    if settings["raw_mesh"]:
        objects = parser.AddModelsFromString(
            mesh_urdf(settings["object_file"], settings["mesh_scale"]), "urdf")
    else:
        objects = parser.AddModels(settings["object_file"])
    if len(objects) != 1:
        raise ValueError("Object preview requires one movable model")
    model = objects[0]
    children = {plant.get_joint(j).child_body().index() for j in plant.GetJointIndices(model)}
    roots = [plant.get_body(b) for b in plant.GetBodyIndices(model) if b not in children]
    if len(roots) != 1:
        raise ValueError("Object preview requires one free base; use a rigid object model")
    if settings["rpy_degrees"] is None:
        q = np.asarray(settings["quaternion_wxyz"], dtype=float)
        transform = RigidTransform(Quaternion(q / np.linalg.norm(q)), settings["position_m"])
    else:
        transform = RigidTransform(RollPitchYaw(np.radians(settings["rpy_degrees"])), settings["position_m"])
    plant.SetDefaultFloatingBaseBodyPose(roots[0], transform)
    overlay_ids = set()
    if settings["frames"]:
        for body in (plant.world_body(), roots[0]):
            overlay_ids.update(AddFrameTriadIllustration(
                plant=plant, scene_graph=scene_graph, body=body,
                length=0.06, radius=0.002,
            ))
    for index, point in enumerate(settings.get("ee_sample_report", {}).get("points_world", [])):
        overlay_ids.add(plant.RegisterVisualGeometry(
            plant.world_body(), RigidTransform(point), Sphere(settings["ee_sampling"]["ee_radius"]),
            f"ee_sample_{index:04d}", [0.05, 0.65, 0.95, 1.0],
        ))
    plant.SetUseSampledOutputPorts(False)
    plant.Finalize()
    hidden_ids = set()
    if settings["hide_robot"]:
        for model_instance in (arm, tool):
            for body_index in plant.GetBodyIndices(model_instance):
                body = plant.get_body(body_index)
                hidden_ids.update(plant.GetVisualGeometriesForBody(body))
                hidden_ids.update(plant.GetCollisionGeometriesForBody(body))
    prepare_render_geometry(scene_graph, plant.get_source_id(), directory,
                            settings["geometry"], overlay_ids, hidden_ids)
    scene_graph.AddRenderer("preview", MakeRenderEngineVtk(RenderEngineVtkParams(
        backend="EGL", default_clear_color=[0.94, 0.96, 0.98],
        lights=[LightParameter(type="directional", frame="camera", direction=[0, 0, 1], intensity=0.85),
                LightParameter(type="directional", frame="world", direction=[0, 0, -1], intensity=0.55)],
    )))
    target = np.asarray(settings["position_m"]) + [0, 0, 0.02]
    if settings["view"] == "top":
        camera_pose = RigidTransform(RotationMatrix.MakeXRotation(math.pi), target + [0, 0, 0.8])
    elif settings["view"] == "object":
        camera_pose = look_at(target + [0.3, -0.3, 0.25], target)
    else:
        camera_pose = look_at(EYE, TARGET)
    core = RenderCameraCore("preview", CameraInfo(settings["width"], settings["height"], 0.9),
                            ClippingRange(0.01, 100), RigidTransform())
    sensor = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(), camera_pose,
                                        DepthRenderCamera(core, DepthRange(0.01, 50))))
    builder.Connect(scene_graph.get_query_output_port(), sensor.query_object_input_port())
    return builder.Build(), plant, sensor


def render_image(settings):
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    settings = dict(settings)
    sampling = settings.get("ee_sampling")
    if sampling is not None:
        settings["ee_sample_report"] = sample_ee_candidates(settings, sampling["count"], sampling["seed"])
    with tempfile.TemporaryDirectory(prefix="mesh_preview_") as directory:
        diagram, _, sensor = build_preview(settings, directory)
        context = diagram.CreateDefaultContext()
        camera_context = sensor.GetMyContextFromRoot(context)
        pixels = sensor.color_image_output_port().Eval(camera_context).data
        image = Image.fromarray(np.array(pixels, copy=True)[:, :, :3])
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default(size=16)
        title = f'{settings["object_name"]} | {settings["scene"]} | {settings["pose"]} | {settings["geometry"]}'
        position = ", ".join(f"{value:.4f}" for value in settings["position_m"])
        draw.rectangle((0, 0, image.width, 72 if sampling else 48), fill=(24, 32, 45))
        draw.text((10, 5), title, fill="white", font=font)
        draw.text((10, 26), f"Object origin in world: [{position}] m", fill="white", font=font)
        if sampling:
            draw.text((10, 49), f'Blue: {sampling["count"]} EE candidate previews | seed {sampling["seed"]}',
                      fill=(105, 215, 255), font=font)
        output = Path(settings["output"])
        output.parent.mkdir(parents=True, exist_ok=True)
        image.save(output, format="PNG")
        if sampling:
            report = dict(settings["ee_sample_report"])
            report.update(object_name=settings["object_name"], scene=settings["scene"], pose=settings["pose"],
                          object_position_world=settings["position_m"],
                          quaternion_wxyz=settings["quaternion_wxyz"], rpy_degrees=settings["rpy_degrees"])
            output.with_suffix(".ee_samples.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=SCENES, default="open_task")
    parser.add_argument("--start", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--pose", choices=("start", "goal"), default="start")
    parser.add_argument("--goal-yaw-degrees", type=int, choices=(90, 0, -90))
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--model", type=Path, help="Preview an SDF, URDF, or MJCF XML with one free base")
    source.add_argument("--mesh", type=Path, help="Preview a raw OBJ as visual geometry")
    parser.add_argument("--mesh-scale", type=float, help="Uniform OBJ scale; default 1 (metres)")
    parser.add_argument("--position", type=float, nargs=3, metavar=("X", "Y", "Z"), help="World position in metres")
    parser.add_argument("--rpy-degrees", type=float, nargs=3, metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--frames", action="store_true", help="Show world and object axes (X red, Y green, Z blue)")
    parser.add_argument("--hide-robot", action="store_true", help="Hide the arm and tool to expose the object and samples")
    parser.add_argument("--view", choices=("scene", "object", "top"), default="scene")
    parser.add_argument("--geometry", choices=("visual", "collision"), default="visual")
    parser.add_argument("--ee-samples", nargs="?", const=64, type=int, metavar="COUNT",
                        help="Overlay reproducible EE candidate previews; default 64; save coordinates as JSON")
    parser.add_argument("--sample-seed", type=int, help="EE preview random seed; default 42")
    parser.add_argument("--sample-height", type=float, metavar="Z",
                        help="Raw OBJ EE centre height in world metres; default -0.012 (17 mm above table)")
    parser.add_argument("--width", type=int, default=1280, help="PNG width in pixels")
    parser.add_argument("--height", type=int, default=960, help="PNG height in pixels")
    parser.add_argument("--output", "-o", type=Path, help="PNG path; default results/previews/<selection>.png")
    parser.add_argument("--dry-run", action="store_true", help="Print selected files, pose, and image path without rendering")
    args = parser.parse_args(argv)
    try:
        settings = preview_settings(args)
        print(json.dumps(settings, indent=2), flush=True)
        if not args.dry_run:
            output = render_image(settings)
            print(f"Saved image: {output}", flush=True)
            if settings["ee_sampling"]:
                print(f'Saved samples: {output.with_suffix(".ee_samples.json")}', flush=True)
    except KeyboardInterrupt:
        return
    except (RuntimeError, ValueError, OSError, KeyError, TypeError, ImportError, ET.ParseError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
