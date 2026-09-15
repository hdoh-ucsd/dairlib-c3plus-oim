"""Configured and raw-mesh EE proposal previews without dynamics."""
from pathlib import Path


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
    if (settings["raw_mesh"] or
            settings.get("ee_sampling", {}).get("mode") == "mesh_section_perimeter"):
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
