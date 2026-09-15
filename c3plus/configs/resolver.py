"""Resolve native trial settings, configuration snapshots, and selected assets."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from .catalog import (MESH_OBJECTS, MODELS, _demo_selection, _read_yaml,
                      _scene_config, asset_sha256, canonical_object, canonical_task, demo_name,
                      evaluation_config, native_task, resolve_object_profile)
from .paths import CONFIG_DIR, EXPERIMENTS_FILE, REPO, _yaml_references, model_assets
from .poses import pose_ids, pose_provenance, pose_source_files, resolve_pose


def planner_environment(config) -> dict[str, str]:
    """Encode native planner overrides using the scene's evaluator polygons.

    ``box_polygons`` assigns polygon references to consecutive native slots;
    ``polygon_overrides`` maps explicit native slots to polygon references.
    Slots without an override retain their controller-configured disc geometry.
    """
    planner = config.get("planner", {})
    if not isinstance(planner, dict):
        raise ValueError("planner must be a mapping")
    unknown = planner.keys() - {"box_polygons", "polygon_overrides",
                                "object_footprint", "object_footprint_points", "obstacle_top_z"}
    if unknown:
        raise ValueError(f"unknown planner settings: {sorted(unknown, key=str)}")
    boxes = planner.get("box_polygons", [])
    overrides = planner.get("polygon_overrides", {})
    if not isinstance(boxes, list) or not isinstance(overrides, dict):
        raise ValueError("planner.box_polygons must be a list and polygon_overrides a mapping")

    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("planner geometry must contain finite numbers")
        try:
            result = float(value)
        except OverflowError as exc:
            raise ValueError("planner geometry must contain finite numbers") from exc
        if not math.isfinite(result):
            raise ValueError("planner geometry must contain finite numbers")
        return result

    def polygon(reference):
        obstacles = config.get("obstacles", {})
        polygons = obstacles.get("polygons", []) if isinstance(obstacles, dict) else []
        if (not isinstance(polygons, list) or type(reference) is not int
                or not 0 <= reference < len(polygons)):
            raise ValueError(f"invalid planner polygon reference: {reference!r}")
        vertices = polygons[reference]
        if (not isinstance(vertices, list) or len(vertices) < 3
                or any(not isinstance(p, (list, tuple)) or len(p) != 2 for p in vertices)):
            raise ValueError(f"polygon {reference} must contain at least three [x, y] vertices")
        return [tuple(number(v) for v in p) for p in vertices]

    def encode(values):
        return ",".join(format(number(v), ".15g") for v in values)

    environment = {}
    encoded_boxes = []
    for reference in boxes:
        points = polygon(reference)
        xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
        ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
        corners = {(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)}
        if (len(points) != 4 or len(corners) != 4 or set(points) != corners
                or any((a[0] == b[0]) == (a[1] == b[1])
                       for a, b in zip(points, points[1:] + points[:1]))):
            raise ValueError(f"planner box polygon {reference} must be an axis-aligned rectangle")
        encoded_boxes.append(encode(((xmin + xmax) / 2, (ymin + ymax) / 2,
                                     (xmax - xmin) / 2, (ymax - ymin) / 2)))
    if encoded_boxes:
        environment["SAMPLING_C3_OBS_BOXES"] = ";".join(encoded_boxes)
    encoded_polygons = []
    for slot, reference in overrides.items():
        if type(slot) is not int or slot < 0:
            raise ValueError(f"invalid planner obstacle slot: {slot!r}")
        if slot < len(boxes):
            raise ValueError(f"planner obstacle slot {slot} has both box and polygon overrides")
        encoded_polygons.append((slot, f"{slot}|" + "|".join(map(encode, polygon(reference)))))
    if encoded_polygons:
        environment["SAMPLING_C3_OBS_POLYS"] = ";".join(value for _, value in sorted(encoded_polygons))
    if "object_footprint" in planner:
        footprint = planner["object_footprint"]
        if footprint not in ("c_glyph", "i_glyph", "r_glyph", "a_glyph"):
            raise ValueError(f"unsupported planner object_footprint: {footprint!r}")
        environment["SAMPLING_C3_OBJECT_FOOTPRINT"] = footprint
    if "object_footprint_points" in planner:
        if "object_footprint" in planner:
            raise ValueError("planner must choose either a named or an explicit object footprint")
        points = planner["object_footprint_points"]
        if (not isinstance(points, list) or len(points) < 3
                or any(not isinstance(p, (list, tuple)) or len(p) != 2 for p in points)):
            raise ValueError("planner.object_footprint_points must contain at least three [x, y] vertices")
        points = [tuple(number(v) for v in point) for point in points]
        if (len(set(points)) < 3 or any(a == b for a, b in zip(points, points[1:] + points[:1]))
                or sum(a[0] * b[1] - b[0] * a[1]
                       for a, b in zip(points, points[1:] + points[:1])) == 0):
            raise ValueError("planner.object_footprint_points must define a nondegenerate polygon")
        environment["SAMPLING_C3_OBJECT_FOOTPRINT_POINTS"] = ";".join(
            ",".join(format(value, ".17g") for value in point) for point in points)
    if "obstacle_top_z" in planner:
        environment["SAMPLING_C3_OBS_TOP_Z"] = encode([planner["obstacle_top_z"]])
    return environment

def _yaml_dependencies(value, repo):
    pending = list(_yaml_references(value, repo))
    configs = {}
    while pending:
        path = pending.pop()
        if path in configs:
            continue
        configs[path] = _read_yaml(path)
        pending.extend(_yaml_references(configs[path], repo))
    controller = value.get("controller", {}) if isinstance(value, dict) else {}
    if controller.get("sampling_mesh_files"):
        channels = configs[repo / controller["lcm_channels_simulation_file"]]
        channels["object_state_channels"][0] = f"OBJECT_{controller['object_body_name']}_STATE_SIMULATION"
    return configs

def _vector(value, length, label, quaternion=False):
    try:
        invalid = (not isinstance(value, list) or len(value) != length
                   or any(isinstance(v, bool) or not isinstance(v, (int, float))
                          or not math.isfinite(v) for v in value))
    except OverflowError:
        invalid = True
    if invalid:
        raise ValueError(f"{label} must contain {length} finite numbers")
    if quaternion and not any(value):
        raise ValueError(f"{label} must be a nonzero quaternion")
    return deepcopy(value)

def _override_goal_yaw(goal, degrees):
    if degrees is None:
        return
    if isinstance(degrees, bool) or degrees not in (-90, 0, 90):
        raise ValueError("Goal yaw must be -90, 0, or 90 degrees")
    if goal["goal_mode"] != 2:
        raise ValueError("Goal yaw override requires fixed goal_mode=2")
    angle = math.radians(degrees) / 2
    quaternion = [math.cos(angle), 0, 0, math.sin(angle)]
    goal.update(fixed_target_orientation=quaternion, fixed_target_orientations=[deepcopy(quaternion)])

def compose_demo_configs(demo, repo=REPO, goal_yaw_degrees=None, object_name=None):
    """Compose one native trial from independent positions and orientations."""
    repo = Path(repo)
    selection = _demo_selection(demo)
    if selection is None:
        if object_name is not None:
            raise ValueError("Object selection requires an indexed experiment")
        path = repo / "examples/sampling_c3" / demo / "parameters/sampling_c3_controller_params.yaml"
        controller = _read_yaml(path)
        goal = _read_yaml(repo / controller.pop("goal_params_file"))
        simulation = _read_yaml(repo / controller.pop("sim_params_file"))
    else:
        scene_name, start, goal_index, encoded_object = selection
        if object_name is not None and canonical_object(object_name) != encoded_object:
            raise ValueError("Object selection does not match the demo identifier")
        data = _read_yaml(repo / EXPERIMENTS_FILE)
        if data["schema_version"] != 1:
            raise ValueError("Unsupported experiment configuration schema")
        scene = _scene_config(data["scenes"], native_task(scene_name))
        profile = resolve_object_profile(scene_name, object_name or encoded_object, repo)
        defaults = data["defaults"]
        start_pose = _vector(list(resolve_pose(scene_name, "start", start, repo=repo)), 3, "Start pose")
        goal_pose = _vector(list(resolve_pose(scene_name, "goal", goal_index, repo=repo)), 3, "Goal pose")
        start_xy, goal_xy = start_pose[:2], goal_pose[:2]
        start_q = [math.cos(start_pose[2] / 2), 0.0, 0.0, math.sin(start_pose[2] / 2)]
        goal_q = [math.cos(goal_pose[2] / 2), 0.0, 0.0, math.sin(goal_pose[2] / 2)]
        height = _vector([profile["object_height"]], 1, "Object height")[0]
        robot = scene.get("robot_joint_overrides", {}).get(
            start, scene.get("robot_joint_preset", profile["robot_joint_preset"]))
        joints = _vector(data["robot_joint_presets"][robot], 5, "Robot joint preset")
        controller = deepcopy(defaults["controller"])
        controller.update(sampling_c3_options_file=profile["sampling_c3_options_file"],
                          sampling_params_file=profile["sampling_params_file"],
                          scenario_params_file=scene["scenario_params_file"],
                          object_model=profile["controller_model"], object_models=[profile["controller_model"]],
                          object_body_name=profile["object_body_name"], base_name=profile["base_name"],
                          base_names=[profile["object_body_name"]])
        controller.update(deepcopy(profile.get("controller", {})))
        simulation = deepcopy(defaults["simulation"])
        initial_pose = [*start_q, *start_xy, height]
        simulation.update(object_model=profile["simulation_model"], object_models=[profile["simulation_model"]],
                          q_init_franka=joints, q_init_object=initial_pose, q_init_objects=[deepcopy(initial_pose)])
        if [start, goal_index] in scene.get("visualize_pairs", []):
            simulation["visualize_drake_sim"] = True
        goal = deepcopy(defaults["goal"])
        goal.update(deepcopy(profile.get("goal", {})))
        position = [*goal_xy, height]
        goal.update(fixed_target_position=position, fixed_target_positions=[deepcopy(position)],
                    fixed_target_orientation=goal_q, fixed_target_orientations=[deepcopy(goal_q)])
    _vector(goal["fixed_target_position"], 3, "Goal position")
    _vector(goal["fixed_target_orientation"], 4, "Goal orientation", quaternion=True)
    _vector(simulation["q_init_object"][:4], 4, "Start orientation", quaternion=True)
    _vector(simulation["q_init_object"][4:], 3, "Start position")
    _override_goal_yaw(goal, goal_yaw_degrees)
    return {"controller": controller, "simulation": simulation, "goal": goal}

def load_demo_configs(demo, repo=REPO, object_name=None):
    """Read source YAMLs for a demo, following only its selected dependencies."""
    repo = Path(repo)
    if _demo_selection(demo) is not None:
        composed = compose_demo_configs(demo, repo, object_name=object_name)
        return {repo / EXPERIMENTS_FILE: _read_yaml(repo / EXPERIMENTS_FILE),
                **_yaml_dependencies(composed, repo),
                **{path: _read_yaml(path) for path in pose_source_files(repo=repo)}}
    path = repo / "examples/sampling_c3" / demo / "parameters/sampling_c3_controller_params.yaml"
    controller = _read_yaml(path)
    return {path: controller, **_yaml_dependencies(controller, repo)}

def demo_config_digest(demo, repo=REPO, goal_yaw_degrees=None, object_name=None, *, composed=None):
    """Fingerprint effective settings and dependencies, excluding unused trials."""
    repo = Path(repo)
    if composed is None:
        composed = compose_demo_configs(demo, repo, goal_yaw_degrees, object_name)
    dependencies = {str(path.relative_to(repo)): data
                    for path, data in _yaml_dependencies(composed, repo).items()}
    payload = {"composed": composed, "dependencies": dependencies}
    if _demo_selection(demo) is not None:
        payload["poses"] = pose_provenance(repo=repo)
    if composed["controller"].get("sampling_mesh_files"):
        payload["assets"] = {str(path.relative_to(repo.resolve())): asset_sha256(path)
                             for path in model_assets(composed, repo)}
        selection = _demo_selection(demo)
        profile = resolve_object_profile(selection[0], object_name or selection[3], repo)
        payload["physics"] = profile["physics"]
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()

def write_demo_configs(demo, directory, repo=REPO, goal_yaw_degrees=None, object_name=None):
    """Save an isolated native configuration and every selected YAML dependency."""
    import yaml

    repo = Path(repo).resolve()
    directory = Path(directory).resolve()
    if directory.exists():
        raise FileExistsError(f"Refusing to overwrite configuration directory: {directory}")
    composed = compose_demo_configs(demo, repo, goal_yaw_degrees, object_name)
    dependencies = _yaml_dependencies(composed, repo)
    pose_files = set(pose_source_files(repo=repo)) if _demo_selection(demo) is not None else set()
    dependencies.update({path: _read_yaml(path) for path in pose_files})
    assets = model_assets(composed, repo) if composed["controller"].get("sampling_mesh_files") else []
    copies = {str(path.relative_to(repo)): directory / "repository" / path.relative_to(repo)
              for path in (*dependencies, *assets)}

    def localize(value):
        if isinstance(value, dict):
            return {key: localize(item) for key, item in value.items()}
        if isinstance(value, list):
            return [localize(item) for item in value]
        return str(copies[value]) if isinstance(value, str) and value in copies else value

    resolved = localize(composed)
    resolved["controller"].update(goal_params_file=str(directory / "goal.yaml"),
                                  sim_params_file=str(directory / "simulation.yaml"))
    directory.mkdir(parents=True, exist_ok=False)
    for source, data in dependencies.items():
        target = copies[str(source.relative_to(repo))]
        target.parent.mkdir(parents=True, exist_ok=True)
        if source in pose_files:
            shutil.copy2(source, target)
        else:
            target.write_text(yaml.safe_dump(localize(data), sort_keys=False))
    for source in assets:
        target = copies[str(source.relative_to(repo))]
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.suffix == ".sdf":
            ET.register_namespace("drake", "http://drake.mit.edu")
            tree = ET.fromstring(source.read_text())
            for uri in tree.iter("uri"):
                value = uri.text.strip()
                referenced = (repo / value if value.startswith("examples/") else source.parent / value).resolve()
                uri.text = str(copies[str(referenced.relative_to(repo))])
            target.write_text(ET.tostring(tree, encoding="unicode") + "\n")
        else:
            shutil.copy2(source, target)
    for role, data in resolved.items():
        (directory / f"{role}.yaml").write_text(yaml.safe_dump(data, sort_keys=False))
    if _demo_selection(demo) is not None:
        shutil.copy2(repo / EXPERIMENTS_FILE, directory / "source_experiments.yaml")
    return directory / "controller.yaml"

def load_controller_goal(demo, repo=REPO, object_name=None):
    """Return the source and planar goal for a composed or legacy demo."""
    repo = Path(repo)
    if _demo_selection(demo) is not None:
        goal_file = repo / EXPERIMENTS_FILE
        config = compose_demo_configs(demo, repo, object_name=object_name)["goal"]
    else:
        parameters = repo / "examples/sampling_c3" / demo / "parameters"
        controller = _read_yaml(parameters / "sampling_c3_controller_params.yaml")
        goal_file = repo / controller["goal_params_file"]
        config = _read_yaml(goal_file)
    x, y, _ = config["fixed_target_position"]
    w, _, _, z = config["fixed_target_orientation"]
    return goal_file, (x, y, 2.0 * math.atan2(z, w))


def environment(obstacle_cost):
    # Isolate settings from previously exported experiment knobs.
    env = {k: v for k, v in os.environ.items() if not k.startswith("SAMPLING_C3_")}
    env.update(PYTHON=sys.executable, SAMPLING_C3_SEED="42",
               SAMPLING_C3_OBSTACLE_MODE="lcs_contact")
    if obstacle_cost == "relu":
        env.update(SAMPLING_C3_RANK_OBS_MODE="relu_footprint",
                   SAMPLING_C3_OBS_RELU_EPS="0.01", SAMPLING_C3_OBS_RELU_W="200")
    return env


def check_scene_assets(scene, object_name=None, repo=REPO, *, all_poses=True):
    """Resolve selected configurations and parse assets; never advance physics."""
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant

    repo = Path(repo).resolve()
    scene = canonical_task(scene)
    profile = resolve_object_profile(scene, object_name, repo=repo)
    config = evaluation_config(scene, object_name, repo=repo)
    planner_environment(config)
    selected = compose_demo_configs(demo_name(scene, 1, 1, object_name), repo=repo, object_name=object_name)
    # Legacy primitive SDFs use Drake extensions without XML namespace
    # declarations. Drake accepts those files; the mesh snapshot helper's
    # stricter XML parser is intended for the imported mesh profiles only.
    assets = (set(model_assets(selected, repo=repo)) if profile["object_name"] in MESH_OBJECTS else
              {(repo / value).resolve() for role in ("controller", "simulation")
               for value in selected[role]["object_models"]})
    urdf = repo / "examples/sampling_c3/urdf"
    models = {urdf / "oim_xarm6_tabletop/xarm6/xarm6_policyport.xml",
              urdf / "end_effector_xarm6_stick.urdf", urdf / "ground_oim_xarm6.urdf"}
    models.update(path for path in assets if path.suffix in (".sdf", ".urdf", ".xml"))
    if MODELS[scene][1]:
        models.add(urdf / MODELS[scene][1])

    def inspect_refs(value):
        if isinstance(value, dict):
            for item in value.values():
                inspect_refs(item)
        elif isinstance(value, list):
            for item in value:
                inspect_refs(item)
        elif isinstance(value, str) and Path(value).suffix in (".yaml", ".yml", ".sdf", ".urdf", ".xml", ".obj", ".json"):
            if Path(value).is_absolute():
                raise RuntimeError(f"Source configuration must use checkout-relative asset paths: {value}")
            if not value.startswith(("examples/", "common/")):
                return
            path = (repo / value).resolve()
            if not path.is_relative_to(repo) or not path.is_file():
                raise RuntimeError(f"Missing or nonportable referenced dependency: {value}")
            if path.suffix in (".sdf", ".urdf", ".xml"):
                models.add(path)

    if profile["object_name"] in MESH_OBJECTS:
        metadata_path = repo / profile["physics_metadata_file"]
        physics = profile["physics"]
        for key in ("simulation_model", "controller_model"):
            if (metadata_path.parent / physics[key]).resolve() != (repo / profile[key]).resolve():
                raise RuntimeError(f"Physics metadata {key} disagrees with {profile['object_name']} profile")
        if physics["body_name"] != profile["object_body_name"]:
            raise RuntimeError(f"Physics metadata body name disagrees with {profile['object_name']} profile")
        pieces = physics["convex_pieces"]
        if len(pieces) != physics["collision_piece_count"]:
            raise RuntimeError(f"Physics metadata collision-piece count is inconsistent: {metadata_path}")
        expected = [(physics["source_mesh"], physics["source_sha256"])]
        expected.extend((piece["file"], piece["sha256"]) for piece in pieces)
        for relative, digest in expected:
            path = (metadata_path.parent / relative).resolve()
            if path not in assets:
                raise RuntimeError(f"Physics metadata asset is not selected by the native models: {path}")
            if asset_sha256(path) != digest:
                raise RuntimeError(f"Physics metadata hash differs from the checked-in asset: {path}")

    starts = pose_ids("start", task=scene, repo=repo)
    goals = pose_ids("goal", task=scene, repo=repo)
    if not all_poses:
        starts, goals = starts[:1], goals[:1]
    for start in starts:
        for goal in goals:
            name = demo_name(scene, start, goal, object_name)
            inspect_refs(compose_demo_configs(name, repo=repo, object_name=object_name))
            for path, saved_config in load_demo_configs(name, repo=repo, object_name=object_name).items():
                if path != repo / EXPERIMENTS_FILE:
                    inspect_refs(saved_config)
            load_controller_goal(name, repo=repo, object_name=object_name)
    plant = MultibodyPlant(0.001)
    model_parser = Parser(plant)
    model_parser.SetAutoRenaming(True)
    from .geometry import _quiet_parser
    with _quiet_parser():
        for path in sorted(models):
            if not path.is_file():
                raise RuntimeError(f"Missing model: {path}")
            model_parser.AddModels(str(path))
    return (f"{profile['object_name']}: {len(starts) * len(goals)} start/goal configurations, selected dependencies, "
            "robot/tool/object/obstacle models and available mesh hashes verified; no simulation advanced")
