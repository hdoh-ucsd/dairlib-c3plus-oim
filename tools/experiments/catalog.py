"""Scene metadata and checkout paths; scientific imports stay in leaf helpers."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[2]
TOOL_DIR = Path(__file__).resolve().parent
CONFIG_DIR = TOOL_DIR / "scene_configs"
OBSTACLE_COSTS = ("exponential", "relu")
MESH_OBJECTS = ("sugar_box", "power_drill", "hammer", "banana")
RUN_OBJECTS = ("T_block", *MESH_OBJECTS)
OBJECTS = ("T_block", "Cblock", *MESH_OBJECTS)
MODELS = {
    "open_task": ("push_t_oimscale_m01.sdf", None),
    "single_obstacle": ("push_t_oimscale_m01.sdf", "single_obstacle_box_oimframe.sdf"),
    "shelf_gap": ("push_t_oimscale_m01.sdf", "scene_shelf_gap_oimframe.sdf"),
    "ycb_clutter": ("push_t_oimscale_m01.sdf", "scene_ycb_clutter_oimframe.sdf"),
    "icra_sign": ("push_c_glyph.sdf", "scene_icra_sign.sdf"),
    "slalom": ("push_t_oimscale_m01.sdf", "scene_slalom_oimframe.sdf"),
}
SCENES = tuple(MODELS)
DEMO_FAMILIES = {
    "open_task": "matched_open_table_xarm6_",
    "single_obstacle": "matched_single_obstacle_xarm6_",
    "shelf_gap": "matched_shelf_gap_xarm6_",
    "ycb_clutter": "matched_ycb_clutter_xarm6_",
    "icra_sign": "anything_icra_c_matched_xarm6_",
    "slalom": "matched_slalom_xarm6_",
}
BINARIES = ("franka_sim", "franka_osc_controller", "franka_sampling_c3_controller")
BUILD_TARGETS = tuple(f"//examples/sampling_c3:{name}" for name in BINARIES)
EXPERIMENTS_FILE = Path("examples/sampling_c3/shared_parameters/experiments.yaml")


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
                                "object_footprint", "obstacle_top_z"}
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
    if "obstacle_top_z" in planner:
        environment["SAMPLING_C3_OBS_TOP_Z"] = encode([planner["obstacle_top_z"]])
    return environment


def demo_name(scene, start, goal, object_name=None):
    prefix = DEMO_FAMILIES[scene]
    if object_name in MESH_OBJECTS:
        if scene != "open_task":
            raise ValueError("Imported objects currently support --scene open_task only")
        prefix = f"open_table_mesh_{object_name}_xarm6_"
    elif object_name is not None and object_name not in OBJECTS:
        raise ValueError(f"Unknown object: {object_name}")
    return prefix + (f"t{start}" if start == goal else f"s{start}g{goal}")


def _read_yaml(path):
    import yaml

    return yaml.safe_load(path.read_text())


def _yaml_references(value, repo):
    if isinstance(value, dict):
        for item in value.values():
            yield from _yaml_references(item, repo)
    elif isinstance(value, list):
        for item in value:
            yield from _yaml_references(item, repo)
    elif (isinstance(value, str) and value.startswith(("examples/", "common/"))
          and value.endswith((".yaml", ".yml"))):
        yield repo / value


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


def _demo_selection(demo):
    families = [(scene, prefix, None) for scene, prefix in DEMO_FAMILIES.items()]
    families.extend(("open_task", f"open_table_mesh_{name}_xarm6_", name) for name in MESH_OBJECTS)
    for scene, prefix, object_name in families:
        if not demo.startswith(prefix):
            continue
        match = re.fullmatch(r"(?:t([1-5])|s([1-5])g([1-5]))", demo[len(prefix):])
        if not match:
            raise ValueError(f"Invalid indexed demo: {demo}")
        start = int(match[1] or match[2])
        goal = int(match[1] or match[3])
        canonical = demo_name(scene, start, goal, object_name)
        if demo != canonical:
            raise ValueError(f"Use the canonical indexed demo name: {canonical}")
        return scene, start, goal, object_name
    return None


def _merge(base, updates):
    result = deepcopy(base)
    for key, value in updates.items():
        result[key] = (_merge(result[key], value)
                       if isinstance(value, dict) and isinstance(result.get(key), dict)
                       else deepcopy(value))
    return result


def _scene_config(scenes, name, visited=()):
    if name in visited:
        raise ValueError(f"Scene inheritance cycle: {name}")
    scene = deepcopy(scenes[name])
    parent = scene.pop("extends", None)
    return _merge(_scene_config(scenes, parent, (*visited, name)), scene) if parent else scene


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


def resolve_object_profile(scene, object_name=None, repo=REPO):
    """Resolve an object and its recorded physics/evaluation metadata."""
    repo = Path(repo)
    data = _read_yaml(repo / EXPERIMENTS_FILE)
    scene_config = _scene_config(data["scenes"], scene)
    name = object_name or scene_config["object_profile"]
    if name not in OBJECTS:
        raise ValueError(f"Unknown object: {name}")
    if name in MESH_OBJECTS and scene != "open_task":
        raise ValueError("Imported objects currently support --scene open_task only")
    if name not in MESH_OBJECTS and name != scene_config["object_profile"]:
        raise ValueError(f"Scene {scene} uses {scene_config['object_profile']}; object override supports imported meshes only")
    profile = _scene_config(data["object_profiles"], name)
    profile["object_name"] = name
    if name in MESH_OBJECTS:
        document = json.loads((repo / profile["physics_metadata_file"]).read_text())
        metadata = document["objects"][name]
        profile.update(metadata["evaluation"])
        profile["physics"] = {**{key: value for key, value in document.items() if key != "objects"}, **metadata}
        profile["object_channel_substring"] = profile["object_body_name"]
    return profile


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
        if encoded_object is not None and object_name not in (None, encoded_object):
            raise ValueError("Object selection does not match the demo identifier")
        data = _read_yaml(repo / EXPERIMENTS_FILE)
        if data["schema_version"] != 1:
            raise ValueError("Unsupported experiment configuration schema")
        scene = _scene_config(data["scenes"], scene_name)
        profile = resolve_object_profile(scene_name, object_name or encoded_object, repo)
        defaults = data["defaults"]
        start_refs = _merge(profile["start_positions"], scene.get("start_positions", {}))
        start_orientation_refs = _merge(_merge(defaults["start_orientations"],
                                               profile.get("start_orientations", {})),
                                        scene.get("start_orientations", {}))
        goal_refs = _merge(profile["goal_positions"], scene.get("goal_positions", {}))
        orientation_refs = _merge(profile["goal_orientations"], scene.get("goal_orientations", {}))
        start_xy = _vector(data["start_positions"][start_refs[start]], 2, "Start position")
        goal_xy = _vector(data["goal_positions"][goal_refs[goal_index]], 2, "Goal position")
        start_q = _vector(data["orientations"][start_orientation_refs[start]],
                          4, "Start orientation", quaternion=True)
        goal_q = _vector(data["orientations"][orientation_refs[goal_index]],
                         4, "Goal orientation", quaternion=True)
        height = _vector([profile["object_height"]], 1, "Object height")[0]
        robot = scene.get("robot_joint_overrides", {}).get(start, profile["robot_joint_preset"])
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
                **_yaml_dependencies(composed, repo)}
    path = repo / "examples/sampling_c3" / demo / "parameters/sampling_c3_controller_params.yaml"
    controller = _read_yaml(path)
    return {path: controller, **_yaml_dependencies(controller, repo)}


def model_assets(composed, repo=REPO):
    """Find selected object models and their meshes for hashing and snapshots."""
    repo = Path(repo).resolve()
    pending = [repo / path for role in ("controller", "simulation")
               for path in composed[role]["object_models"]]
    pending.extend(repo / path for path in composed["controller"].get("sampling_mesh_files", []))
    paths = set()
    while pending:
        path = pending.pop().resolve()
        if path in paths:
            continue
        path.relative_to(repo)  # Only checkout assets are reproducible here.
        if not path.is_file():
            raise FileNotFoundError(path)
        paths.add(path)
        if path.suffix == ".sdf":
            for uri in ET.fromstring(path.read_text()).iter("uri"):
                value = (uri.text or "").strip()
                if "://" in value:
                    raise ValueError(f"Object model must reference local mesh assets: {value}")
                pending.append(repo / value if value.startswith("examples/") else path.parent / value)
    return sorted(paths)


def demo_config_digest(demo, repo=REPO, goal_yaw_degrees=None, object_name=None):
    """Fingerprint effective settings and dependencies, excluding unused trials."""
    repo = Path(repo)
    composed = compose_demo_configs(demo, repo, goal_yaw_degrees, object_name)
    dependencies = {str(path.relative_to(repo)): data
                    for path, data in _yaml_dependencies(composed, repo).items()}
    payload = {"composed": composed, "dependencies": dependencies}
    if composed["controller"].get("sampling_mesh_files"):
        payload["assets"] = {str(path.relative_to(repo.resolve())): hashlib.sha256(path.read_bytes()).hexdigest()
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
