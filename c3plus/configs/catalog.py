"""Scene and object catalogue queries, inheritance, and native demo identity."""
from __future__ import annotations

from copy import deepcopy
from contextlib import contextmanager
from contextvars import ContextVar
import hashlib
import json
from pathlib import Path
import re

from .paths import CONFIG_DIR, EXPERIMENTS_FILE, REPO

OBSTACLE_COSTS = ("exponential", "relu")
TASKS = ("icra_sign", "open_table", "shelf_gap", "single_obstacle", "slalom", "ycb_clutter")
OBJECTS = ("T_shape", "hammer", "sugar_box", "power_drill", "banana")
MESH_OBJECTS = OBJECTS[1:]
RUN_OBJECTS = OBJECTS
SCENES = TASKS
TASK_ALIASES = {"open_task": "open_table"}
OBJECT_ALIASES = {"T_block": "T_shape"}
_NATIVE_TASKS = {"open_table": "open_task"}
_NATIVE_OBJECTS = {"T_shape": "T_block"}
_CONFIGURATION_SNAPSHOT = ContextVar("configuration_snapshot", default=None)
MODELS = {
    "open_table": ("push_t_oimscale_m01.sdf", None),
    "single_obstacle": ("push_t_oimscale_m01.sdf", "single_obstacle_box_oimframe.sdf"),
    "shelf_gap": ("push_t_oimscale_m01.sdf", "scene_shelf_gap_oimframe.sdf"),
    "ycb_clutter": ("push_t_oimscale_m01.sdf", "scene_ycb_clutter_oimframe.sdf"),
    "icra_sign": ("push_t_oimscale_m01.sdf", "scene_icra_sign.sdf"),
    "slalom": ("push_t_oimscale_m01.sdf", "scene_slalom_oimframe.sdf"),
}
DEMO_FAMILIES = {
    "open_table": "matched_open_table_xarm6_",
    "single_obstacle": "matched_single_obstacle_xarm6_",
    "shelf_gap": "matched_shelf_gap_xarm6_",
    "ycb_clutter": "matched_ycb_clutter_xarm6_",
    "icra_sign": "matched_icra_sign_xarm6_",
    "slalom": "matched_slalom_xarm6_",
}


@contextmanager
def configuration_snapshot(repo=REPO):
    """Read each config/asset once during a bounded plan, never across invocations."""
    from .poses import pose_catalogue_snapshot

    repo = Path(repo).resolve()
    current = _CONFIGURATION_SNAPSHOT.get()
    if current is not None and current["repo"] == repo:
        yield
        return
    token = _CONFIGURATION_SNAPSHOT.set({"repo": repo, "yaml": {}, "json": {}, "hash": {}})
    try:
        with pose_catalogue_snapshot(repo=repo):
            yield
    finally:
        _CONFIGURATION_SNAPSHOT.reset(token)


def _cached_read(path, kind, reader):
    path = Path(path).resolve()
    snapshot = _CONFIGURATION_SNAPSHOT.get()
    if snapshot is None or not path.is_relative_to(snapshot["repo"]):
        return reader(path)
    cache = snapshot[kind]
    if path not in cache:
        cache[path] = reader(path)
    return deepcopy(cache[path])


def asset_sha256(path):
    """Hash an asset, sharing reads only inside configuration_snapshot()."""
    return _cached_read(path, "hash", lambda source: hashlib.sha256(source.read_bytes()).hexdigest())


def canonical_task(name):
    """Validate a public task name, accepting the explicit historical alias."""
    name = TASK_ALIASES.get(name, name)
    if name not in TASKS:
        raise ValueError(f"Unknown task: {name}; choose from {', '.join(TASKS)}")
    return name


def canonical_object(name):
    """Validate a manipulated object; legacy C assets are not campaign objects."""
    name = OBJECT_ALIASES.get(name, name)
    if name not in OBJECTS:
        raise ValueError(f"Unknown object: {name}; choose from {', '.join(OBJECTS)}")
    return name


def native_task(name):
    name = canonical_task(name)
    return _NATIVE_TASKS.get(name, name)


def native_object(name):
    name = canonical_object(name)
    return _NATIVE_OBJECTS.get(name, name)


def demo_name(scene, start, goal, object_name=None):
    scene = canonical_task(scene)
    object_name = canonical_object(object_name or "T_shape")
    start, goal = str(start), str(goal)
    prefix = DEMO_FAMILIES[scene]
    if object_name in MESH_OBJECTS:
        prefix = f"mesh_{native_task(scene)}_{object_name}_xarm6_"
    return prefix + (f"t{start}" if start == goal else f"s{start}g{goal}")

def _read_yaml(path):
    import yaml

    return _cached_read(path, "yaml", lambda source: yaml.safe_load(source.read_text()))

def _demo_selection(demo):
    if demo.startswith("anything_icra_c_matched_xarm6_"):
        raise ValueError("Historical Cblock demos are not managed campaign objects; select T_shape explicitly")
    families = [(scene, prefix, "T_shape", False) for scene, prefix in DEMO_FAMILIES.items()]
    families.extend((scene, f"mesh_{native_task(scene)}_{name}_xarm6_", name, False)
                    for scene in TASKS for name in MESH_OBJECTS)
    families.extend(("open_table", f"open_table_mesh_{name}_xarm6_", name, True)
                    for name in MESH_OBJECTS)
    for scene, prefix, object_name, legacy in families:
        if not demo.startswith(prefix):
            continue
        match = re.fullmatch(r"(?:t([1-9][0-9]*)|s([1-9][0-9]*)g([1-9][0-9]*))", demo[len(prefix):])
        if not match:
            raise ValueError(f"Invalid indexed demo: {demo}")
        start = int(match[1] or match[2])
        goal = int(match[1] or match[3])
        canonical = demo_name(scene, start, goal, object_name)
        if not legacy and demo != canonical:
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

def resolve_object_profile(scene, object_name=None, repo=REPO):
    """Resolve an object and its recorded physics/evaluation metadata."""
    repo = Path(repo)
    data = _read_yaml(repo / EXPERIMENTS_FILE)
    scene_config = _scene_config(data["scenes"], native_task(scene))
    name = canonical_object(object_name or scene_config["object_profile"])
    profile = _scene_config(data["object_profiles"], native_object(name))
    profile.update(deepcopy(profile.pop("evaluation", {})))
    profile["object_name"] = name
    profile["native_object_name"] = native_object(name)
    if name in MESH_OBJECTS:
        document = _cached_read(repo / profile["physics_metadata_file"], "json",
                                lambda source: json.loads(source.read_text()))
        metadata = document["objects"][name]
        profile.update(metadata["evaluation"])
        profile["physics"] = {**{key: value for key, value in document.items() if key != "objects"}, **metadata}
        profile["object_channel_substring"] = profile["object_body_name"]
    return profile


def evaluation_config(task, object_name=None, repo=REPO):
    """Combine task obstacles with the selected object's authored geometry."""
    repo = Path(repo)
    task = canonical_task(task)
    profile = resolve_object_profile(task, object_name, repo)
    config = _read_yaml(repo / CONFIG_DIR.relative_to(REPO) / f"{native_task(task)}.yaml")
    for key in ("footprint", "block_half_height", "tip_target_z", "tip_floor_z_real",
                "object_channel_substring", "object_name", "native_object_name",
                "object_body_name", "simulation_model", "controller_model", "physics"):
        if key in profile:
            config[key] = deepcopy(profile[key])
    config.update(task=task, native_task=native_task(task))
    planner = config.setdefault("planner", {})
    # T retains the native TFootprint point set. Mesh polygons are the existing
    # recorded evaluation outlines, in the same body frame as their models.
    planner.pop("object_footprint", None)
    if profile["object_name"] in MESH_OBJECTS:
        planner["object_footprint_points"] = deepcopy(profile["footprint"])
    return config
