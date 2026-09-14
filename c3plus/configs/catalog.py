"""Scene and object catalogue queries, inheritance, and native demo identity."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import re

from .paths import EXPERIMENTS_FILE, REPO

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
