"""Scene metadata and checkout paths; scientific imports stay in leaf helpers."""
from __future__ import annotations

import math
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
TOOL_DIR = Path(__file__).resolve().parent
CONFIG_DIR = TOOL_DIR / "scene_configs"
OBSTACLE_COSTS = ("exponential", "relu")
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


def demo_name(scene, start, goal):
    return DEMO_FAMILIES[scene] + (f"t{start}" if start == goal else f"s{start}g{goal}")


def load_controller_goal(demo, repo=REPO):
    """Return the goal file and planar pose referenced by a demo's controller."""
    import yaml

    parameters = repo / "examples/sampling_c3" / demo / "parameters"
    controller = yaml.safe_load((parameters / "sampling_c3_controller_params.yaml").read_text())
    goal_file = repo / controller["goal_params_file"]
    config = yaml.safe_load(goal_file.read_text())
    x, y, _ = config["fixed_target_position"]
    w, _, _, z = config["fixed_target_orientation"]
    return goal_file, (x, y, 2.0 * math.atan2(z, w))
