"""Validate and describe experiments without launching or writing files."""
import hashlib
import math
from pathlib import Path
import yaml

from c3plus.configs import (CONFIG_DIR, MESH_OBJECTS, OBSTACLE_COSTS, REPO, RUN_OBJECTS,
                           SCENES, compose_demo_configs, demo_config_digest, demo_name,
                           load_controller_goal, model_assets, resolve_object_profile)

def yaw_suffix(degrees):
    """Give each supported absolute goal orientation a distinct run identity."""
    if degrees is None:
        return ""
    if isinstance(degrees, bool) or degrees not in (-90, 0, 90):
        raise ValueError("Goal yaw must be -90, 0, or 90 degrees")
    return "_yaw_" + {-90: "m090", 0: "000", 90: "p090"}[degrees]

def plan_run(scene, obstacle_cost, start, goal, out, cap=600, port=18001,
             goal_pose=None, max_frames=1200, goal_yaw_degrees=None, object_name=None, steps=None):
    """Validate inputs and describe a run without launching or writing files."""
    if scene not in SCENES or obstacle_cost not in OBSTACLE_COSTS:
        raise ValueError("Unknown scene or obstacle cost")
    if not 1 <= start <= 5 or not 1 <= goal <= 5:
        raise ValueError("Start/goal indices must be between 1 and 5")
    if cap <= 0 or not 1024 <= port <= 65535 or max_frames <= 0:
        raise ValueError("Invalid cap, port, or frame count")
    if steps is not None and (type(steps) is not int or steps <= 0):
        raise ValueError("--steps must be a positive execution-step budget")
    if object_name is not None and object_name not in RUN_OBJECTS:
        raise ValueError(f"Unsupported run object: {object_name}; choose one of {', '.join(RUN_OBJECTS)}")
    object_options = {"object_name": object_name} if object_name is not None else {}
    profile = resolve_object_profile(scene, repo=REPO, **object_options) if object_options else None
    if profile is not None and object_name not in MESH_OBJECTS:
        # Built-in objects retain the scene's existing recorder channel.
        scene_config = yaml.safe_load((CONFIG_DIR / f"{scene}.yaml").read_text())
        profile["object_channel_substring"] = scene_config["object_channel_substring"]
    suffix = yaw_suffix(goal_yaw_degrees)
    demo = demo_name(scene, start, goal, **object_options)
    goal_file, controller_goal = load_controller_goal(demo, repo=REPO, **object_options)
    resolved = compose_demo_configs(demo, repo=REPO, goal_yaw_degrees=goal_yaw_degrees, **object_options)
    source_controller_goal = controller_goal
    if goal_yaw_degrees is not None:
        controller_goal = (*controller_goal[:2], math.radians(goal_yaw_degrees))
    x, y, _ = controller_goal
    pose = tuple(goal_pose) if goal_pose is not None else controller_goal
    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise ValueError("Goal pose must contain three finite values")
    # Archived manifests rounded their evaluation yaw values. Evaluation goals
    # must agree with the native target, including any explicit yaw override.
    angle_delta = math.atan2(math.sin(pose[2] - controller_goal[2]),
                             math.cos(pose[2] - controller_goal[2]))
    if math.hypot(pose[0] - x, pose[1] - y) > 1e-4 or abs(angle_delta) > 1e-4:
        raise ValueError("Manifest goal_pose does not match the selected demo goal; "
                         "choose a --goal index from 1 to 5 and an optional --goal-yaw-degrees.")
    object_suffix = f"_{object_name}" if object_name is not None else ""
    plan = {"scene": scene, "obstacle_cost": obstacle_cost, "start": start, "goal_index": goal,
            "run_id": f"{obstacle_cost}_{scene}{object_suffix}_s{start:02d}g{goal:02d}{suffix}_seed42",
            "demo": demo, "seed": 42, "controller_goal": controller_goal,
            "start_pose": resolved["simulation"]["q_init_object"],
            "evaluation_goal": pose, "configuration_file": str(goal_file.relative_to(REPO)),
            "configuration_digest": demo_config_digest(demo, repo=REPO, goal_yaw_degrees=goal_yaw_degrees,
                                                        **object_options),
            "out": str(Path(out).resolve()), "wall_cap_seconds": cap, "port": port,
            "max_frames": max_frames}
    if steps is not None:
        plan["execution_step_budget"] = steps
    if goal_yaw_degrees is not None:
        plan.update(goal_yaw_degrees=int(goal_yaw_degrees), source_controller_goal=source_controller_goal)
    if profile is not None:
        plan.update(object_name=object_name, object_profile=profile,
                    simulation_model=profile["simulation_model"],
                    controller_model=profile["controller_model"],
                    object_body_name=profile["object_body_name"],
                    object_channel_substring=profile["object_channel_substring"])
        plan["asset_sha256"] = {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in model_assets(resolved, REPO)}
    return plan
