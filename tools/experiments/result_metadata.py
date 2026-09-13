"""Project saved experiment metadata without substituting current tuning."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import yaml


def _read_mapping(path):
    raw = path.read_bytes()
    if path.suffix == ".json":
        data = json.loads(raw)
    else:
        data = yaml.safe_load(raw)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a metadata mapping in {path}")
    try:
        json.dumps(data, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Metadata must contain finite JSON-compatible values: {path}") from exc
    return data, hashlib.sha256(raw).hexdigest()


def _snapshot_path(reference, run_dir):
    """Resolve recorded container paths exclusively inside this run's snapshot."""
    if not isinstance(reference, str) or not reference:
        return None
    source = Path(reference)
    candidates = [source] if source.is_absolute() else [run_dir / source, run_dir / "config" / source]
    parts = source.parts
    for index, part in enumerate(parts):
        if part == "config":
            candidates.append(run_dir.joinpath(*parts[index:]))
    if reference.startswith(("examples/", "common/")):
        candidates.append(run_dir / "config/repository" / reference)
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_relative_to(run_dir / "config") and resolved.exists():
            return resolved
    return None


def build_metadata(run_dir, scene, run_id, cfg, summary, n_intervals, control_dt):
    """Return reference-shaped metadata from saved files and observed intervals.

    ``summary`` supplies evaluator ``pos_tol`` and ``ang_tol``; they must not be
    replaced with the native goal-generator tolerances. ``control_dt`` is the
    observed constant sampling interval, or None when intervals vary. No files
    are written, and missing native settings remain unknown.
    """
    run_dir = Path(run_dir).resolve()
    sources, missing = {}, []

    def read(path):
        if path is None:
            return {}
        path = Path(path)
        if not path.exists():
            missing.append(str(path.relative_to(run_dir)))
            return {}
        data, digest = _read_mapping(path)
        sources[str(path.relative_to(run_dir))] = digest
        return data

    runtime = read(run_dir / "runtime_status.json")
    for key, expected in (("run_id", run_id), ("scene", scene)):
        if key in runtime and runtime[key] != expected:
            raise ValueError(f"runtime_status.{key} does not match the requested run")
    configurations = {}

    def snapshot(reference):
        path = _snapshot_path(reference, run_dir)
        if path is None:
            if reference:
                missing.append(reference)
            return {}
        relative = str(path.relative_to(run_dir))
        if relative not in configurations:
            configurations[relative] = read(path)
        return configurations[relative]

    controller = snapshot("config/controller.yaml")
    if not controller and runtime.get("controller_params_file"):
        controller = snapshot(runtime["controller_params_file"])
    simulation = snapshot("config/simulation.yaml")
    if not simulation and controller.get("sim_params_file"):
        simulation = snapshot(controller["sim_params_file"])
    goal = snapshot("config/goal.yaml")
    if not goal and controller.get("goal_params_file"):
        goal = snapshot(controller["goal_params_file"])
    options = snapshot(controller.get("sampling_c3_options_file"))
    sampling = snapshot(controller.get("sampling_params_file"))
    # Keep the remaining recorded YAML dependencies too: OSC, repositioning,
    # solver tolerances and channels affect reproducibility outside the planner.
    pending = list(configurations.values())
    visited = set()
    while pending:
        value = pending.pop()
        if isinstance(value, dict):
            pending.extend(value.values())
        elif isinstance(value, list):
            pending.extend(value)
        elif isinstance(value, str) and value.endswith((".yaml", ".yml")) and value not in visited:
            visited.add(value)
            pending.append(snapshot(value))

    profile = runtime.get("object_profile") or {}
    if not isinstance(profile, dict):
        raise ValueError("runtime_status.object_profile must be a mapping")
    object_name = runtime.get("object_name") or cfg.get("object_name") or profile.get("object_name")
    obstacles = cfg.get("obstacles") or {}
    if not isinstance(obstacles, dict):
        raise ValueError("Evaluation obstacles must be a mapping")
    projected_obstacles = [{"type": "polygon", "vertices": polygon}
                           for polygon in (obstacles.get("polygons") or [])]
    for disc in obstacles.get("discs") or []:
        x, y, radius = disc
        projected_obstacles.append({"type": "circle", "center": [x, y], "radius": radius})

    hyperparameters = dict.fromkeys((
        "config", "steps", "samples", "horizon", "object", "temperature",
        "robot_substeps", "object_samples", "plant", "object_substeps", "n_admm",
        "rho", "rho_torque", "gamma", "consensus_object_weight", "consensus",
        "consensus_source", "local_goal", "local_goal_lookahead", "lagged_consensus",
        "iterations", "control_dt", "goal_pos_tol", "goal_theta_tol"))
    hyperparameters.update(config="xarm6", steps=n_intervals, horizon=options.get("N"),
                           object=object_name, n_admm=options.get("admm_iter"),
                           control_dt=control_dt, goal_pos_tol=summary.get("pos_tol"),
                           goal_theta_tol=summary.get("ang_tol"), costs={},
                           c3plus={"obstacle_cost": runtime.get("obstacle_cost"),
                                   "wall_cap_seconds": runtime.get("wall_cap_seconds"),
                                   "configurations": configurations,
                                   "sampler_environment": runtime.get("sampler_settings")})
    recorded_runtime = {key: runtime[key] for key in (
        "runtime", "commit", "worktree_dirty", "configuration_digest", "config_sha256",
        "binary_sha256", "asset_sha256", "execution", "simulation_wall_seconds",
        "seed_verified", "goal_yaw_verified", "wrapper_rc", "postprocess_rc", "render_rc",
        "cost_fig_rc", "failures") if key in runtime}
    run = {"world": "3d", "task": scene, "robot": "xarm6", "algorithm": "c3plus",
           "robot_opt": None, "object_opt": None, "seed": runtime.get("seed"),
           "start_index": str(runtime["start"]) if runtime.get("start") is not None else None,
           "goal_index": str(runtime["goal_index"]) if runtime.get("goal_index") is not None else None,
           "backend": "drake", "interactive": None, "run_id": run_id, "object": object_name}
    static = {"goal": cfg.get("goal"), "object_footprint_body": cfg.get("footprint"),
              "object_limit_surface_d": None, "object_wrench_limit": None,
              "obstacles": projected_obstacles, "robot": "xarm6", "sim_timestep": simulation.get("dt"),
              "pusher_radius": cfg.get("pusher_radius"), "object_name": object_name,
              "simulation_model": runtime.get("simulation_model") or cfg.get("simulation_model")
                                  or simulation.get("object_model"),
              "controller_model": runtime.get("controller_model") or cfg.get("controller_model")
                                  or controller.get("object_model"),
              "object_body_name": runtime.get("object_body_name") or cfg.get("object_body_name")
                                  or controller.get("object_body_name"),
              "object_physics": profile.get("physics"),
              "footprint_method": cfg.get("footprint_method", profile.get("footprint_method"))}
    static.update({key: cfg[key] for key in ("table", "block_half_height", "tip_target_z",
                                            "tip_floor_branch", "tip_floor_z_real") if key in cfg})
    provenance = {"metadata_sources": [{"path": path, "sha256": digest}
                                        for path, digest in sorted(sources.items())],
                  "missing_optional_metadata": sorted(set(missing)),
                  "recorded_runtime": recorded_runtime,
                  "metadata_semantics": {
                      "steps": "Observed intervals between retained states; not the requested run budget.",
                      "control_dt": "Observed constant state interval, or null for variable intervals.",
                      "horizon": "Saved C3 prediction horizon N, not an MPPI horizon configuration.",
                      "n_admm": "Saved inner C3 ADMM iterations (admm_iter); not object/robot consensus iterations.",
                      "costs": "Diagnostic weights supplied by the exporter, not the full native optimization objective.",
                      "null": "Unrecorded or without an equivalent in this workflow; no current defaults substituted."}}
    return deepcopy({"run": run, "hyperparameters": hyperparameters, "static": static,
                     "provenance": provenance})
