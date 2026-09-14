"""Project saved experiment metadata without substituting current tuning."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import yaml


def add_recorded_semantics(result, run_dir=None):
    """Add native thresholds, object identity and source provenance from saved data.

    Trajectories, original snapshots and native configurations are never rebuilt.
    Legacy native metadata is moved out of evaluation hyperparameters, removing
    only verified duplicate mappings. Historical patches are never inferred.
    """
    directory = Path(run_dir).resolve() if run_dir is not None else None
    provenance = result.setdefault("provenance", {})
    files = (provenance.get("configuration") or {}).get("files") or {}
    sources = provenance.setdefault("metadata_sources", [])
    runtime = result.get("runtime_status")
    if runtime is None and directory is not None and (directory / "runtime_status.json").is_file():
        runtime, _ = _read_mapping(directory / "runtime_status.json")
    runtime = runtime or provenance.get("recorded_runtime") or {}
    if not isinstance(runtime, dict):
        raise ValueError("Saved runtime status must be a mapping")
    for key, expected in (("run_id", result.get("run_id")), ("scene", result.get("scenario"))):
        if expected is not None and key in runtime and runtime[key] != expected:
            raise ValueError(f"Runtime {key} does not match the result")

    def saved(name):
        entry = files.get(name)
        path = directory / name if directory is not None else None
        if path is not None and path.exists() and (
                path.is_symlink() or not path.resolve().is_relative_to(directory / "config")):
            raise ValueError(f"Invalid saved configuration path: {name}")
        raw = None
        if entry is not None:
            raw = entry["text"].encode("utf-8")
            if (hashlib.sha256(raw).hexdigest() != entry["sha256"]
                    or len(raw) != entry["size_bytes"]):
                raise ValueError(f"Embedded configuration hash mismatch: {name}")
            if path is not None and path.exists() and path.read_bytes() != raw:
                raise ValueError(f"Saved configuration changed: {name}")
        elif path is not None and path.is_file():
            raw = path.read_bytes()
        if raw is None:
            return None, None
        digest = hashlib.sha256(raw).hexdigest()
        data = json.loads(raw) if name.endswith(".json") else yaml.safe_load(raw)
        if not isinstance(data, dict):
            raise ValueError(f"Saved configuration must be a mapping: {name}")
        # YAML integer keys become strings in the preserved JSON mapping.
        data = json.loads(json.dumps(data, allow_nan=False))
        if entry is not None and "data" in entry and entry["data"] != data:
            raise ValueError(f"Embedded configuration text/data mismatch: {name}")
        expected = (runtime.get("config_sha256") or {}).get(name)
        if expected is not None and expected != digest:
            raise ValueError(f"Recorded configuration hash mismatch: {name}")
        previous = [item for item in sources if item.get("path") == name]
        if any(item.get("sha256") != digest for item in previous):
            raise ValueError(f"Metadata source hash mismatch: {name}")
        if not previous:
            sources.append({"path": name, "sha256": digest})
        return data, {"path": name, "sha256": digest, "size_bytes": len(raw)}

    hyperparameters = result.setdefault("hyperparameters", {})
    native = provenance.setdefault("c3plus", {})
    legacy_native = hyperparameters.get("c3plus") or {}
    for key, value in legacy_native.items():
        if key in native and native[key] != value:
            raise ValueError(f"Conflicting saved C3+ metadata: {key}")
        native[key] = value
    configurations = native.get("configurations") or {}
    # Keep a mapping wherever it is the only saved record. A mapping can be
    # removed only after validating its exact text/data snapshot in provenance.
    remaining = {}
    for name, value in configurations.items():
        if name in files:
            preserved, _ = saved(name)
            if json.loads(json.dumps(value, allow_nan=False)) != preserved:
                raise ValueError(f"Native configuration disagrees with the saved snapshot: {name}")
        else:
            remaining[name] = value
    if remaining:
        native["configurations"] = remaining
    else:
        native.pop("configurations", None)
    if "obstacle_cost" in native:
        if "obstacle_cost" in hyperparameters and hyperparameters["obstacle_cost"] != native["obstacle_cost"]:
            raise ValueError("Conflicting saved obstacle cost")
        hyperparameters["obstacle_cost"] = native["obstacle_cost"]
    hyperparameters.pop("c3plus", None)
    goal = configurations.get("config/goal.yaml")
    saved_goal, _ = saved("config/goal.yaml")
    if goal is not None and saved_goal is not None and goal != saved_goal:
        raise ValueError("Native goal configuration disagrees with the saved snapshot")
    goal = saved_goal if saved_goal is not None else goal
    if goal is not None and not isinstance(goal, dict):
        raise ValueError("Native goal configuration must be a mapping")

    def threshold(key):
        value = (goal or {}).get(key)
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                  or not math.isfinite(value) or value <= 0):
            raise ValueError(f"Invalid saved native threshold: {key}")
        return value

    result.setdefault("native_controller", {}).update(
        success=None,
        success_thresholds={"position_m": threshold("position_success_threshold"),
                            "orientation_rad": threshold("orientation_success_threshold")},
        thresholds_source="config/goal.yaml" if goal is not None else None,
        success_source="No native C3+ success/completion signal was recorded.")

    cfg = provenance.get("evaluation_scene_config") or {}
    profile = runtime.get("object_profile") or {}
    if not isinstance(profile, (dict, str)):
        raise ValueError("Saved object profile must be a name or mapping")
    candidates = [runtime.get("object_name"), cfg.get("object_name"),
                  profile.get("object_name") if isinstance(profile, dict) else profile]
    canonical = lambda name: "T_block" if name == "Tblock" else name
    names = [canonical(name) for name in candidates if name is not None]
    if any(not isinstance(name, str) or not name for name in names) or len(set(names)) > 1:
        raise ValueError("Conflicting or invalid saved object identities")
    object_name = names[0] if names else None
    identity_source = "saved runtime/evaluation object profile" if names else None
    if object_name is None:
        catalogue, _ = saved("config/source_experiments.yaml")
        if catalogue is not None:
            scenes = catalogue.get("scenes") or {}
            scene = result.get("scenario") or result.get("run", {}).get("task")
            visited = set()
            while scene in scenes:
                if scene in visited:
                    raise ValueError("Saved scene inheritance has a cycle")
                visited.add(scene)
                selection = scenes[scene]
                if not isinstance(selection, dict):
                    raise ValueError("Saved scene selection must be a mapping")
                if selection.get("object_profile") is not None:
                    name = selection["object_profile"]
                    if not isinstance(name, str) or name not in (catalogue.get("object_profiles") or {}):
                        raise ValueError("Saved scene selects an unknown object profile")
                    object_name = canonical(name)
                    identity_source = "config/source_experiments.yaml:scenes." + scene + ".object_profile"
                    break
                scene = selection.get("extends")
    if object_name is not None:
        result.setdefault("run", {})["object"] = object_name
        hyperparameters["object"] = object_name
        result.setdefault("static", {})["object_name"] = object_name
        result["object_name"] = object_name
        provenance["object_identity_source"] = identity_source

    state, descriptor = saved("config/source_state.json")
    expected_state = runtime.get("source_state")
    if state is not None:
        if not isinstance(expected_state, dict) or any(
                expected_state.get(key) != value for key, value in descriptor.items()):
            raise ValueError("Source state snapshot disagrees with runtime provenance")
        if state.get("format") != "git-source-state/v1" or any(
                state.get(key) != expected_state.get(key) for key in ("base_commit", "scope")):
            raise ValueError("Invalid source state envelope metadata")
        if runtime.get("commit") is not None and state.get("base_commit") != runtime["commit"]:
            raise ValueError("Source state base commit disagrees with the recorded commit")
        provenance["source_state"] = {**expected_state, "status": "recorded",
            "embedded_json_pointer": ("/provenance/configuration/files/config~1source_state.json/data"
                                      if "config/source_state.json" in files else None)}
    elif expected_state is not None:
        raise ValueError("Recorded source state snapshot is missing")
    else:
        provenance["source_state"] = {"status": "unavailable",
            "worktree_dirty": runtime.get("worktree_dirty"),
            "reason": "No source state snapshot was recorded; the historical patch cannot be recovered from the current checkout."}
    return result


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
    mean observed state interval, or None when it cannot be established. No files
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
                           control_dt=control_dt,
                           control_dt_source="mean_observed_state_interval",
                           goal_pos_tol=summary.get("pos_tol"),
                           goal_theta_tol=summary.get("ang_tol"),
                           obstacle_cost=runtime.get("obstacle_cost"))
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
    provenance = {"c3plus": {"obstacle_cost": runtime.get("obstacle_cost"),
                              "wall_cap_seconds": runtime.get("wall_cap_seconds"),
                              "configurations": configurations,
                              "sampler_environment": runtime.get("sampler_settings")},
                  "metadata_sources": [{"path": path, "sha256": digest}
                                        for path, digest in sorted(sources.items())],
                  "missing_optional_metadata": sorted(set(missing)),
                  "recorded_runtime": recorded_runtime,
                  "metadata_semantics": {
                      "steps": "Observed intervals between retained states; not the requested run budget.",
                      "control_dt": "Mean observed state interval: (dynamic.time[-1]-dynamic.time[0])/steps_run; null unless all observed intervals are positive. Not physics or planner dt.",
                      "horizon": "Saved C3 prediction horizon N, not an MPPI horizon configuration.",
                      "n_admm": "Saved inner C3 ADMM iterations (admm_iter); not object/robot consensus iterations.",
                      "costs": "Diagnostic weights supplied by the exporter, not the full native optimization objective.",
                      "null": "Unrecorded or without an equivalent in this workflow; no current defaults substituted."}}
    return deepcopy({"run": run, "hyperparameters": hyperparameters, "static": static,
                     "provenance": provenance})
