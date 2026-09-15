"""Compact metrics for modern, physically execution-aligned C3+ results.

The input may contain only the selected evaluation fields. No raw recordings,
controller configuration snapshots or external OIM installation are required.
"""

import math
from statistics import fmean

from .schema import EXECUTION_SEMANTICS_VERSION


# These are experiment inputs, unlike IDs, observed counts and measured timing.
_SETTING_KEYS = (
    "seed", "obstacle_cost", "horizon", "n_admm", "samples", "temperature",
    "rho", "rho_torque", "gamma", "consensus_object_weight", "consensus",
    "local_goal", "local_goal_lookahead", "lagged_consensus", "iterations",
    "robot_substeps", "object_samples", "object_substeps", "plant",
    "goal_pos_tol", "goal_theta_tol", "goal_yaw_degrees", "start_yaw_degrees",
    "robot", "backend", "world", "config", "cap",
)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _count(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} values")
    return [_number(item, name) for item in value]


def _identity(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _threshold(hp, thresholds, hp_key, evaluation_key):
    value = hp.get(hp_key, thresholds.get(evaluation_key))
    value = _number(value, f"hyperparameters.{hp_key}")
    if value < 0:
        raise ValueError(f"{hp_key} must be nonnegative")
    if evaluation_key in thresholds:
        other = _number(thresholds[evaluation_key], f"evaluation.thresholds.{evaluation_key}")
        if other != value:
            raise ValueError(f"{hp_key} disagrees with evaluation.thresholds.{evaluation_key}")
    return value


def _settings(identity, hp, budget, object_name):
    settings = {"object": object_name}
    if hp.get("object") is not None and hp["object"] != object_name:
        raise ValueError("run.object disagrees with hyperparameters.object")
    for key in _SETTING_KEYS:
        recorded = [block[key] for block in (identity, hp) if block.get(key) is not None]
        if not recorded:
            continue
        if any(value != recorded[0] for value in recorded[1:]):
            raise ValueError(f"run.{key} disagrees with hyperparameters.{key}")
        value = recorded[0]
        if not isinstance(value, (str, int, float, bool)):
            raise ValueError(f"setting {key} must be scalar")
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            _number(value, f"setting {key}")
        settings[key] = value
    for name in ("start", "goal"):
        recorded = [block[key] for block in (identity, hp)
                    for key in (name, name + "_index") if block.get(key) is not None]
        normalized = [int(value) if isinstance(value, str) and value.isdecimal() else value
                      for value in recorded]
        if normalized:
            if (any(isinstance(value, bool) or not isinstance(value, (int, str))
                    for value in normalized)
                    or any(value != normalized[0] for value in normalized[1:])):
                raise ValueError(f"inconsistent or invalid {name} setting aliases")
            settings[name] = normalized[0]
    if budget is not None:
        settings["execution_step_budget"] = budget
    return settings


def _frequency(execution, dynamic, n, warnings):
    """Validate available timing aliases; never use a simulation period."""
    candidates = []
    if execution.get("wall_time") is not None:
        wall = _vector(execution["wall_time"], n + 1, "execution.wall_time")
        if wall[0] != 0 or any(b <= a for a, b in zip(wall, wall[1:])):
            raise ValueError("execution.wall_time must start at zero and strictly increase")
        candidates.append(("execution.wall_time differences", [b-a for a, b in zip(wall, wall[1:])]))
    for block, key, label in ((execution, "step_wall_time", "execution.step_wall_time"),
                              (dynamic, "compute_time", "dynamic.compute_time")):
        if block.get(key) is None:
            continue
        durations = _vector(block[key], n, label)
        if any(value <= 0 for value in durations):
            raise ValueError(f"{label} must be strictly positive")
        candidates.append((label, durations))
    for label, values in candidates[1:]:
        if any(not math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)
               for a, b in zip(candidates[0][1], values)):
            raise ValueError(f"{label} disagrees with execution wall timing")
    stored = execution.get("frequency_hz")
    if stored is not None and _number(stored, "execution.frequency_hz") <= 0:
        raise ValueError("execution.frequency_hz must be positive when available")
    if not candidates or not n:
        warnings.append("Execution frequency unavailable: no measured execution wall intervals.")
        return None
    frequency = 1.0 / fmean(candidates[0][1])
    if not math.isfinite(frequency) or frequency <= 0:
        raise ValueError("execution wall durations imply an invalid frequency")
    if stored is not None and not math.isclose(stored, frequency, rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError("execution.frequency_hz disagrees with measured execution wall timing")
    return frequency


def trial_metrics(run):
    """Validate one modern result and reduce it to a small, JSON-safe record.

    Goal residuals use the first simultaneously successful execution endpoint,
    or the final endpoint on failure. Explicit trajectory means and physical
    time retain their original scoring window through first success or all
    endpoints on failure. Throughput uses every recorded outer execution step.
    Unsuccessful steps use the explicit execution budget, or remain unavailable
    for unlimited runs. Present malformed telemetry raises ``ValueError``.
    """
    run = _mapping(run, "result")
    schema = _mapping(run.get("schema"), "schema")
    if (schema.get("version") != "c3plus-reference-projection-v1"
            or schema.get("semantics_version") != EXECUTION_SEMANTICS_VERSION):
        raise ValueError("requires a modern C3+ execution result schema")
    identity = _mapping(run.get("run"), "run")
    if identity.get("algorithm") != "c3plus":
        raise ValueError("run.algorithm must be c3plus")
    task = _identity(identity.get("task"), "run.task")
    object_name = _identity(identity.get("object"), "run.object")
    run_id = _identity(identity.get("run_id"), "run.run_id")
    hp = _mapping(run.get("hyperparameters"), "hyperparameters")
    execution = _mapping(run.get("execution"), "execution")
    if execution.get("alignment") != "physical_policy_boundaries_v1":
        raise ValueError("genuine physical_policy_boundaries_v1 execution alignment is required")
    n = _count(execution.get("n_steps_executed"), "execution.n_steps_executed")
    warnings = []
    budget = execution.get("step_budget")
    if budget is not None and _count(budget, "execution.step_budget") < n:
        raise ValueError("execution.step_budget is smaller than executed steps")
    if "steps" in hp and hp["steps"] != budget:
        raise ValueError("hyperparameters.steps disagrees with execution.step_budget")
    if hp.get("steps") is not None:
        _count(hp["steps"], "hyperparameters.steps")
    dynamic = _mapping(run.get("dynamic"), "dynamic")
    sim = _vector(execution.get("sim_time"), n + 1, "execution.sim_time")
    times = _vector(dynamic.get("time"), n + 1, "dynamic.time")
    if sim != times:
        raise ValueError("dynamic.time disagrees with execution.sim_time")
    if sim[0] < 0 or any(b <= a for a, b in zip(sim, sim[1:])):
        raise ValueError("execution.sim_time must be nonnegative and strictly increasing")
    poses = dynamic.get("object_pose")
    if not isinstance(poses, (list, tuple)) or len(poses) != n + 1:
        raise ValueError("dynamic.object_pose must contain N+1 execution boundary states")
    poses = [_vector(pose, 3, "dynamic.object_pose entry") for pose in poses]
    goal = _vector(_mapping(run.get("static"), "static").get("goal"), 3, "static.goal")
    evaluation = _mapping(run.get("evaluation", {}), "evaluation")
    thresholds = _mapping(evaluation.get("thresholds", {}), "evaluation.thresholds")
    pos_tol = _threshold(hp, thresholds, "goal_pos_tol", "position_m")
    yaw_tol = _threshold(hp, thresholds, "goal_theta_tol", "orientation_rad")
    pos_errors = [math.hypot(p[0]-goal[0], p[1]-goal[1]) for p in poses[1:]]
    yaw_errors = [abs((p[2]-goal[2]+math.pi) % (2*math.pi)-math.pi) for p in poses[1:]]
    if any(not math.isfinite(error) for error in pos_errors + yaw_errors):
        raise ValueError("execution goal errors must be finite")
    first = next((i for i, (pos, yaw) in enumerate(zip(pos_errors, yaw_errors), 1)
                  if pos < pos_tol and yaw < yaw_tol), None)
    scored = first if first is not None else n
    if first is None and budget is None:
        warnings.append("Failed trial has no configured execution-step budget; steps_to_goal is unavailable.")
    if not n:
        warnings.append("No executed endpoints: position and orientation errors are unavailable.")
    diagnostics = {}
    for section, key, output in (("recording", "n_snapshots", "recorder_snapshots"),
                                 ("planning", "n_updates", "planning_updates")):
        value = _mapping(run.get(section, {}), section).get(key)
        diagnostics[output] = _count(value, f"{section}.{key}") if value is not None else None
    trajectory_position = fmean(pos_errors[:scored]) if scored else None
    trajectory_orientation = fmean(yaw_errors[:scored]) if scored else None
    return {
        "task": task, "method": "c3plus", "object": object_name, "run_id": run_id,
        "settings": _settings(identity, hp, budget, object_name),
        "success": first is not None, "steps_to_goal": first if first is not None else budget,
        "execution_step_budget": budget, "execution_steps_recorded": n,
        **diagnostics, "execution_time": sim[scored] - sim[0],
        "frequency_hz": _frequency(execution, dynamic, n, warnings),
        "eps_d": pos_errors[scored - 1] if scored else None,
        "eps_o": yaw_errors[scored - 1] if scored else None,
        "trajectory_mean_position_error": trajectory_position,
        "trajectory_mean_orientation_error": trajectory_orientation,
        "theta": trajectory_orientation,  # Historical trajectory-mean alias.
        "warnings": warnings,
    }


def aggregate_metrics(trials):
    """Arithmetic per-trial means, with explicit availability denominators."""
    if not trials:
        raise ValueError("aggregate_metrics needs at least one trial")
    result = {"n": len(trials), "SR": sum(t["success"] for t in trials) / len(trials)}
    available = {"SR": len(trials)}
    warnings = []
    for key, source, successful_only in (("eps_d", "eps_d", False),
            ("eps_d_success", "eps_d", True),
            ("eps_o", "eps_o", False), ("eps_o_success", "eps_o", True),
            ("trajectory_mean_position_error", "trajectory_mean_position_error", False),
            ("trajectory_mean_position_error_success", "trajectory_mean_position_error", True),
            ("trajectory_mean_orientation_error", "trajectory_mean_orientation_error", False),
            ("trajectory_mean_orientation_error_success", "trajectory_mean_orientation_error", True),
            ("theta", "theta", False),
            ("steps", "steps_to_goal", False), ("frequency_hz", "frequency_hz", False),
            ("execution_time", "execution_time", False)):
        selected = [trial for trial in trials if not successful_only or trial["success"]]
        values = [trial.get(source) for trial in selected]
        values = [value for value in values if isinstance(value, (int, float))
                  and not isinstance(value, bool) and math.isfinite(value)]
        result[key] = fmean(values) if values else None
        available[key] = len(values)
        if len(values) < len(selected):
            warnings.append(f"{key} available for {len(values)}/{len(selected)} trials; "
                            "the mean excludes unavailable values.")
    result.update(available=available, warnings=warnings)
    return result
