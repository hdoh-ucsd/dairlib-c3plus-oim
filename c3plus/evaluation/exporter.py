"""Project saved native execution and asynchronous snapshots without resampling."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

from c3plus.recording.execution import read_execution_steps, read_native_execution
from c3plus.recording.planning import read_planning_updates
from c3plus.utils.serialization import _json_values
from .metadata import add_recorded_semantics, build_metadata
from .schema import (BLOCKS, SNAPSHOT_STATE_ARRAYS, SNAPSHOT_INTERVAL_ARRAYS,
                     SNAPSHOT_SEMANTICS_VERSION, EXECUTION_SEMANTICS_VERSION,
                     projection_schema)

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(q):  # [w,x,y,z]
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _number(value):
    return None if value is None or value == "" else float(value)


def _differences(values, times, angular=()):
    """Backward estimates at recorded times; never invent an initial velocity."""
    width = len(values[0])
    result = [[None] * width]
    for previous, current, t0, t1 in zip(values, values[1:], times, times[1:]):
        dt = t1 - t0
        row = []
        for j, (a, b) in enumerate(zip(previous, current)):
            if dt <= 0 or a is None or b is None or not math.isfinite(a + b):
                row.append(None)
            else:
                row.append((wrap(b - a) if j in angular else b - a) / dt)
        result.append(row)
    return result


def add_execution_timing(result, events):
    """Add only execution timing; its event axis is independent of snapshots."""
    if events is None:
        return result
    if not isinstance(events, list):
        raise ValueError("Execution events must be a list")
    times = []
    for index, event in enumerate(events):
        if (not isinstance(event, dict) or type(event.get("execution_step")) is not int
                or event["execution_step"] != index):
            raise ValueError("Execution steps must be consecutive source counters starting at zero")
        value = event.get("execution_wall_time")
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or (index == 0 and value != 0)
                or (times and value <= times[-1])):
            raise ValueError("Execution wall timestamps must be finite, start at zero, and strictly increase")
        times.append(value)
    n_intervals = max(0, len(times) - 1)
    elapsed = times[-1] - times[0] if times else 0.0
    mean = elapsed / n_intervals if n_intervals else None
    frequency = 1.0 / mean if mean is not None and mean > 0 else None
    if frequency is not None and not math.isfinite(frequency):
        frequency = None
    result["dynamic"].update(execution_step=[event["execution_step"] for event in events],
                             execution_wall_time=times)
    result["execution_timing"] = {
        "n_steps": len(times), "n_intervals": n_intervals,
        "elapsed_wall_time_s": elapsed, "mean_step_wall_time_s": mean,
        "frequency_hz": frequency, "source": "monotonic_wall_time_at_execution_step"}
    return result


def add_execution_projection(result, cfg, native, updates):
    """Project exact simulator policy boundaries, preserving every raw snapshot.

    A policy ends at the next adoption or at the directly observed terminal
    boundary. The terminal policy is censored by the existing run shutdown.
    Missing or inconsistent telemetry fails closed, without synthetic states.
    """
    if native is None:
        return result  # Historical files retain their original projection.
    headers = native["headers"]
    if not headers or any(h.get("alignment") != "physical_policy_boundaries_v1" for h in headers):
        raise ValueError("Native execution alignment unavailable: " + str(headers))
    if len(headers) != 1 or len(native["terminals"]) != 1:
        raise ValueError("Execution requires one native header and one exact terminal boundary")
    budget = headers[0].get("step_budget")
    if budget is not None and (type(budget) is not int or budget <= 0):
        raise ValueError("Execution budget must be a configured positive integer or null")
    events = native["boundaries"]
    n = len(events)
    if not n:
        raise ValueError("No applied policy: an execution-aligned initial state is unavailable")
    terminal = native["terminals"][0]
    if terminal.get("reason") not in {"shutdown", "step_budget"}:
        raise ValueError("Unknown execution terminal boundary")
    if budget is not None and (n > budget or terminal["reason"] == "step_budget" and n != budget):
        raise ValueError("Native execution count disagrees with the configured budget")
    if not isinstance(updates, list):
        raise ValueError("Planning identity records required to verify applied policies")
    plans = {}
    for i, update in enumerate(updates):
        if (type(update.get("update")) is not int or update["update"] != i
                or type(update.get("utime")) is not int or update["utime"] <= 0
                or update["utime"] in plans or update.get("mode") not in {"c3", "reposition"}):
            raise ValueError("Invalid or ambiguous planning update identity")
        plans[update["utime"]] = update
    ids = [event.get("plan_utime") for event in events]
    if (any(type(value) is not int or value not in plans for value in ids) or len(set(ids)) != n
            or any(b <= a for a, b in zip(ids, ids[1:]))):
        raise ValueError("An applied policy must resolve to exactly one recorded planning update")
    if terminal.get("plan_utime") != ids[-1]:
        raise ValueError("Terminal state must close the last actually applied policy")
    channels = headers[0].get("object_channels", [])
    matches = [i for i, channel in enumerate(channels) if cfg["object_channel_substring"] in channel]
    if len(matches) != 1:
        raise ValueError("Ambiguous manipulated object in native execution telemetry")
    object_index = matches[0]
    boundaries = [*events, terminal]
    wall, sim, poses, full_poses, velocities, robot_q, robot_v = [], [], [], [], [], [], []
    def finite_vector(value, size=None):
        return (isinstance(value, list) and (size is None or len(value) == size)
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                        for v in value))
    for i, boundary in enumerate(boundaries):
        if type(boundary.get("boundary_step")) is not int or boundary["boundary_step"] != i:
            raise ValueError("Native execution boundaries must be consecutive starting at zero")
        w, t = boundary.get("wall_time"), boundary.get("sim_time")
        if (not finite_vector([w, t]) or i == 0 and w != 0
                or wall and (w <= wall[-1] or t <= sim[-1])):
            raise ValueError("Execution wall and simulation boundary times must strictly increase")
        objects = boundary.get("objects")
        if not isinstance(objects, list) or len(objects) != len(channels):
            raise ValueError("Native execution object dimensions disagree with configuration")
        obj = objects[object_index]
        q, v = obj.get("q"), obj.get("v")
        rq, rv = boundary.get("robot_q"), boundary.get("robot_v")
        if (not finite_vector(q, 7) or not finite_vector(v, 6)
                or not finite_vector(rq) or not rq or not finite_vector(rv, len(rq))
                or robot_q and len(rq) != len(robot_q[0])
                or not math.isclose(sum(x*x for x in q[:4]), 1.0, abs_tol=1e-5)):
            raise ValueError("Invalid exact native execution state")
        wall.append(w); sim.append(t)
        poses.append([q[4], q[5], quat_yaw(q[:4])])
        full_poses.append(q); velocities.append(v)
        robot_q.append(rq); robot_v.append(rv)
    intervals = [b-a for a, b in zip(wall, wall[1:])]
    if not finite_vector(intervals) or not all(dt > 0 for dt in intervals):
        raise ValueError("Execution wall intervals must be finite and positive")
    mean = sum(intervals) / n
    frequency = 1.0 / mean
    if not math.isfinite(frequency) or frequency <= 0:
        raise ValueError("Execution frequency must be finite and positive")
    snapshots = deepcopy(result["dynamic"])
    result.setdefault("recording", {}).update(
        semantics="Asynchronous retained C3_DEBUG_CURR/observation snapshots; not executed actions.",
        n_snapshots=len(snapshots["time"]), n_recorded_intervals=len(snapshots["time"])-1,
        snapshot_dynamic=snapshots, execution_native=deepcopy(native), planning_updates=deepcopy(updates))
    result["planning"] = {
        "semantics": "Completed ComputePlan updates and selected-policy dispatches; skipped policies remain planning-only.",
        "n_updates": len(updates), "admm_iterations_per_solve": result["hyperparameters"].get("n_admm"),
        "admm_semantics": "Inner ADMM iterations per candidate solve; multiple candidates and optional second solves per update.",
        "update_sim_time": [update["utime"] * 1e-6 for update in updates],
        "update_sim_time_source": "Robot feedback timestamp consumed by each planning update; not execution time.",
        "solve_time_s": None}
    result["execution"] = {
        "alignment": "physical_policy_boundaries_v1",
        "semantics": "Selected outer policies actually applied by the simulator, including C3 and reposition.",
        "n_steps_executed": n, "step_budget": budget,
        "sim_time": sim, "wall_time": wall, "step_wall_time": intervals,
        "frequency_hz": frequency, "plan_utime": ids,
        "mode": [plans[value]["mode"] for value in ids],
        "terminal_reason": terminal["reason"],
        "terminal_semantics": "Direct native terminal state; final held policy ends at the run boundary, not its planned horizon.",
        "source": "monotonic_wall_time_at_physical_policy_boundary"}
    result["dynamic"] = {
        "time": sim, "object_pose": poses, "object_pose_3d": full_poses,
        "object_spatial_velocity": velocities,
        "robot_joint_positions": robot_q, "robot_joint_velocities": robot_v,
        "compute_time": intervals}
    result.pop("execution_timing", None)
    result["hyperparameters"].update(steps=budget, control_dt=None,
                                     control_dt_source="variable_physical_policy_duration")
    schema = result["schema"]
    schema.update(semantics_version=EXECUTION_SEMANTICS_VERSION,
        indexing="For N applied outer policies, dynamic states and execution boundary times have N+1 entries. "
                 "State 0 is recorded immediately before the first applied policy; state i+1 closes policy i. "
                 "Raw asynchronous snapshots remain in recording.snapshot_dynamic.",
        sampling="Exact native plant states at physical policy boundaries; no snapshot interpolation.",
        compute_time="For C3+, dynamic.compute_time is the measured wall-clock duration of each genuine outer "
                     "execution/control step and is used for execution frequency. It is not optimizer solve time. "
                     "Internal planner solve timing, when available, lives under planning.",
        state_arrays=[key for key in result["dynamic"] if key != "compute_time"],
        interval_arrays=["compute_time"],
        velocities="object_spatial_velocity is native world angular [wx,wy,wz] then translational [vx,vy,vz]; "
                   "robot_joint_velocities are native plant joint velocities.",
        qpos="The projected static.state_layout describes recording.snapshot_dynamic.qpos/qvel only.",
        evaluation="Table-II scoring uses the first simultaneous strict goal crossing at an execution endpoint. "
                   "The legacy evaluation section and its costs remain snapshot diagnostics, including settling.")
    schema["missing"].pop("compute_time", None)
    schema["legacy_fields"].update(
        n_control_steps="Deprecated snapshot count; use recording.n_snapshots. Never executed controls.",
        steps_run="Deprecated recorded interval count; use recording.n_recorded_intervals.",
        **{"hyperparameters.steps": "For semantics_version>=4: configured execution-step budget; null means unlimited.",
           "dynamic.evaluation_costs": "Moved to recording.snapshot_dynamic.evaluation_costs; snapshot diagnostics.",
           "dynamic.evaluation_total": "Moved to recording.snapshot_dynamic.evaluation_total; snapshot diagnostics."})
    result["provenance"].setdefault("metadata_semantics", {}).update(
        steps="Configured execution-step budget from native runtime telemetry; null means unlimited.",
        control_dt="Variable physical execution duration: scalar unavailable; use execution.sim_time boundaries.",
        compute_time=schema["compute_time"])
    return result


def project_result(run_dir, scene, run_id, cfg, summary, steps, rows,
                   pos_tol=0.05, ang_tol=0.1, evaluation_costs=None, include_semantics=True,
                   execution_steps=None, execution_native=None, planning_updates=None):
    """Project recorded snapshots into the reference's state/interval layout.

    M observed states define M-1 observed intervals. There is no synthetic t=0
    state and no claim that a recorded effort caused the next sampled state.
    Legacy summary fields retain their meanings; steps_run counts intervals.
    """
    if not rows:
        raise ValueError("Cannot export an empty trajectory")
    if summary.get("run_id") != run_id or summary.get("scenario") != scene:
        raise ValueError("Result run_id/scenario does not match the requested run")
    raw = {}
    for step in steps:
        channels = [ch for ch in step["objects"] if cfg["object_channel_substring"] in ch]
        if not channels:
            continue
        if len(channels) != 1:
            raise ValueError("Ambiguous manipulated object channel")
        k = int(step["control_step"])
        if k in raw:
            raise ValueError("Duplicate raw control_step")
        raw[k] = (step, step["objects"][channels[0]])
    indices = [int(row["control_step"]) for row in rows]
    if len(raw) != len(rows) or set(indices) != set(raw) or any(
            b <= a for a, b in zip(indices, indices[1:])):
        raise ValueError("CSV and raw control steps do not match in increasing order")
    if summary.get("n_control_steps") != len(rows):
        raise ValueError("Summary n_control_steps does not match recorded rows")
    times, poses, poses_3d, robot_q, robot_v, efforts, tips, tilts = [], [], [], [], [], [], [], []
    nq = len(raw[indices[0]][0]["robot_q"])
    for k, row in zip(indices, rows):
        step, pose = raw[k]
        if row.get("run_id", run_id) != run_id or row.get("scenario", scene) != scene:
            raise ValueError("CSV run_id/scenario does not match the requested run")
        t = float(row["sim_time"])
        if not math.isfinite(t) or not math.isclose(t, float(step["sim_time"]), abs_tol=1e-9, rel_tol=0):
            raise ValueError("CSV and raw sim_time do not match")
        planar = [float(row[key]) for key in ("object_x", "object_y", "object_yaw")]
        if len(pose) < 7 or len(step["robot_q"]) != nq:
            raise ValueError("Inconsistent raw pose or robot joint dimensions")
        if not all(math.isfinite(v) for v in [*planar, *pose[:7], *cfg["goal"]]):
            raise ValueError("Nonfinite object pose or goal")
        if (not math.isclose(planar[0], pose[4], abs_tol=1e-9, rel_tol=0)
                or not math.isclose(planar[1], pose[5], abs_tol=1e-9, rel_tol=0)
                or abs(wrap(planar[2] - quat_yaw(pose[:4]))) > 1e-9):
            raise ValueError("CSV and raw object poses do not match")
        for j, key in enumerate(("goal_x", "goal_y", "goal_yaw")):
            if key in row:
                delta = float(row[key]) - cfg["goal"][j]
                if not math.isfinite(delta) or abs(wrap(delta) if j == 2 else delta) > 1e-9:
                    raise ValueError("CSV goal does not match the scene configuration")
        times.append(t)
        poses.append(planar)
        poses_3d.append(list(pose[:7]))
        robot_q.append(list(step["robot_q"]))
        for key, target in (("robot_v", robot_v), ("robot_u", efforts)):
            vector = step.get(key) or [None] * nq
            if len(vector) != nq:
                raise ValueError(f"Inconsistent {key} dimensions")
            target.append(list(vector))
        tips.append([_number(row.get(key)) for key in ("tip_x", "tip_y", "tip_z")])
        roll, pitch = (_number(row.get(key)) for key in ("tip_roll", "tip_pitch"))
        tilts.append(None if roll is None or pitch is None or not math.isfinite(roll + pitch)
                     else math.acos(max(-1.0, min(1.0, -math.cos(roll) * math.cos(pitch)))))

    n = len(rows) - 1
    dt = [b - a for a, b in zip(times, times[1:])]
    observed_span = times[-1] - times[0]
    observed_dt = observed_span / n if n > 0 and math.isfinite(observed_span) and all(d > 0 for d in dt) else None
    object_v = _differences(poses, times, angular=(2,))
    robot_xy = [p[:2] for p in tips]
    robot_xy_v = _differences(robot_xy, times)
    object_z_v = _differences([[p[6]] for p in poses_3d], times)
    metadata = build_metadata(run_dir, scene, run_id, cfg,
                              {**summary, "pos_tol": pos_tol, "ang_tol": ang_tol}, n, observed_dt)
    metadata["provenance"]["trajectory_sources"] = [
        {"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        for name in ("steps_raw.jsonl", f"{run_id}_metrics.csv")
        if (path := Path(run_dir) / name).is_file()]
    metadata["provenance"]["evaluation_scene_config"] = cfg
    metadata["evaluation"] = {"weights": evaluation_costs}
    metadata["provenance"]["metadata_semantics"]["diagnostic_weights"] = (
        "Saved offline diagnostic weights; native planner weights are under provenance."
        if evaluation_costs is not None else
        "Historical diagnostic weights were not recorded; no current defaults substituted.")
    metadata["static"].update(
        qpos_size=nq + 4, qvel_size=nq + 4,
        block_qpos_adr=[nq, nq + 1, nq + 2], block_dof_adr=[nq, nq + 1, nq + 2],
        block_z_qpos_adr=nq + 3, block_z_dof_adr=nq + 3,
        state_layout=[f"robot_joint_{i + 1}" for i in range(nq)] + ["object_x", "object_y", "object_yaw", "object_z"])
    schema = projection_schema()
    dynamic = dict(time=times, object_pose=poses, object_velocity=object_v,
                   robot_pos=robot_xy, robot_vel=robot_xy_v,
                   robot_control=[[None] * nq for _ in range(n)],
                   qpos=[q + p + [full[6]] for q, p, full in zip(robot_q, poses, poses_3d)],
                   qvel=[q + v + z for q, v, z in zip(robot_v, object_v, object_z_v)],
                   tip_z=[p[2] for p in tips[1:]], tip_tilt=tilts[1:],
                   contact_normal_force_z=[None] * n, robot_contact_force=[None] * n,
                   control_step=indices, object_pose_3d=poses_3d, robot_joint_effort=efforts,
                   tip_z_state=[p[2] for p in tips], tip_tilt_state=tilts,
                   evaluation_costs={key: [_number(row.get(key)) for row in rows] for key in BLOCKS})
    for key in ("position_error_m", "orientation_error_rad", "physical_contact_active",
                "pusher_object_gap", "min_obstacle_clearance", "evaluation_total"):
        dynamic[key] = [_number(row.get(key)) for row in rows]
    result = {**summary, "steps_run": n, "schema": schema, **metadata, "dynamic": dynamic}
    add_execution_timing(result, read_execution_steps(run_dir) if execution_steps is None else execution_steps)
    if include_semantics:
        add_result_semantics(result, run_dir)
        add_execution_projection(result, cfg,
            read_native_execution(run_dir) if execution_native is None else execution_native,
            read_planning_updates(run_dir) if planning_updates is None else planning_updates)
    return result


def add_result_semantics(result, run_dir=None):
    """Upgrade evaluation metadata while preserving recorded states and legacy summary values."""
    dynamic = result["dynamic"]
    times = dynamic["time"]
    if not isinstance(times, list) or not times or not all(
            isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t) for t in times):
        raise ValueError("Evaluation requires nonempty, finite recorded snapshot times")
    count = len(times)
    states = SNAPSHOT_STATE_ARRAYS
    intervals = SNAPSHOT_INTERVAL_ARRAYS
    if "compute_time" in dynamic:
        timing = dynamic["compute_time"]
        if not isinstance(timing, list) or len(timing) != count - 1:
            raise ValueError(f"dynamic.compute_time must contain {count - 1} recorded entries")
        if all(value is None for value in timing):
            dynamic.pop("compute_time")
        else:
            intervals += ("compute_time",)
    for keys, length in ((states, count), (intervals, count - 1)):
        for key in keys:
            if not isinstance(dynamic.get(key), list) or len(dynamic[key]) != length:
                raise ValueError(f"dynamic.{key} must contain {length} recorded entries")
    costs = dynamic.get("evaluation_costs")
    if not isinstance(costs, dict) or any(not isinstance(v, list) or len(v) != count for v in costs.values()):
        raise ValueError("Diagnostic cost components must align with recorded snapshots")
    for key, expected in (("n_control_steps", count), ("steps_run", count - 1)):
        if key in result and result[key] != expected:
            raise ValueError(f"Legacy {key} does not match the recorded trajectory")

    parameters = result["hyperparameters"]
    observed_span = times[-1] - times[0]
    parameters["control_dt"] = (observed_span / (count - 1) if count > 1 and math.isfinite(observed_span)
                                and all(b > a for a, b in zip(times, times[1:])) else None)
    parameters["control_dt_source"] = "mean_observed_state_interval"
    saved_evaluation = result.get("evaluation") or {}
    weights = saved_evaluation.get("weights", parameters.get("costs"))
    if "weights" in saved_evaluation and "costs" in parameters and parameters["costs"] != weights:
        raise ValueError("Conflicting saved diagnostic weights")
    parameters.pop("costs", None)
    weights_source = parameters.pop("costs_source", None)
    if weights_source is not None:
        result.setdefault("provenance", {}).setdefault("metadata_semantics", {})["diagnostic_weights"] = weights_source
    position, orientation = (parameters.get(key) for key in ("goal_pos_tol", "goal_theta_tol"))
    for value in (position, orientation):
        if value is not None and (not isinstance(value, (int, float)) or isinstance(value, bool)
                                  or not math.isfinite(value) or value < 0):
            raise ValueError("Recorded evaluation thresholds must be finite and nonnegative")
    checks = []
    for pos_error, ang_error in zip(dynamic["position_error_m"], dynamic["orientation_error_rad"]):
        known = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in (position, orientation, pos_error, ang_error))
        checks.append(pos_error < position and ang_error < orientation if known else None)
    ever = True if True in checks else None if None in checks else False
    result.update(n_snapshots=count, n_recorded_intervals=count - 1,
                  evaluation={"ever_success": ever, "final_success": checks[-1],
                              "first_success_t": next((t for t, ok in zip(times, checks) if ok), None),
                              "thresholds": {"position_m": position, "orientation_rad": orientation},
                              "weights": weights,
                              "costs": {"components": costs, "total": dynamic["evaluation_total"]},
                              "success_semantics": "Simultaneous strict < comparisons on retained snapshots; unknown thresholds/errors remain null.",
                              "cost_semantics": "Offline diagnostic evaluation, not the native C3+ optimization objective; total sums available components only."})
    schema = result.setdefault("schema", {})
    schema.update(semantics_version=SNAPSHOT_SEMANTICS_VERSION,
                  indexing="n_snapshots counts retained asynchronous states; n_recorded_intervals is n_snapshots-1. "
                           "Neither counts executed controls. Entry 0 is the first observed state; no initial state is invented. "
                           "Interval tip_z/tip_tilt use state[i+1]. Keep the actual dynamic.time.",
                  state_arrays=list(states), interval_arrays=list(intervals),
                  legacy_fields={
                      "success": "Legacy common-evaluation ever-success, not native C3+ completion; use evaluation.ever_success.",
                      "t_success": "Legacy first successful retained snapshot time; use evaluation.first_success_t.",
                      "first_success_t": "Legacy common-evaluation event time from recorder object messages; may differ from the retained-snapshot evaluation.first_success_t.",
                      "n_control_steps": "Legacy retained-snapshot count; use n_snapshots. Not executed control actions.",
                      "steps_run": "Legacy adjacent recorded interval count; use n_recorded_intervals.",
                      "hyperparameters.steps": "Recorded interval count, not an executed action count or requested budget.",
                      "hyperparameters.goal_pos_tol": "Common evaluation position threshold; native thresholds are under native_controller.",
                      "hyperparameters.goal_theta_tol": "Common evaluation orientation threshold; native thresholds are under native_controller.",
                      "hyperparameters.costs": "Legacy diagnostic weights were relocated to evaluation.weights; not native controller weights.",
                      "hyperparameters.c3plus": "Legacy native configuration was relocated to provenance.c3plus and verified provenance.configuration snapshots.",
                      "dynamic.evaluation_costs": "Alias of evaluation.costs.components: offline diagnostic component series.",
                      "dynamic.evaluation_total": "Alias of evaluation.costs.total: sum of available offline components."})
    schema.setdefault("missing", {}).update(
        predicted_plans="Predicted object/robot plans were not recorded.",
        wrench_consensus_trajectories="Wrench and consensus trajectories were not recorded.")
    if "compute_time" in dynamic:
        schema["missing"].pop("compute_time", None)
    else:
        schema["missing"]["compute_time"] = "No per-step optimization timing recording; unavailable array omitted."
    schema["sampling"] = ("Asynchronous latest-message snapshots on C3_DEBUG_CURR; dynamic.time is unchanged. "
                          "Individual object/robot timestamps were not retained; no resampling or initial state is invented. "
                          "Execution time is the observed last-minus-first timestamp. control_dt is its mean per "
                          "recorded interval when all intervals are positive, not a physics or planner timestep.")
    result.setdefault("provenance", {}).setdefault("metadata_semantics", {})["control_dt"] = (
        "Mean observed state interval: (dynamic.time[-1]-dynamic.time[0])/steps_run; "
        "null unless all observed intervals are positive. Not physics or planner dt.")
    return add_recorded_semantics(result, run_dir)


def write_result_json(path, result):
    """Serialize before replacing the result, with strict JSON nulls for missing data."""
    path = Path(path)
    text = json.dumps(_json_values(result), indent=2, allow_nan=False) + "\n"
    expected = json.loads(text)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        if json.loads(temporary.read_text()) != expected:
            raise ValueError("Result JSON failed read-back verification")
        if path.exists():
            os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
