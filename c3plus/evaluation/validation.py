"""Verify result alignment and original recordings before packaging."""
import csv
from copy import deepcopy
import io
import math

from .serialization import _json_values
from .exporter import project_result, add_result_semantics, add_execution_projection

def _validate(result, recording, cfg, directory):
    if not isinstance(result.get("schema"), dict) or not isinstance(result.get("dynamic"), dict):
        raise ValueError("Enrich the result with postprocess --export-only before compacting")
    rows = list(csv.DictReader(io.StringIO(recording["metrics_csv"])))
    if not rows:
        raise ValueError("The original metrics CSV is empty")
    static = result.get("static", {})
    if static.get("goal") != cfg.get("goal") or static.get("object_footprint_body") != cfg.get("footprint"):
        raise ValueError("Result goal or footprint does not match the saved evaluation configuration")
    for key, column in (("final_position_error", "position_error_m"),
                        ("final_orientation_error", "orientation_error_rad"),
                        ("sim_time_end", "sim_time")):
        actual, expected = float(result[key]), float(rows[-1][column])
        if not math.isfinite(actual) or not math.isclose(actual, expected, abs_tol=1e-9, rel_tol=0):
            raise ValueError(f"Result {key} does not match the last recorded metrics row")
    hyperparameters = result.get("hyperparameters", {})
    projected = project_result(directory, result["scenario"], result["run_id"], cfg, result,
                               recording["steps_raw"], rows,
                               pos_tol=hyperparameters.get("goal_pos_tol"),
                               ang_tol=hyperparameters.get("goal_theta_tol"),
                               evaluation_costs=(result.get("evaluation") or {}).get("weights", hyperparameters.get("costs")),
                               include_semantics=False, execution_steps=recording.get("execution_steps"))
    if "execution_timing" in result:
        if "execution_steps" not in recording or result["execution_timing"] != projected.get("execution_timing"):
            raise ValueError("Execution timing disagrees with the recorded dispatch events")
    def without_unavailable_timing(dynamic):
        values = dict(dynamic)
        timing = values.get("compute_time")
        if isinstance(timing, list) and len(timing) == result["steps_run"] and all(value is None for value in timing):
            values.pop("compute_time")
        return values
    aligned = (result.get("execution") or {}).get("alignment") == "physical_policy_boundaries_v1"
    snapshot_dynamic = recording.get("snapshot_dynamic") if aligned else result["dynamic"]
    if (result.get("steps_run") != projected["steps_run"]
            or without_unavailable_timing(snapshot_dynamic) != without_unavailable_timing(_json_values(projected["dynamic"]))):
        raise ValueError("Result trajectory does not faithfully represent the raw steps and CSV")
    # Recheck additive semantics against the original embedded metadata. The
    # temporary projection above may have no native files after prior cleanup.
    semantics = deepcopy(result)
    semantics["dynamic"] = deepcopy(snapshot_dynamic)
    add_result_semantics(semantics, directory)
    if aligned:
        native = recording.get("execution_native")
        if native is None:
            raise ValueError("Missing original native execution boundary records")
        add_execution_projection(semantics, cfg, native, recording.get("planning_updates"))
        for key in ("dynamic", "execution", "planning"):
            if result[key] != _json_values(semantics[key]):
                raise ValueError(f"Result {key} disagrees with native physical execution records")
        if hyperparameters.get("steps") != semantics["hyperparameters"]["steps"]:
            raise ValueError("Result execution step budget disagrees with native configuration")
    if result["schema"].get("semantics_version", 0) >= 3:
        for key in ("control_dt", "control_dt_source"):
            if hyperparameters.get(key) != semantics["hyperparameters"].get(key):
                raise ValueError(f"Result hyperparameters.{key} disagrees with the recorded timestamps")
    for key in ("evaluation", "native_controller", "n_snapshots", "n_recorded_intervals"):
        if key in result and result[key] != semantics[key]:
            raise ValueError(f"Result {key} disagrees with the recorded data")
    trace = recording["state_trace"]
    if not trace:
        raise ValueError("The original replay trace is empty")
    for frame in trace:
        values = [frame["t"], *frame["q"], *frame["obj"]]
        if len(frame["obj"]) != 7 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError("Invalid original replay trace")
