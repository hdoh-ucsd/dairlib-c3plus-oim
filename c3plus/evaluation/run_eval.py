"""Evaluate completed C3+ runs without loading their raw recording payloads."""

import argparse
import json
import mmap
import re
from collections import defaultdict
from pathlib import Path

from .metrics import _SETTING_KEYS, aggregate_metrics, trial_metrics
from .serialization import _json


def _fields(*keys):
    return dict.fromkeys(keys, True)


# Keep only metric inputs and scalar experiment settings. In particular, raw
# states, provenance, controller snapshots and videos never enter Python memory.
_SETTINGS = (*_SETTING_KEYS, "object", "steps", "start", "start_index", "goal", "goal_index")
_PROJECTION = {
    "schema": _fields("version", "semantics_version"),
    "run": _fields(*_SETTINGS, "task", "algorithm", "run_id"),
    "hyperparameters": _fields(*_SETTINGS),
    "static": _fields("goal"),
    "dynamic": _fields("object_pose", "time", "compute_time"),
    "execution": _fields("alignment", "n_steps_executed", "step_budget", "sim_time",
                         "wall_time", "step_wall_time", "frequency_hz"),
    "evaluation": {"thresholds": _fields("position_m", "orientation_rad")},
    "planning": _fields("n_updates"),
    "recording": _fields("n_snapshots"),
}
_SPACE = re.compile(rb"[ \t\r\n]*")
_STRUCTURE = re.compile(rb'[{}\[\]"]')
_STRING_END = re.compile(rb'["\\\x00-\x1f]')
_PRIMITIVE = re.compile(rb'(?:true|false|null|-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)')


class _ProjectionReader:
    """Walk mapped JSON, decoding selected fields with the shared strict parser.

    Unselected containers are structurally skipped, not deserialized or copied.
    Their raw telemetry is deliberately outside evaluation validation. All
    selected values and the enclosing object structure are strictly parsed.
    """

    def __init__(self, data):
        self.data = data
        self.pos = 0

    def space(self):
        self.pos = _SPACE.match(self.data, self.pos).end()

    def string_end(self, start):
        pos = start + 1
        while True:
            match = _STRING_END.search(self.data, pos)
            if match is None:
                raise ValueError("Unterminated JSON string")
            token = self.data[match.start()]
            if token == 34:
                return match.end()
            if token != 92:
                raise ValueError("Control character in JSON string")
            pos = match.end()
            escape = self.data[pos:pos + 1]
            if escape == b"u":
                if not re.fullmatch(rb"[0-9a-fA-F]{4}", self.data[pos + 1:pos + 5]):
                    raise ValueError("Invalid JSON Unicode escape")
                pos += 5
            elif escape and escape in b'"\\/bfnrt':
                pos += 1
            else:
                raise ValueError("Invalid JSON string escape")

    def value_end(self):
        start = self.pos
        token = self.data[start:start + 1]
        if token == b'"':
            return self.string_end(start)
        if token not in (b"{", b"["):
            match = _PRIMITIVE.match(self.data, start)
            if match is None:
                raise ValueError(f"Invalid JSON value at byte {start}")
            return match.end()
        stack = [token]
        pos = start + 1
        while stack:
            match = _STRUCTURE.search(self.data, pos)
            if match is None:
                raise ValueError("Unterminated JSON container")
            token = match.group()
            pos = match.end()
            if token == b'"':
                pos = self.string_end(match.start())
            elif token in (b"{", b"["):
                stack.append(token)
            elif stack.pop() != (b"{" if token == b"}" else b"["):
                raise ValueError("Mismatched JSON container")
        return pos

    def read(self, selection):
        self.space()
        if isinstance(selection, dict) and self.data[self.pos:self.pos + 1] == b"{":
            self.pos += 1
            result, seen = {}, set()
            self.space()
            if self.data[self.pos:self.pos + 1] == b"}":
                self.pos += 1
                return result
            while True:
                self.space()
                if self.data[self.pos:self.pos + 1] != b'"':
                    raise ValueError("Expected JSON object key")
                end = self.string_end(self.pos)
                key = _json(self.data[self.pos:end])
                if key in seen:
                    raise ValueError(f"Duplicate JSON key: {key}")
                seen.add(key)
                self.pos = end
                self.space()
                if self.data[self.pos:self.pos + 1] != b":":
                    raise ValueError("Expected colon after JSON key")
                self.pos += 1
                value = self.read(selection.get(key, False))
                if key in selection:
                    result[key] = value
                self.space()
                token = self.data[self.pos:self.pos + 1]
                self.pos += 1
                if token == b"}":
                    return result
                if token != b",":
                    raise ValueError("Expected comma or closing JSON object")
        end = self.value_end()
        value = _json(self.data[self.pos:end]) if selection is not False else None
        self.pos = end
        return value


def read_metric_fields(path):
    """Read just the execution metric projection of one result file."""
    with Path(path).open("rb") as source:
        if not source.seek(0, 2):
            raise ValueError("Empty JSON file")
        with mmap.mmap(source.fileno(), 0, access=mmap.ACCESS_READ) as data:
            reader = _ProjectionReader(data)
            result = reader.read(_PROJECTION)
            reader.space()
            if reader.pos != len(data):
                raise ValueError("Trailing content after JSON value")
    return result


def collect_trials(runs_dir, out_dir=None):
    """Discover result artifacts and retain compact records, one file at a time."""
    runs_dir = Path(runs_dir).resolve()
    out_dir = Path(out_dir).resolve() if out_dir else None
    excluded = out_dir if out_dir and out_dir != runs_dir and out_dir.is_relative_to(runs_dir) else None
    if not runs_dir.is_dir():
        raise ValueError(f"Runs directory does not exist: {runs_dir}")
    trials, seen = [], set()
    # *_result.json is the existing exporter/package artifact contract. Thus
    # manifests, runtime state, configuration and our aggregate are excluded.
    for path in sorted(runs_dir.rglob("*_result.json")):
        if path.is_symlink() or (excluded and path.is_relative_to(excluded)):
            continue
        try:
            run = read_metric_fields(path)
            schema = run.get("schema") if isinstance(run, dict) else None
            if not isinstance(schema, dict) or schema.get("version") != "c3plus-reference-projection-v1":
                continue
            trial = trial_metrics(run)
            del run
            if trial["run_id"] in seen:
                raise ValueError(f"Duplicate run_id: {trial['run_id']}")
            seen.add(trial["run_id"])
            trial["source_file"] = str(path.relative_to(runs_dir))
            trials.append(trial)
        except (ValueError, OSError) as error:
            raise ValueError(f"{path}: {error}") from error
    if not trials:
        raise ValueError(f"No modern C3+ *_result.json trials found under {runs_dir}")
    return trials


def varying_settings(trials):
    varying = {}
    keys = set().union(*(trial["settings"] for trial in trials))
    for key in sorted(keys, key=lambda key: (key != "object", key)):
        values = {json.dumps(trial["settings"].get(key), sort_keys=True) for trial in trials}
        if len(values) > 1:
            varying[key] = [json.loads(value) for value in sorted(values)]
    return varying


def evaluate(runs_dir, out_dir=None):
    trials = collect_trials(runs_dir, out_dir)
    groups = defaultdict(list)
    for trial in trials:
        groups[(trial["task"], trial["method"])].append(trial)
    results = []
    warnings = [f"{trial['object']} ({trial['source_file']}): {warning}"
                for trial in trials for warning in trial["warnings"]]
    for (task, method), members in sorted(groups.items()):
        result = {"task": task, "method": method, **aggregate_metrics(members),
                  "averaged_over": varying_settings(members)}
        for field in result["averaged_over"]:
            if field != "object":
                warnings.append(f"{task}/{method}: experiment setting {field} varies across trials.")
        warnings.extend(f"{task}/{method}: {warning}" for warning in result["warnings"])
        results.append(result)
    return {
        "schema": "c3plus-aggregate-evaluation-v2",
        "source_runs_directory": str(Path(runs_dir).resolve()),
        "n_runs": len(trials), "n_task_groups": len({trial['task'] for trial in trials}),
        "grouping": ["task", "method"],
        "averaged_over": varying_settings(trials), "results": results, "trials": trials,
        "metric_definitions": {
            "success": "First execution endpoint with position and wrapped yaw errors strictly below recorded tolerances.",
            "eps_d": "Position error at first successful execution endpoint, or final endpoint on failure (m); arithmetic per-trial mean in groups.",
            "eps_d_success": "Arithmetic mean eps_d over successful trials only (m).",
            "eps_o": "Absolute wrapped yaw error at the same endpoint as eps_d (rad); arithmetic per-trial mean in groups.",
            "eps_o_success": "Arithmetic mean eps_o over successful trials only (rad).",
            "trajectory_mean_position_error": "Mean position error over executed endpoints through first success, or all endpoints on failure (m); arithmetic per-trial mean in groups.",
            "trajectory_mean_position_error_success": "Arithmetic mean trajectory_mean_position_error over successful trials only (m).",
            "trajectory_mean_orientation_error": "Mean absolute wrapped yaw error over executed endpoints through first success, or all endpoints on failure (rad); arithmetic per-trial mean in groups.",
            "trajectory_mean_orientation_error_success": "Arithmetic mean trajectory_mean_orientation_error over successful trials only (rad).",
            "theta": "Legacy alias for trajectory_mean_orientation_error (rad), not the success-endpoint orientation error.",
            "steps": "First successful outer execution step; configured execution-step budget on failure; null if absent.",
            "frequency_hz": "Arithmetic mean of per-run 1 / mean(outer execution wall durations), using all recorded steps.",
            "execution_time": "Simulation time to first success; recorded physical execution span on failure (s).",
            "missing_values": "Means exclude unavailable values; available records each denominator.",
        },
        "warnings": warnings,
    }


_COLUMNS = (("task", "task", None), ("method", "method", None), ("n", "n", "d"),
            ("SR", "SR", ".2f"), ("eps_d", "eps_d", ".3f"),
            ("eps_d^s", "eps_d_success", ".3f"), ("eps_o", "eps_o", ".3f"),
            ("eps_o^s", "eps_o_success", ".3f"),
            ("steps", "steps", ".0f"), ("f (Hz)", "frequency_hz", ".2f"),
            ("T (s)", "execution_time", ".2f"))


def format_table(results, output_format="text"):
    headers = [label for label, _, _ in _COLUMNS]
    rows = [["-" if row[key] is None else format(row[key], spec) if spec else str(row[key])
             for _, key, spec in _COLUMNS] for row in results]
    if output_format == "markdown":
        def markdown(row):
            return "| " + " | ".join(value.replace("|", "\\|").replace("\n", " ") for value in row) + " |"
        return "\n".join([markdown(headers), markdown(["---"] * len(headers)), *map(markdown, rows)])
    if output_format == "latex":
        escapes = {"\\": r"\textbackslash{}", "_": r"\_", "^": r"\textasciicircum{}",
                   "&": r"\&", "%": r"\%", "$": r"\$", "#": r"\#",
                   "{": r"\{", "}": r"\}", "~": r"\textasciitilde{}"}
        def latex(row):
            return " & ".join("".join(escapes.get(char, char) for char in value) for value in row) + r" \\"
        columns = "ll" + "r" * (len(headers) - 2)
        return "\n".join([r"\begin{tabular}{" + columns + "}", latex(headers), r"\hline",
                          *map(latex, rows), r"\end{tabular}"])
    widths = [max(map(len, column)) for column in zip(headers, *rows)]
    def line(row):
        return "  ".join(value.ljust(width) if i < 2 else value.rjust(width)
                         for i, (value, width) in enumerate(zip(row, widths)))
    return "\n".join([line(headers), "-" * len(line(headers)), *map(line, rows)])


def format_report(summary, output_format="text"):
    lines = [f"{summary['n_runs']} runs -> {summary['n_task_groups']} task groups"]
    for group in summary["results"]:
        fields = group["averaged_over"]
        description = "; ".join(f"{key} ({', '.join(str(value) for value in values)})"
                                for key, values in fields.items()) or "none"
        prefix = f"{group['task']}/{group['method']} " if len(summary["results"]) > 1 else ""
        lines.append(f"{prefix}averaged over: {description}")
    lines.extend(["", format_table(summary["results"], output_format)])
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-dir", type=Path, required=True, help="Directory of completed C3+ run results (searched recursively)")
    parser.add_argument("--out-dir", type=Path, help="Save compact JSON and text summaries here (default: print only)")
    parser.add_argument("--format", choices=("text", "markdown", "latex"), default="text", help="Terminal table format")
    parser.add_argument("--diagnostics", action="store_true", help="Print one compact metric record per run")
    args = parser.parse_args(argv)
    try:
        summary = evaluate(args.runs_dir, args.out_dir)
        if args.diagnostics:
            keys = ("object", "success", "steps_to_goal", "execution_step_budget", "execution_steps_recorded",
                    "recorder_snapshots", "planning_updates", "execution_time", "frequency_hz", "eps_d", "eps_o",
                    "trajectory_mean_position_error", "trajectory_mean_orientation_error", "theta")
            for trial in summary["trials"]:
                print(json.dumps({key: trial[key] for key in keys}, allow_nan=False))
            print()
        print(format_report(summary, args.format), end="")
        if args.out_dir:
            args.out_dir.mkdir(parents=True, exist_ok=True)
            name = args.runs_dir.resolve().name
            outputs = {args.out_dir / f"{name}.json": json.dumps(summary, indent=2, allow_nan=False) + "\n",
                       args.out_dir / f"{name}.txt": format_report(summary)}
            if args.format != "text":
                suffix = "md" if args.format == "markdown" else "tex"
                outputs[args.out_dir / f"{name}.{suffix}"] = format_table(summary["results"], args.format) + "\n"
            for path in outputs:
                if path.is_symlink() or path.resolve() in {args.runs_dir.resolve() / trial["source_file"] for trial in summary["trials"]}:
                    raise ValueError(f"Refusing to overwrite result or symlink: {path}")
            for path, contents in outputs.items():
                path.write_text(contents, encoding="utf-8")
                print(f"Saved: {path}")
    except (ValueError, OSError) as error:
        parser.exit(1, f"Evaluation failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
