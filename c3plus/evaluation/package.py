"""Keep a verified experiment as one result JSON and one video."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile

import yaml

from c3plus.configs.paths import REPO
from c3plus.recording.execution import read_execution_steps, read_native_execution
from c3plus.recording.planning import read_planning_updates
from .serialization import _json
from .metadata import add_recorded_semantics
from .validation import _validate

LOGS = {f"{name}.log" for name in ("launcher", "planner", "sim", "osc", "recorder",
                                      "postprocess", "render", "cost_fig", "package")}

def _digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {"size_bytes": path.stat().st_size, "sha256": digest.hexdigest()}


def _result_path(directory, run_id=None):
    if run_id is not None:
        if not isinstance(run_id, str) or Path(run_id).name != run_id or run_id in ("", ".", ".."):
            raise ValueError("Invalid run ID")
        return directory / f"{run_id}_result.json"
    paths = list(directory.glob("*_result.json"))
    if len(paths) != 1:
        raise ValueError("Expected exactly one result JSON in the run directory")
    return paths[0]


def load_status(run_dir, run_id=None):
    directory = Path(run_dir)
    path = directory / "runtime_status.json"
    if path.is_file():
        status = _json(path.read_text())
    else:
        if not directory.exists() or (run_id is None and not list(directory.glob("*_result.json"))):
            return {}
        result_path = _result_path(directory, run_id)
        if not result_path.exists():
            return {}
        result = _json(result_path.read_text())
        status = result.get("runtime_status", {})
    if not isinstance(status, dict):
        raise ValueError("Runtime status must be a mapping")
    return status


def completion(run_dir, run_id=None):
    """Recognize both legacy completion markers and verified compact bundles."""
    directory = Path(run_dir)
    try:
        path = _result_path(directory, run_id)
        if path.is_symlink():
            return False
        result = _json(path.read_text())
        identity = result["run_id"]
        if path.name != f"{identity}_result.json":
            return False
        video = directory / f"{identity}.mp4"
        if video.is_symlink() or not video.is_file() or not video.stat().st_size:
            return False
        package = result.get("package")
        if package is None:
            return (directory / "RUN_COMPLETE").is_file()
        recorded = package.get("video", {})
        return (package.get("status") == "complete" and package.get("cleanup_complete") is True
                and recorded.get("file") == video.name
                and all(recorded.get(key) == value for key, value in _digest(video).items()))
    except (OSError, ValueError, TypeError, KeyError):
        return False


def _inventory(directory, run_id):
    allowed = LOGS | {"RUN_COMPLETE", "runtime_status.json", "evaluation_scene_config.yaml",
                     "steps_raw.jsonl", "state_trace.jsonl", f"{run_id}_metrics.csv",
                     f"{run_id}_manifest.yaml", f"{run_id}_cost_diagnostics.png",
                     f"{run_id}_eval_metrics.png", f"{run_id}_result.json", f"{run_id}.mp4"}
    inventory = {}
    for path in sorted(directory.rglob("*")):
        relative = path.relative_to(directory)
        if path.is_symlink():
            raise ValueError(f"Refusing a symlink in the run: {relative}")
        if relative.parts[0] not in {"config", "tmp"} and str(relative) not in allowed:
            raise ValueError(f"Unrecognized run artifact: {relative}")
        if path.is_dir():
            continue
        if not path.is_file():
            raise ValueError(f"Not a regular run artifact: {relative}")
        if relative.parts[0] == "config" and path.suffix.lower() not in {".yaml", ".yml", ".json", ".sdf", ".obj"}:
            raise ValueError(f"Unrecognized configuration artifact: {relative}")
        if path.name not in {f"{run_id}_result.json", f"{run_id}.mp4"} or len(relative.parts) != 1:
            inventory[str(relative)] = _digest(path)
    return inventory


def _video(path):
    if path.is_symlink() or not path.is_file() or not path.stat().st_size:
        raise ValueError("A nonempty MP4 is required before compacting a run")
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_streams",
                            "-show_format", "-of", "json", str(path)],
                           check=True, capture_output=True, text=True, timeout=30)
    metadata = _json(probe.stdout)
    streams = metadata.get("streams", [])
    duration = float(metadata.get("format", {}).get("duration", 0))
    if (not streams or streams[0].get("codec_type") != "video"
            or not streams[0].get("codec_name") or int(streams[0].get("width", 0)) <= 0
            or int(streams[0].get("height", 0)) <= 0 or not math.isfinite(duration) or duration <= 0):
        raise ValueError("MP4 has no usable video stream")
    frames = streams[0].get("nb_frames")
    if frames not in (None, "N/A") and int(frames) <= 0:
        raise ValueError("MP4 has no video frames")
    return {"file": path.name, **_digest(path), "probe": metadata}


def _yaml(text, name):
    result = yaml.safe_load(text)
    if not isinstance(result, dict):
        raise ValueError(f"Expected a mapping in {name}")
    return _json(json.dumps(result, allow_nan=False))


def _logs(path):
    lines = path.read_text(errors="replace").splitlines()
    interesting = set()
    for index, line in enumerate(lines):
        if re.search(r"\b(warn(?:ing)?|error|fail(?:ed|ure)?|abort(?:ed)?|exception|traceback|"
                     r"assert(?:ion)?|segmentation|topple|invalid|fatal)\b", line, re.I):
            interesting.update(range(max(0, index - 2), min(len(lines), index + 4)))
    return {**_digest(path), "line_count": len(lines),
            "commands": [line for line in lines if line.startswith(("[COMMAND] ", "[CWD] "))],
            "warnings_and_errors": [{"line": i + 1, "text": lines[i]} for i in sorted(interesting)],
            "tail": [line[:4096] for line in lines[-40:]],
            "tail_limits": {"lines": 40, "characters_per_line": 4096}}


def _configuration(directory, inventory):
    files, assets = {}, {}
    for name, digest in inventory.items():
        path = directory / name
        if not name.startswith("config/") and name not in {"evaluation_scene_config.yaml"} and not name.endswith("_manifest.yaml"):
            continue
        if path.suffix.lower() == ".obj":
            relative = Path(name).relative_to("config/repository")
            source = (REPO / relative).resolve()
            if not source.is_relative_to(REPO.resolve()) or not source.is_file() or _digest(source) != digest:
                raise ValueError(f"Snapshot mesh has no identical repository asset: {name}")
            assets[name] = {**digest, "source_path": str(relative)}
        else:
            text = path.read_bytes().decode("utf-8")
            data = _yaml(text, name) if path.suffix.lower() in {".yaml", ".yml"} else _json(text) if path.suffix.lower() == ".json" else None
            files[name] = {**digest, "text": text}
            if data is not None:
                files[name]["data"] = data
    return {"files": files, "repository_assets": assets}


def _write(path, data):
    encoded = json.dumps(data, allow_nan=False, indent=2) + "\n"
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        if _json(temporary.read_text()) != data:
            raise ValueError("Compact result failed read-back verification")
        os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
        if _json(path.read_text()) != data:
            raise ValueError("Saved compact result failed read-back verification")
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def compact_run(run_dir, run_id=None, status=None, require_legacy_complete=True):
    """Validate and embed original data before removing known transient files.

    The caller must hold sampling_c3_run.lock. Cleanup is retriable: the first
    atomic replacement contains every recording and cleanup inventory, and the
    final replacement marks cleanup_complete only after all removals succeed.
    """
    directory = Path(run_dir)
    if directory.is_symlink():
        raise ValueError("Refusing to compact a symlinked run directory")
    directory = directory.resolve()
    path = _result_path(directory, run_id)
    result = _json(path.read_text())
    run_id = result["run_id"]
    if path.name != f"{run_id}_result.json":
        raise ValueError("Result filename and run ID disagree")
    inventory = _inventory(directory, run_id)
    package = result.get("package") or {}
    retry = package.get("status") == "complete" and package.get("cleanup_complete") is False
    if package.get("cleanup_complete") is True:
        if inventory or not completion(directory, run_id):
            raise ValueError("The completed bundle was changed after cleanup")
        return result
    if require_legacy_complete and not retry and not (directory / "RUN_COMPLETE").is_file():
        raise ValueError("Only completed runs can be compacted")
    runtime = status if status is not None else result.get("runtime_status") if retry else load_status(directory, run_id)
    if not isinstance(runtime, dict):
        raise ValueError("Missing runtime status")
    runtime = _json(json.dumps(runtime, allow_nan=False))
    for name, digest in (runtime.get("config_sha256") or {}).items():
        if not retry and name not in inventory:
            raise ValueError(f"Recorded configuration snapshot is missing: {name}")
        if name in inventory and inventory[name]["sha256"] != digest:
            raise ValueError(f"Saved configuration changed after the run: {name}")
    for key, expected in (("run_id", run_id), ("scene", result["scenario"])):
        if key in runtime and runtime[key] != expected:
            raise ValueError(f"Runtime {key} does not match this result")
    if any(runtime.get(key) != 0 for key in ("wrapper_rc", "postprocess_rc", "render_rc")):
        raise ValueError("Successful launch, postprocessing and rendering are required before cleanup")
    if any(runtime.get(key) is False for key in ("seed_verified", "goal_yaw_verified")):
        raise ValueError("A run with failed seed or goal verification cannot be compacted")
    video = _video(directory / f"{run_id}.mp4")
    if retry:
        expected_inventory = package["cleanup_inventory"]
        if any(expected_inventory.get(name) != digest for name, digest in inventory.items()):
            raise ValueError("A remaining artifact changed after partial cleanup")
        if any(package["video"].get(key) != video[key] for key in ("file", "size_bytes", "sha256")):
            raise ValueError("The video changed after partial cleanup")
        recording = result["recording"]
        cfg = result["provenance"]["configuration"]["files"]["evaluation_scene_config.yaml"]["data"]
    else:
        recording = dict(result.get("recording") or {})
        recording.update({key: [_json(line) for line in (directory / filename).read_text().splitlines() if line.strip()]
                          for key, filename in (("steps_raw", "steps_raw.jsonl"), ("state_trace", "state_trace.jsonl"))})
        recording["metrics_csv"] = (directory / f"{run_id}_metrics.csv").read_bytes().decode("utf-8")
        execution_steps = read_execution_steps(directory)
        if execution_steps is not None:
            recording["execution_steps"] = execution_steps
        native = read_native_execution(directory)
        if native is not None:
            recording["execution_native"] = native
            recording["planning_updates"] = read_planning_updates(directory)
            if (result.get("execution") or {}).get("step_budget") != runtime.get("execution_step_budget"):
                raise ValueError("Native execution budget disagrees with the recorded launch arguments")
        configuration = _configuration(directory, inventory)
        cfg = configuration["files"]["evaluation_scene_config.yaml"]["data"]
        logs = {name: _logs(directory / name) for name in sorted(LOGS & inventory.keys())}
        final = None
        if "recorder.log" in inventory:
            for line in (directory / "recorder.log").read_text().splitlines():
                if line.startswith("FINAL "):
                    final = _json(line[6:])
        result.update(recording=recording, runtime_status=runtime,
                      diagnostics={"logs": logs, "recorder_final": final})
        result.setdefault("provenance", {})["configuration"] = configuration
    # Verify the source-state envelope against runtime's hash before deleting
    # it, and point final metadata to the verified embedded copy.
    add_recorded_semantics(result, directory)
    _validate(result, recording, cfg, directory)
    if _inventory(directory, run_id) != inventory:
        raise ValueError("Run artifacts changed during compaction")
    if _digest(directory / video["file"]) != {key: video[key] for key in ("size_bytes", "sha256")}:
        raise ValueError("Video changed during compaction")
    result["package"] = {"status": "complete", "cleanup_complete": False, "video": video,
                         "cleanup_inventory": package["cleanup_inventory"] if retry else inventory}
    _write(path, result)
    # The verified JSON now contains the original recordings and configuration.
    # Delete only paths checked above, then remove their empty parent folders.
    for name in sorted(inventory):
        target = directory / name
        if (target.is_symlink() or not target.resolve().is_relative_to(directory)
                or not target.is_file() or _digest(target) != inventory[name]):
            raise ValueError(f"Run artifact changed before cleanup: {name}")
        target.unlink()
    for name in ("config", "tmp"):
        root = directory / name
        if root.exists():
            if root.is_symlink() or not root.resolve().is_relative_to(directory):
                raise ValueError(f"Run directory changed before cleanup: {name}")
            for folder in sorted((p for p in root.rglob("*") if p.is_dir()), reverse=True):
                folder.rmdir()
            root.rmdir()
    if {item.name for item in directory.iterdir()} != {path.name, video["file"]}:
        raise ValueError("Unexpected artifacts appeared during cleanup")
    result["package"]["cleanup_complete"] = True
    _write(path, result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        cache = REPO / ".cache"
        cache.mkdir(exist_ok=True)
        with (cache / "sampling_c3_run.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("Another managed run or packaging job is active") from exc
            result = compact_run(args.run_dir)
        print(f"[COMPACT] {result['run_id']} JSON + MP4: {args.run_dir}")
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
