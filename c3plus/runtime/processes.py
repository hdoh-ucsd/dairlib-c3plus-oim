"""Subprocess logging, native readiness checks and process-group cleanup."""
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import time
import json

from c3plus.configs.paths import REPO

def classify_failure(out):
    failures = []
    for process, name in (("controller", "planner.log"), ("simulator", "sim.log"),
                          ("osc", "osc.log")):
        text = (out / name).read_text(errors="replace")
        if "AllFinite(q_v)" in text:
            reason = "AllFinite(q_v)"
        elif "abort: Failure" in text and "CheckForWorkspaceLimitViolations" in text:
            reason = "workspace_limit_assertion"
        elif "SAMPLING_C3_TOPPLE_GUARD tripped" in text:
            reason = "topple_guard"
        elif "terminate called" in text or "Segmentation fault" in text:
            reason = "other_process_failure"
        else:
            continue
        failures.append({"process": process, "reason": reason})
    if not failures and any(s in (out / "launcher.log").read_text(errors="replace")
                            for s in ("Aborted", "Segmentation fault")):
        failures.append({"process": "unknown", "reason": "see_launcher_log"})
    return failures

def logged_command(command, log, env):
    """Retain complete phase logs and forward recorder progress as it arrives."""
    with log.open("w") as stream:
        stream.write(f"[COMMAND] {shlex.join(map(os.fsdecode, command))}\n")
        stream.write(f"[CWD] {shlex.quote(str(REPO))}\n")
        stream.flush()
        with subprocess.Popen(command, cwd=REPO, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, errors="replace", bufsize=1) as process:
            for line in process.stdout:
                stream.write(line)
                stream.flush()
                if re.match(r"^\[[^\]]+\] step=\d+ ", line):
                    print(line, end="", flush=True)
            return process.wait()

def check_binary(name, repo=REPO):
    """Inspect linkage and flags without constructing native simulation systems."""
    path = Path(repo) / ".build/bin/examples/sampling_c3" / name
    rebuild = "Rebuild inside Docker with python3 -m c3plus.utils build."
    if not path.is_file() or not os.access(path, os.X_OK):
        raise RuntimeError(f"Missing executable {path}. {rebuild}")
    linked = subprocess.run(["ldd", str(path)], capture_output=True, text=True, timeout=20)
    if linked.returncode or "not found" in linked.stdout + linked.stderr:
        raise RuntimeError(f"Shared-library check failed for {name}: "
                           f"{(linked.stdout + linked.stderr).strip()}. {rebuild}")
    help_result = subprocess.run([str(path), "--helpshort"], capture_output=True, text=True, timeout=20)
    help_text = help_result.stdout + help_result.stderr
    required = ["controller_params"]
    if name in ("franka_sim", "franka_osc_controller"):
        required.append("execution_logging")
    if name == "franka_sim":
        required.extend(("execution_step_budget", "execution_stop_file"))
    if name == "franka_sampling_c3_controller":
        required.append("goal_yaw_degrees")
    missing = [flag for flag in required
               if not re.search(r"(?:^|\s)-" + re.escape(flag) + r"(?:\s|=)", help_text)]
    if missing:
        raise RuntimeError(f"{name} lacks required flags: {', '.join('--' + flag for flag in missing)}. {rebuild}")
    return "executable, shared libraries, and current experiment flags verified"


def check_experiment_capabilities(repo=REPO):
    """Reject a pre-matrix planner without constructing any native systems."""
    binary = Path(repo) / ".build/bin/examples/sampling_c3/franka_sampling_c3_controller"
    response = subprocess.run([str(binary), "--print_experiment_capabilities"],
                              capture_output=True, text=True, timeout=20)
    try:
        capabilities = json.loads(response.stdout)
    except (ValueError, TypeError):
        capabilities = {}
    required = {"object_scene_support_version": 1, "mesh_objects_with_obstacles": True,
                "custom_object_footprint": True}
    if response.returncode or any(capabilities.get(key) != value for key, value in required.items()):
        raise RuntimeError("Native planner lacks full task/object support. Rebuild inside Docker with python3 -m c3plus.utils build.")
    return capabilities


def start_session(command, **kwargs):
    """Start a native process or recorder pipeline in its own session."""
    return subprocess.Popen(["setsid", *command], **kwargs)


def cleanup_sessions(processes):
    """Preserve the native trio/recorder TERM, one-second grace, KILL order."""
    for process in processes:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except OSError:
                pass
    time.sleep(1)
    for process in processes:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except OSError:
                pass
    for process in processes:
        if process is not None:
            process.wait()
