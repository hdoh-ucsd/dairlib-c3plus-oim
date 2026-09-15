"""Launch the isolated native trio and snapshot recorder with fixed lifecycle order."""
import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from c3plus.configs.paths import REPO
from c3plus.runtime.lcm import multicast_url
from c3plus.runtime.processes import cleanup_sessions, start_session


def _wait_without_recorder(processes, cap):
    """Bound an unrecorded trial: first native exit, or the cap.

    No recorder means nothing detects success, so the cap is the normal ending
    and is reported as success. A native process that exits first is reported
    with its own status so crashes stay visible to the caller.
    """
    deadline = time.monotonic() + cap
    while time.monotonic() < deadline:
        for name in ("sim", "planner", "osc"):
            process = processes[name]
            if process is not None and process.poll() is not None:
                return process.returncode
        time.sleep(0.2)
    return 0


def _supports(binary, flag):
    # Gflags help may exit nonzero; inspect its text without constructing systems.
    result = subprocess.run([str(binary), "--helpshort"], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    return bool(re.search(r"(?:^|\s)-" + re.escape(flag) + r"(?:\s|=)", result.stdout))


def launch(demo, object_name, goal, cap, port, out, controller_params=None,
           goal_yaw_degrees=None, steps=None, record=True):
    """Run one native trial.

    With ``record`` (the default) the snapshot recorder both captures telemetry
    and defines the trial's lifetime: the trial ends when the recorder exits on
    its cap, on first success, or on the execution-step budget.

    With ``record=False`` no recorder runs, so nothing observes success and no
    telemetry is written. The trial then ends when a native process exits or the
    same ``cap`` elapses, whichever happens first. This is for watching or
    profiling the algorithm; it cannot produce a result JSON or any metric.
    """
    binary_dir = REPO / ".build/bin/examples/sampling_c3"
    controller_args = []
    planner_goal_args = []
    if controller_params:
        controller_params = Path(controller_params)
        if not controller_params.is_file():
            raise ValueError(f"Missing controller YAML: {controller_params}")
        controller_params = controller_params.parent.resolve() / controller_params.name
        for name in ("franka_sim", "franka_osc_controller", "franka_sampling_c3_controller"):
            result = subprocess.run([str(binary_dir / name), "--helpshort"], stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
            def supports(flag):
                return re.search(r"(?:^|\s)-" + re.escape(flag) + r"(?:\s|=)", result.stdout)
            if not supports("controller_params"):
                raise ValueError(f"{name} does not support --controller_params. "
                                 "Rebuild with python3 -m c3plus.utils build before launching.")
            if name != "franka_sampling_c3_controller" and not supports("execution_logging"):
                raise ValueError(f"{name} lacks physical execution logging. "
                                 "Rebuild all native targets with python3 -m c3plus.utils build.")
        controller_args = [f"--controller_params={controller_params}"]
    if goal_yaw_degrees is not None:
        if not re.fullmatch(r"([+-]?90|[+-]?0)([.]0+)?", str(goal_yaw_degrees)):
            raise ValueError("GOAL_YAW_DEGREES must be -90, 0, or 90 (absolute world yaw).")
        if not _supports(binary_dir / "franka_sampling_c3_controller", "goal_yaw_degrees"):
            raise ValueError("Controller does not support --goal_yaw_degrees. "
                             "Rebuild with python3 -m c3plus.utils build before launching this campaign.")
        planner_goal_args = [f"--goal_yaw_degrees={goal_yaw_degrees}"]
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    temporary = out / "tmp"
    temporary.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TMPDIR=str(temporary))
    env["SAMPLING_C3_OBSTACLE_MODE"] = env.get("SAMPLING_C3_OBSTACLE_MODE") or "lcs_contact"
    simulation_args = ["--execution_logging=true"]
    recorder_args = []
    if steps is not None:
        if not re.fullmatch(r"[1-9][0-9]*", str(steps)):
            raise ValueError("--steps must be positive")
        stop_file = str(temporary / "execution_stop.json")
        simulation_args += [f"--execution_step_budget={steps}", f"--execution_stop_file={stop_file}"]
        recorder_args += ["--stop-file", stop_file]
    url = multicast_url(port)
    processes = dict.fromkeys(("sim", "osc", "planner", "recorder"))
    previous = {}
    hangup_received = False
    def interrupted(signum, frame):
        nonlocal hangup_received
        if signum == signal.SIGHUP:
            hangup_received = True
        raise SystemExit(128 + signum)
    try:
        for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.getsignal(sig)
            if sig != signal.SIGHUP or previous[sig] != signal.SIG_IGN:
                signal.signal(sig, interrupted)
        with ExitStack() as files:
            def native(name, binary, arguments):
                log = files.enter_context((out / f"{name}.log").open("w"))
                processes[name] = start_session([str(binary_dir / binary), *arguments],
                    cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT)
            native("osc", "franka_osc_controller", ["--is_simulation=true", f"--demo_name={demo}",
                "--robot_model=xarm6", "--execution_logging=true", f"--lcm_url={url}", *controller_args])
            native("planner", "franka_sampling_c3_controller", ["--is_simulation=true", f"--demo_name={demo}",
                "--robot_model=xarm6", f"--lcm_url={url}", *controller_args, *planner_goal_args])
            if record:
                # Retain shell pipefail/tee semantics in this single recorder pipeline.
                processes["recorder"] = start_session(["bash", "-c",
                    'set -o pipefail; recorder_log=$1; shift; "$@" 2>&1 | tee "$recorder_log"',
                    "recorder", str(out / "recorder.log"), env.get("PYTHON") or "python3",
                    "-m", "c3plus.recording.recorder", "--goal", *map(str, goal),
                    "--object-name", object_name, "--out-steps", str(out / "steps_raw.jsonl"),
                    "--out-trace", str(out / "state_trace.jsonl"), "--url", url,
                    "--duration", str(cap), "--exit-on-success", *recorder_args], cwd=REPO, env=env)
            time.sleep(3)
            native("sim", "franka_sim", [f"--demo_name={demo}", "--robot_model=xarm6", "--matched_mu",
                f"--lcm_url={url}", *controller_args, *simulation_args])
            if record:
                rc = processes["recorder"].wait()
            else:
                rc = _wait_without_recorder(processes, float(cap))
            print(f"RUN DONE {demo} -> {out}", flush=True)
            return rc if rc >= 0 else 128 - rc
    finally:
        cleanup_sessions(processes.values())
        # Bash reported signal deaths of background native jobs even when their
        # own logs were empty; preserve the failure classifier's fallback.
        for name in ("sim", "osc", "planner"):
            process = processes[name]
            if process is not None:
                message = {-signal.SIGABRT: "Aborted", -signal.SIGSEGV: "Segmentation fault"}.get(process.returncode)
                if message is not None:
                    print(f"{name}: {message}", file=sys.stderr, flush=True)
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if hangup_received:
            # An untrapped Bash hangup cleans up, then remains signal-terminated:
            # subprocess returncode -SIGHUP, or 129 as observed by a shell.
            signal.signal(signal.SIGHUP, signal.SIG_DFL)
            os.kill(os.getpid(), signal.SIGHUP)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("demo")
    parser.add_argument("object_name")
    parser.add_argument("goal", nargs=3)
    parser.add_argument("cap")
    parser.add_argument("port")
    parser.add_argument("out")
    parser.add_argument("--controller-params")
    parser.add_argument("--goal-yaw-degrees")
    parser.add_argument("--steps")
    parser.add_argument("--no-record", dest="record", action="store_false",
                        help="Skip the snapshot recorder: no telemetry, no success detection")
    args = parser.parse_args(argv)
    try:
        return launch(args.demo, args.object_name, args.goal, args.cap, args.port, args.out,
                      args.controller_params, args.goal_yaw_degrees, args.steps, args.record)
    except (ValueError, OSError) as exc:
        parser.exit(2, f"{exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
