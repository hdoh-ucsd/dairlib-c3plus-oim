"""Read exact native policy boundaries and historical dispatch telemetry."""
import json
from pathlib import Path

def read_execution_steps(run_dir):
    """Read producer-clock dispatch events, never infer them from snapshots."""
    path = Path(run_dir) / "planner.log"
    if not path.is_file():
        return None
    prefix = "[C3_EXECUTION_STEP] "
    enabled, events = False, []
    with path.open() as stream:
        for line in stream:
            if line.strip() == "[C3_EXECUTION_TIMING] monotonic_wall_time_at_execution_step":
                enabled = True
            elif line.startswith(prefix):
                events.append(json.loads(line[len(prefix):]))
    # Distinguish zero measured events from a legacy run without instrumentation.
    return events if enabled or events else None


def read_native_execution(run_dir):
    """Read native physical boundaries; never align cached debug observations."""
    path = Path(run_dir) / "sim.log"
    if not path.is_file():
        return None
    data = {"headers": [], "boundaries": [], "terminals": []}
    prefixes = {"[C3_EXECUTION_LOGGING] ": "headers",
                "[C3_EXECUTION_BOUNDARY] ": "boundaries",
                "[C3_EXECUTION_TERMINAL] ": "terminals"}
    for line in path.read_text().splitlines():
        for prefix, key in prefixes.items():
            if line.startswith(prefix):
                data[key].append(json.loads(line[len(prefix):]))
    return data if any(data.values()) else None
