"""Read native planning-update identities independently of execution."""
import json
from pathlib import Path

def read_planning_updates(run_dir):
    path = Path(run_dir) / "planner.log"
    if not path.is_file():
        return None
    prefix = "[C3_PLANNING_UPDATE] "
    return [json.loads(line[len(prefix):]) for line in path.read_text().splitlines()
            if line.startswith(prefix)]
