"""Read a campaign selection from YAML instead of retyping CLI selectors.

The spec says *what to run* -- tasks, objects, pose pairs -- and how much of
each trial to keep: its budget (`cap`, `steps`) and its artifacts (`record`,
`video`). It does not define what a task or object is; that stays in
`examples/sampling_c3/shared_parameters/experiments.yaml`.

`campaign` loads `campaign.yaml` from the repository root when you pass no
selection flags. Passing any of `--tasks`, `--objects`, `--pairs` or
`--obstacle_cost` selects from the command line instead and ignores the file,
so a one-off run never needs the file edited.
"""
from __future__ import annotations

from pathlib import Path

import yaml

from .catalog import canonical_object, canonical_task
from .paths import REPO

DEFAULT_SPEC = REPO / "campaign.yaml"
PAIR_PRESETS = ("all", "diagonal", "smoke")
_KEYS = {"tasks", "scenes", "objects", "pairs", "obstacle_cost", "cap", "steps",
         "record", "video", "seed"}


def _sequence(value, name):
    if isinstance(value, str) or not isinstance(value, (list, tuple)):
        raise ValueError(f"campaign spec {name} must be a list")
    if not value:
        raise ValueError(f"campaign spec {name} must not be empty")
    return list(value)


def _pairs(value):
    """A preset name, or explicit [start, goal] pairs."""
    if isinstance(value, str):
        if value not in PAIR_PRESETS:
            raise ValueError(f"campaign spec pairs must be one of {list(PAIR_PRESETS)}, got {value!r}")
        return value
    pairs = []
    for entry in _sequence(value, "pairs"):
        if (not isinstance(entry, (list, tuple)) or len(entry) != 2
                or any(isinstance(v, bool) or not isinstance(v, int) for v in entry)):
            raise ValueError(f"campaign spec pair {entry!r} must be two integers [start, goal]")
        if not all(1 <= int(v) <= 5 for v in entry):
            raise ValueError(f"campaign spec pair {entry!r} must use pose IDs 1-5")
        pairs.append((int(entry[0]), int(entry[1])))
    if not pairs:
        raise ValueError("campaign spec pairs must not be empty")
    return pairs


def _flag(value, name):
    if not isinstance(value, bool):
        raise ValueError(f"campaign spec {name} must be true or false, got {value!r}")
    return value


def load_campaign_spec(path=None):
    """Parse a campaign spec. Returns a dict of the settings it declares.

    Only keys the file actually sets are returned, so a spec can carry just a
    selection and leave caps and costs to the CLI defaults.

    Raises:
        ValueError: On an unknown key or an invalid value. Failing loudly
            matters here: a misspelled task in a silently ignored key would
            run a different campaign than the file appears to describe.
    """
    path = Path(DEFAULT_SPEC if path is None else path)
    loaded = yaml.safe_load(path.read_text())
    if not isinstance(loaded, dict):
        raise ValueError(f"campaign spec {path} must contain a mapping")
    unknown = set(loaded) - _KEYS
    if unknown:
        raise ValueError(f"unknown campaign spec keys: {sorted(map(str, unknown))}")
    if "tasks" in loaded and "scenes" in loaded:
        raise ValueError("campaign spec sets both tasks and scenes; they are the same field")

    spec = {}
    tasks = loaded.get("tasks", loaded.get("scenes"))
    if tasks is not None:
        spec["scenes"] = [canonical_task(task) for task in _sequence(tasks, "tasks")]
    if loaded.get("objects") is not None:
        spec["objects"] = [canonical_object(obj) for obj in _sequence(loaded["objects"], "objects")]
    if loaded.get("pairs") is not None:
        spec["pairs"] = _pairs(loaded["pairs"])
    if loaded.get("obstacle_cost") is not None:
        cost = loaded["obstacle_cost"]
        if cost not in ("exponential", "relu", "both"):
            raise ValueError(f"campaign spec obstacle_cost must be exponential, relu or both, got {cost!r}")
        spec["obstacle_cost"] = cost
    if loaded.get("cap") is not None:
        cap = loaded["cap"]
        if isinstance(cap, bool) or not isinstance(cap, int) or cap < 1:
            raise ValueError(f"campaign spec cap must be a positive whole number of seconds, got {cap!r}")
        spec["cap"] = cap
    if loaded.get("steps") is not None:
        steps = loaded["steps"]
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
            raise ValueError(f"campaign spec steps must be a positive whole number, got {steps!r}")
        spec["steps"] = steps
    if loaded.get("video") is not None:
        spec["video"] = _flag(loaded["video"], "video")
    if loaded.get("record") is not None:
        spec["record"] = _flag(loaded["record"], "record")
        if not spec["record"]:
            # Nothing is recorded, so there is no trajectory left to render.
            # Say so here rather than letting the campaign discover it later.
            if loaded.get("video") is True:
                raise ValueError("campaign spec sets record: false with video: true; "
                                 "an unrecorded run has nothing to render")
            spec["video"] = False
    if loaded.get("seed") is not None and loaded["seed"] != 42:
        raise ValueError("This reproduction workflow uses seed 42 only")
    return spec
