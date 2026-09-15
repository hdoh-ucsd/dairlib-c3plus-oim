"""Pinned, local task-specific object poses from the OIM comparison source."""
from __future__ import annotations

import hashlib
import json
import math
from contextlib import contextmanager
from contextvars import ContextVar
from copy import deepcopy
from pathlib import Path
import re

from .paths import REPO

POSE_DIRECTORY = Path("examples/poses")
PROVENANCE_FILE = POSE_DIRECTORY / "provenance.json"
_SNAPSHOT = ContextVar("canonical_pose_snapshot", default=None)


def _snapshot(repo):
    value = _SNAPSHOT.get()
    return value if value is not None and value["repo"] == Path(repo).resolve() else None


@contextmanager
def pose_catalogue_snapshot(repo=REPO):
    """Reuse one verified catalogue during a bounded plan/check, then release it.

    File changes are visible on the next invocation outside this context. No
    process-global cache can hide edits between campaign planning and launch.
    """
    if _snapshot(repo) is not None:
        yield
        return
    directory = Path(repo).resolve()
    snapshot = {"repo": directory, "catalogue": load_pose_catalogue(directory),
                "provenance": pose_provenance(directory),
                "sources": pose_source_files(directory)}
    token = _SNAPSHOT.set(snapshot)
    try:
        yield
    finally:
        _SNAPSHOT.reset(token)


def _provenance(repo):
    path = Path(repo) / PROVENANCE_FILE
    data = json.loads(path.read_text())
    if (not isinstance(data, dict) or data.get("format") != "oim-canonical-poses/v1"
            or not re.fullmatch(r"[0-9a-f]{40}", str(data.get("source_commit", "")))
            or not isinstance(data.get("files"), dict) or not data["files"]):
        raise ValueError(f"Invalid canonical pose provenance: {path}")
    for name, record in data["files"].items():
        if (not re.fullmatch(r"[a-z][a-z0-9_]*\.yaml", name)
                or not isinstance(record, dict)
                or record.get("source_path") != str(POSE_DIRECTORY / name)
                or not re.fullmatch(r"[0-9a-f]{64}", str(record.get("sha256", "")))):
            raise ValueError(f"Invalid canonical pose file record: {name!r}")
    return data


def _source_bytes(repo, provenance):
    directory = (Path(repo) / POSE_DIRECTORY).resolve()
    for name in sorted(provenance["files"]):
        path = directory / name
        if path.resolve().parent != directory:
            raise ValueError(f"Pose file leaves its local catalogue: {path}")
        content = path.read_bytes()
        if hashlib.sha256(content).hexdigest() != provenance["files"][name]["sha256"]:
            raise ValueError(f"Canonical pose source hash mismatch: {path}")
        yield path, content


def pose_source_files(repo=REPO):
    """Verified local YAMLs and provenance, for configuration snapshots/digests."""
    if (snapshot := _snapshot(repo)) is not None:
        return snapshot["sources"]
    data = _provenance(repo)
    return (Path(repo) / PROVENANCE_FILE,
            *(path for path, _ in _source_bytes(repo, data)))


def _read_poses(content, path):
    import yaml

    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node):
        pairs = loader.construct_pairs(node, deep=True)
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate pose key {key!r}: {path}")
            result[key] = value
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    data = yaml.load(content, Loader=UniqueLoader)
    if not isinstance(data, dict) or set(data) != {"starts", "goals"}:
        raise ValueError(f"Pose catalogue requires separate starts and goals: {path}")
    for kind, poses in data.items():
        if not isinstance(poses, dict) or not poses:
            raise ValueError(f"Missing canonical {kind}: {path}")
        for key, value in poses.items():
            if not isinstance(key, str) or not re.fullmatch(r"[1-9][0-9]*", key):
                raise ValueError(f"Pose IDs must be positive integer strings: {path}: {key!r}")
            if (not isinstance(value, list) or len(value) != 3
                    or any(isinstance(x, bool) or not isinstance(x, (int, float))
                           or not math.isfinite(x) for x in value)):
                raise ValueError(f"Pose {kind}/{key} must be finite [x, y, yaw]: {path}")
    return data


def load_pose_catalogue(repo=REPO):
    """Return task -> starts/goals -> ordered string ID -> exact [x, y, yaw].

    XY is the world object-body origin in metres; yaw is radians about +Z.
    Robot joints and object support height belong to the native configuration.
    """
    if (snapshot := _snapshot(repo)) is not None:
        return deepcopy(snapshot["catalogue"])
    provenance = _provenance(repo)
    return {path.stem: _read_poses(content, path)
            for path, content in _source_bytes(repo, provenance)}


def _kind(kind):
    if kind not in ("start", "goal"):
        raise ValueError(f"Pose kind must be start or goal, got {kind!r}")
    return kind + "s"


def pose_ids(kind, task=None, repo=REPO):
    """Ordered source IDs; without a task, require identical IDs across tasks."""
    key = _kind(kind)
    catalogue = load_pose_catalogue(repo)
    if task is not None:
        if task not in catalogue:
            raise ValueError(f"No canonical poses for task {task!r}")
        return tuple(catalogue[task][key])
    orders = [tuple(poses[key]) for poses in catalogue.values()]
    if any(order != orders[0] for order in orders[1:]):
        raise ValueError(f"Canonical {kind} ID ordering differs across tasks")
    return orders[0]


def resolve_pose(task, kind, pose_id, repo=REPO):
    """Select an exact local source pose; missing IDs/tasks never use defaults."""
    key = _kind(kind)
    catalogue = load_pose_catalogue(repo)
    if task not in catalogue:
        raise ValueError(f"No canonical poses for task {task!r}")
    poses = catalogue[task][key]
    if isinstance(pose_id, bool) or str(pose_id) not in poses:
        raise ValueError(f"No canonical {kind} {pose_id!r} for {task}; available: {', '.join(poses)}")
    return list(poses[str(pose_id)])


def pose_provenance(repo=REPO):
    """Verified source identity and a checkout-independent whole-catalogue hash."""
    if (snapshot := _snapshot(repo)) is not None:
        return deepcopy(snapshot["provenance"])
    data = _provenance(repo)
    sources = list(_source_bytes(repo, data))
    provenance_bytes = (Path(repo) / PROVENANCE_FILE).read_bytes()
    digest = hashlib.sha256()
    digest.update(str(PROVENANCE_FILE).encode() + b"\0" + provenance_bytes)
    for path, content in sources:
        digest.update(str(POSE_DIRECTORY / path.name).encode() + b"\0" + content)
    return {**data, "local_directory": str(POSE_DIRECTORY),
            "provenance_sha256": hashlib.sha256(provenance_bytes).hexdigest(),
            "catalogue_sha256": digest.hexdigest()}
