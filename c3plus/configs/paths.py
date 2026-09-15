"""Canonical checkout paths and selected asset references."""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[2]
RUNTIME_DIR = REPO / "c3plus/runtime"
CONFIG_DIR = Path(__file__).resolve().parent / "scenes"
EXPERIMENTS_FILE = Path("examples/sampling_c3/shared_parameters/experiments.yaml")
BINARIES = ("franka_sim", "franka_osc_controller", "franka_sampling_c3_controller")
BUILD_TARGETS = tuple(f"//examples/sampling_c3:{name}" for name in BINARIES)


def _yaml_references(value, repo):
    if isinstance(value, dict):
        for item in value.values():
            yield from _yaml_references(item, repo)
    elif isinstance(value, list):
        for item in value:
            yield from _yaml_references(item, repo)
    elif (isinstance(value, str) and value.startswith(("examples/", "common/"))
          and value.endswith((".yaml", ".yml"))):
        yield repo / value

def model_assets(composed, repo=REPO):
    """Find selected object models and their meshes for hashing and snapshots."""
    repo = Path(repo).resolve()
    pending = [repo / path for role in ("controller", "simulation")
               for path in composed[role]["object_models"]]
    pending.extend(repo / path for path in composed["controller"].get("sampling_mesh_files", []))
    paths = set()
    while pending:
        path = pending.pop().resolve()
        if path in paths:
            continue
        path.relative_to(repo)  # Only checkout assets are reproducible here.
        if not path.is_file():
            raise FileNotFoundError(path)
        paths.add(path)
        if path.suffix == ".sdf":
            document = path.read_text()
            # Drake accepts its historical extension prefix without an XML
            # namespace declaration. Supply it only to this read-only parser.
            if "xmlns:drake=" not in document:
                document = document.replace("<sdf", '<sdf xmlns:drake="http://drake.mit.edu"', 1)
            for uri in ET.fromstring(document).iter("uri"):
                value = (uri.text or "").strip()
                if "://" in value:
                    raise ValueError(f"Object model must reference local mesh assets: {value}")
                pending.append(repo / value if value.startswith("examples/") else path.parent / value)
    return sorted(paths)
