"""Check the reproducible runtime environment without advancing simulation."""
import argparse
import importlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from c3plus.configs.paths import BINARIES
from c3plus.configs.catalog import MESH_OBJECTS, SCENES
from c3plus.configs.resolver import check_scene_assets
from c3plus.runtime.lcm import check_multicast
from c3plus.runtime.processes import check_binary

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-only", action="store_true", help="Skip optional mesh-generation imports")
    parser.add_argument("--require-binaries", action="store_true", help="Check native targets and shared libraries")
    parser.add_argument("--check-scenes", action="store_true",
                        help="Parse six scenes and all four imported object profiles, including selected assets")
    args = parser.parse_args(argv)
    checks = []

    def check(label, function):
        try:
            detail = function()
            checks.append({"check": label, "passed": True, "detail": detail})
        except Exception as exc:
            checks.append({"check": label, "passed": False, "detail": str(exc)})

    def require_command(name):
        path = shutil.which(name)
        if path is None:
            raise RuntimeError(f"Missing command: {name}")
        return path

    for name in ("bazel", "git", "g++", "pkg-config", "java", "bash", "setsid", "tee", "ldd", "ffmpeg", "ffprobe", "dot"):
        check(name, lambda name=name: require_command(name))
    modules = ["pydrake", "pydrake.lcm", "pydrake.geometry", "pydrake.math",
               "pydrake.multibody.parsing", "pydrake.multibody.plant",
               "pydrake.systems.framework", "pydrake.systems.sensors", "pydrake.common.eigen_geometry",
               "numpy", "scipy", "matplotlib", "matplotlib.pyplot", "mpl_toolkits.mplot3d",
               "PIL.Image", "PIL.ImageDraw", "trimesh", "yaml"]
    if not args.runtime_only:
        modules.extend(["ruamel.yaml", "lxml.etree", "fast_simplification", "pyglet", "vhacdx"])
    for module in modules:
        check("import " + module, lambda module=module: importlib.import_module(module).__name__)

    def drake_version():
        found = version("drake")
        if found != "1.51.1":
            raise RuntimeError(f"Expected Drake 1.51.1, got {found}")
        return found
    check("Drake version", drake_version)

    check("LCM multicast", check_multicast)

    def graphviz_svg():
        result = subprocess.run(["dot", "-Tsvg"], input="digraph { check -> svg }",
                                capture_output=True, text=True)
        if result.returncode or "<svg" not in result.stdout:
            raise RuntimeError(result.stderr.strip() or "Graphviz did not produce SVG")
        return "SVG generation passed"
    check("Graphviz SVG", graphviz_svg)

    def gurobi_distribution():
        home = Path(os.environ.get("GUROBI_HOME", "/opt/gurobi/gurobi1003/linux64"))
        if not (home / "lib/libgurobi100.so").is_file():
            raise RuntimeError(f"Gurobi 10.0 shared library missing under {home}. "
                               "Rebuild the Docker image with ./docker/shell.sh --build.")
        return "Gurobi 10.0 distribution found; license requirements depend on the solver selected"
    check("Gurobi distribution", gurobi_distribution)
    def snopt_distribution():
        path = Path(os.environ.get("SNOPT_PATH", "/opt/snopt/snopt7.6.tar.gz"))
        if not path.is_file():
            raise RuntimeError(f"SNOPT build archive missing: {path}. Rebuild the Docker image with ./docker/shell.sh --build.")
        return "SNOPT build archive found"
    check("SNOPT distribution", snopt_distribution)
    if args.require_binaries:
        for name in BINARIES:
            check(name, lambda name=name: check_binary(name))
    if args.check_scenes:
        for scene in SCENES:
            check("scene " + scene, lambda scene=scene: check_scene_assets(scene))
        for object_name in MESH_OBJECTS:
            check("object open_task/" + object_name,
                  lambda object_name=object_name: check_scene_assets("open_task", object_name))
    print(json.dumps({"python": sys.executable, "checks": checks}, indent=2))
    return int(any(not c["passed"] for c in checks))

if __name__ == "__main__":
    raise SystemExit(main())
