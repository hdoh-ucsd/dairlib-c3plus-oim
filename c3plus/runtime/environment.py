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
import socket
import fcntl

from c3plus.configs.paths import BINARIES, REPO
from c3plus.configs.catalog import MESH_OBJECTS, OBJECTS, SCENES
from c3plus.configs.resolver import check_scene_assets
from c3plus.configs.catalog import configuration_snapshot
from c3plus.runtime.lcm import check_multicast
from c3plus.runtime.processes import check_binary

def collect_checks(runtime_only=False, require_binaries=False, check_scenes=False):
    """Collect runtime checks without printing or advancing simulation."""
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
    if not runtime_only:
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
    if require_binaries:
        for name in BINARIES:
            check(name, lambda name=name: check_binary(name))
    if check_scenes:
        for scene in SCENES:
            check("scene " + scene, lambda scene=scene: check_scene_assets(scene))
        for object_name in MESH_OBJECTS:
            check("object open_table/" + object_name,
                  lambda object_name=object_name: check_scene_assets("open_table", object_name))
    return checks


@configuration_snapshot()
def full_preflight(planned=(), output_root=None, require_runtime=True):
    """Check the entire selected matrix before the first native process starts."""
    from c3plus.configs.geometry import validate_suite_geometry
    from c3plus.configs.resolver import environment
    from c3plus.runtime.processes import check_experiment_capabilities

    errors = []
    if require_runtime:
        checks = collect_checks(runtime_only=True, require_binaries=True)
        errors.extend(f"{check['check']}: {check['detail']}" for check in checks if not check["passed"])
        if not errors:
            try:
                check_experiment_capabilities()
            except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as exc:
                errors.append(str(exc))
        lock_path = REPO / ".cache/sampling_c3_run.lock"
        if lock_path.exists():
            try:
                with lock_path.open("r") as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                errors.append("Another managed run or packaging job is active")
    # Every native asset and selected controller profile is checked, including
    # those that would otherwise be reached only near the end of a long suite.
    for task in SCENES:
        for obj in OBJECTS:
            try:
                check_scene_assets(task, obj, all_poses=False)
            except Exception as exc:
                errors.append(f"{task}/{obj}: {exc}")
    geometry = validate_suite_geometry()
    errors.extend(json.dumps(error, sort_keys=True) for error in geometry["errors"])
    if environment("exponential").get("SAMPLING_C3_SEED") != "42":
        errors.append("Canonical deterministic sampler seed is not 42")
    if output_root is not None:
        root = Path(output_root)
        parent = root
        while not parent.exists():
            parent = parent.parent
        if not parent.is_dir() or not os.access(parent, os.W_OK | os.X_OK):
            errors.append(f"Output directory is not writable: {parent}")
        for plan in planned:
            path = Path(plan["out"])
            if path != path.resolve() or not path.is_relative_to(root):
                errors.append(f"Run output has a symlink or leaves its campaign root: {path}")
    if require_runtime:
        for port in sorted({p["port"] for p in planned}):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.bind(("0.0.0.0", port))
            except OSError as exc:
                errors.append(f"LCM port {port} is unavailable: {exc}")
    if errors:
        raise RuntimeError("Full-suite preflight failed before launching any run:\n" + "\n".join(errors))
    return {"valid": True, "task_object_combinations": len(SCENES) * len(OBJECTS),
            "geometry": geometry, "seed": 42, "native_runtime_checked": require_runtime}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-only", action="store_true", help="Skip optional mesh-generation imports")
    parser.add_argument("--require-binaries", action="store_true", help="Check native targets and shared libraries")
    parser.add_argument("--check-scenes", action="store_true", help="Parse task and object assets")
    parser.add_argument("--suite", choices=["full"], help="Validate every task/object/pose combination")
    args = parser.parse_args(argv)
    checks = collect_checks(args.runtime_only, args.require_binaries, args.check_scenes and not args.suite)
    if args.suite:
        try:
            detail = full_preflight(require_runtime=args.require_binaries)
            checks.append({"check": "full suite", "passed": True, "detail": detail})
        except Exception as exc:
            checks.append({"check": "full suite", "passed": False, "detail": str(exc)})
    print(json.dumps({"python": sys.executable, "checks": checks}, indent=2))
    return int(any(not check["passed"] for check in checks))


if __name__ == "__main__":
    raise SystemExit(main())
