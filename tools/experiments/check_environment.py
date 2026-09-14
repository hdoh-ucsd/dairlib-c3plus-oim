#!/usr/bin/env python3
"""Check build/runtime, optional mesh preparation, LCM, and scene/object assets."""
import argparse
import hashlib
import importlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid

if __package__:
    from .catalog import (BINARIES, CONFIG_DIR, EXPERIMENTS_FILE, MESH_OBJECTS, MODELS, REPO, SCENES,
                          compose_demo_configs, demo_name, load_controller_goal, load_demo_configs,
                          model_assets, planner_environment, resolve_object_profile)
else:
    from catalog import (BINARIES, CONFIG_DIR, EXPERIMENTS_FILE, MESH_OBJECTS, MODELS, REPO, SCENES,
                         compose_demo_configs, demo_name, load_controller_goal, load_demo_configs,
                         model_assets, planner_environment, resolve_object_profile)


def check_binary(name, repo=REPO):
    """Inspect linkage and flags without constructing native simulation systems."""
    path = Path(repo) / ".build/bin/examples/sampling_c3" / name
    rebuild = "Rebuild inside Docker with python3 -m tools.experiments build."
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


def check_scene_assets(scene, object_name=None, repo=REPO):
    """Resolve selected configurations and parse assets; never advance physics."""
    import yaml
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant

    repo = Path(repo).resolve()
    profile = resolve_object_profile(scene, object_name, repo=repo)
    config = yaml.safe_load((repo / CONFIG_DIR.relative_to(REPO) / f"{scene}.yaml").read_text())
    planner_environment(config)
    selected = compose_demo_configs(demo_name(scene, 1, 1, object_name), repo=repo, object_name=object_name)
    # Legacy primitive SDFs use Drake extensions without XML namespace
    # declarations. Drake accepts those files; the mesh snapshot helper's
    # stricter XML parser is intended for the imported mesh profiles only.
    assets = (set(model_assets(selected, repo=repo)) if profile["object_name"] in MESH_OBJECTS else
              {(repo / value).resolve() for role in ("controller", "simulation")
               for value in selected[role]["object_models"]})
    urdf = repo / "examples/sampling_c3/urdf"
    models = {urdf / "oim_xarm6_tabletop/xarm6/xarm6_policyport.xml",
              urdf / "end_effector_xarm6_stick.urdf", urdf / "ground_oim_xarm6.urdf"}
    models.update(path for path in assets if path.suffix in (".sdf", ".urdf", ".xml"))
    if MODELS[scene][1]:
        models.add(urdf / MODELS[scene][1])

    def inspect_refs(value):
        if isinstance(value, dict):
            for item in value.values():
                inspect_refs(item)
        elif isinstance(value, list):
            for item in value:
                inspect_refs(item)
        elif isinstance(value, str) and Path(value).suffix in (".yaml", ".yml", ".sdf", ".urdf", ".xml", ".obj", ".json"):
            if Path(value).is_absolute():
                raise RuntimeError(f"Source configuration must use checkout-relative asset paths: {value}")
            if not value.startswith(("examples/", "common/")):
                return
            path = (repo / value).resolve()
            if not path.is_relative_to(repo) or not path.is_file():
                raise RuntimeError(f"Missing or nonportable referenced dependency: {value}")
            if path.suffix in (".sdf", ".urdf", ".xml"):
                models.add(path)

    if profile["object_name"] in MESH_OBJECTS:
        metadata_path = repo / profile["physics_metadata_file"]
        physics = profile["physics"]
        for key in ("simulation_model", "controller_model"):
            if (metadata_path.parent / physics[key]).resolve() != (repo / profile[key]).resolve():
                raise RuntimeError(f"Physics metadata {key} disagrees with {profile['object_name']} profile")
        if physics["body_name"] != profile["object_body_name"]:
            raise RuntimeError(f"Physics metadata body name disagrees with {profile['object_name']} profile")
        pieces = physics["convex_pieces"]
        if len(pieces) != physics["collision_piece_count"]:
            raise RuntimeError(f"Physics metadata collision-piece count is inconsistent: {metadata_path}")
        expected = [(physics["source_mesh"], physics["source_sha256"])]
        expected.extend((piece["file"], piece["sha256"]) for piece in pieces)
        for relative, digest in expected:
            path = (metadata_path.parent / relative).resolve()
            if path not in assets:
                raise RuntimeError(f"Physics metadata asset is not selected by the native models: {path}")
            if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise RuntimeError(f"Physics metadata hash differs from the checked-in asset: {path}")

    for start in range(1, 6):
        for goal in range(1, 6):
            name = demo_name(scene, start, goal, object_name)
            inspect_refs(compose_demo_configs(name, repo=repo, object_name=object_name))
            for path, saved_config in load_demo_configs(name, repo=repo, object_name=object_name).items():
                if path != repo / EXPERIMENTS_FILE:
                    inspect_refs(saved_config)
            load_controller_goal(name, repo=repo, object_name=object_name)
    plant = MultibodyPlant(0.001)
    model_parser = Parser(plant)
    model_parser.SetAutoRenaming(True)
    for path in sorted(models):
        if not path.is_file():
            raise RuntimeError(f"Missing model: {path}")
        model_parser.AddModels(str(path))
    return (f"{profile['object_name']}: 25 start/goal configurations, selected dependencies, "
            "robot/tool/object/obstacle models and available mesh hashes verified; no simulation advanced")


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

    def multicast():
        from pydrake.lcm import DrakeLcm
        lc = DrakeLcm(f"udpm://239.255.76.67:{42000 + os.getpid() % 20000}?ttl=0")
        channel = "DAIRLIB_DEPENDENCY_CHECK_" + uuid.uuid4().hex
        received = []
        lc.Subscribe(channel, lambda data: received.append(data))
        deadline = time.monotonic() + 2
        while not received and time.monotonic() < deadline:
            lc.Publish(channel, b"dependency-check")
            lc.HandleSubscriptions(timeout_millis=100)
        if not received:
            raise RuntimeError("LCM multicast loopback failed; check the container multicast route")
        return "publish/subscribe round trip passed"
    check("LCM multicast", multicast)

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
    sys.exit(main())
