#!/usr/bin/env python3
"""Check build/runtime, optional mesh preparation, LCM, and all six scene assets."""
import argparse
import importlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid

REPO = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-only", action="store_true", help="Skip optional mesh-generation imports")
    parser.add_argument("--require-binaries", action="store_true", help="Check native targets and shared libraries")
    parser.add_argument("--check-scenes", action="store_true", help="Parse robot/tool/object/obstacle assets for all scenes")
    args = parser.parse_args()
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

    for name in ("bazel", "git", "g++", "pkg-config", "java", "setsid", "ffmpeg", "ffprobe", "dot"):
        check(name, lambda name=name: require_command(name))
    modules = ["pydrake", "numpy", "scipy", "matplotlib", "PIL", "trimesh", "yaml"]
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
            raise RuntimeError("Gurobi 10.0 shared library missing; set GUROBI_HOME")
        return "Gurobi 10.0 distribution found; license requirements depend on the solver selected"
    check("Gurobi distribution", gurobi_distribution)
    if args.require_binaries:
        for name in ("franka_sim", "franka_osc_controller", "franka_sampling_c3_controller"):
            def binary(name=name):
                path = REPO / "bazel-bin/examples/sampling_c3" / name
                if not os.access(path, os.X_OK):
                    raise RuntimeError(f"Build {name} inside this environment first")
                output = subprocess.run(["ldd", str(path)], capture_output=True, text=True, check=True).stdout
                if "not found" in output:
                    raise RuntimeError(output)
                return "executable and shared libraries found"
            check(name, binary)
    if args.check_scenes:
        from run_experiment import C, MODELS, SCENES
        for scene in SCENES:
            def scene_assets(scene=scene):
                from pydrake.multibody.parsing import Parser
                from pydrake.multibody.plant import MultibodyPlant
                plant = MultibodyPlant(0.001)
                model_parser = Parser(plant)
                model_parser.SetAutoRenaming(True)
                urdf = REPO / "examples/sampling_c3/urdf"
                models = ["oim_xarm6_tabletop/xarm6/xarm6_policyport.xml",
                          "end_effector_xarm6_stick.urdf", "ground_oim_xarm6.urdf",
                          *[p for p in MODELS[scene] if p]]
                for path in models:
                    model_parser.AddModels(str(urdf / path))
                referenced_models = set()
                import yaml
                def inspect_refs(value):
                    if isinstance(value, dict):
                        for item in value.values(): inspect_refs(item)
                    elif isinstance(value, list):
                        for item in value: inspect_refs(item)
                    elif isinstance(value, str) and value.startswith(("examples/", "common/")):
                        path = REPO / value
                        if path.suffix in (".yaml", ".sdf", ".urdf", ".xml"):
                            if not path.is_file():
                                raise RuntimeError(f"Missing referenced dependency: {value}")
                            if path.suffix in (".sdf", ".urdf", ".xml"):
                                referenced_models.add(path)
                for m in range(1, 6):
                    for n in range(1, 6):
                        demo = REPO / "examples/sampling_c3" / C.demo_name(scene, m, n)
                        if not (demo / "parameters/sim_params.yaml").is_file():
                            raise RuntimeError(f"Missing demo config: {demo}")
                        for config in (demo / "parameters").glob("*.yaml"):
                            inspect_refs(yaml.safe_load(config.read_text()))
                        C.goal_of(scene, n)
                already_parsed = {urdf / path for path in models}
                for path in sorted(referenced_models - already_parsed):
                    model_parser.AddModels(str(path))
                return "robot/tool/simulator/controller models parsed; 25 demo configs and referenced dependencies found"
            check("scene " + scene, scene_assets)
    print(json.dumps({"python": sys.executable, "checks": checks}, indent=2))
    return int(any(not c["passed"] for c in checks))


if __name__ == "__main__":
    sys.exit(main())
