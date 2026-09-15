from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from c3plus import configs as S
from c3plus.runtime import environment as E
from c3plus.configs import resolver as Resolver

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_environment_checks_current_native_flags_without_running_systems(self):
        required = {
            "franka_sim": ["controller_params", "execution_logging", "execution_step_budget", "execution_stop_file"],
            "franka_osc_controller": ["controller_params", "execution_logging"],
            "franka_sampling_c3_controller": ["controller_params", "goal_yaw_degrees"],
        }
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            for name, flags in required.items():
                executable = repo / ".build/bin/examples/sampling_c3" / name
                executable.parent.mkdir(parents=True, exist_ok=True)
                executable.write_text("Must never execute this fixture")
                executable.chmod(0o755)
                for omitted in (None, *flags):
                    help_text = "\n".join(f" -{flag} (flag)" for flag in flags if flag != omitted)
                    # Gflags --helpshort can exit nonzero; emitted flags are authoritative.
                    responses = [subprocess.CompletedProcess([], 0, "linked libraries", ""),
                                 subprocess.CompletedProcess([], 1, help_text, "")]
                    with self.subTest(binary=name, omitted=omitted), \
                            patch.object(E.subprocess, "run", side_effect=responses) as run:
                        if omitted is None:
                            self.assertIn("verified", E.check_binary(name, repo=repo))
                        else:
                            with self.assertRaisesRegex(RuntimeError, "--" + omitted):
                                E.check_binary(name, repo=repo)
                        self.assertEqual([call.args[0] for call in run.call_args_list],
                                         [["ldd", str(executable)], [str(executable), "--helpshort"]])
            broken = subprocess.CompletedProcess([], 0, "libgurobi100.so => not found", "")
            with patch.object(E.subprocess, "run", return_value=broken) as run, \
                    self.assertRaisesRegex(RuntimeError, "libgurobi100.so"):
                E.check_binary("franka_sim", repo=repo)
            self.assertEqual(run.call_count, 1)


    def test_environment_checks_selected_mesh_metadata_and_dependencies(self):
        parsed = []
        parser_module = types.ModuleType("pydrake.multibody.parsing")
        plant_module = types.ModuleType("pydrake.multibody.plant")
        class Parser:
            def __init__(self, plant):
                pass
            def SetAutoRenaming(self, value):
                pass
            def AddModels(self, path):
                parsed.append(Path(path))
        parser_module.Parser = Parser
        plant_module.MultibodyPlant = lambda timestep: object()
        demo = S.demo_name("open_task", 1, 1, "sugar_box")
        profile = S.resolve_object_profile("open_task", "sugar_box")
        composed = S.compose_demo_configs(demo, object_name="sugar_box")
        paths = set(S.load_demo_configs(demo, object_name="sugar_box"))
        paths.update(S.model_assets(composed))
        paths.update(S.REPO / name for name in (
            profile["physics_metadata_file"], str(S.CONFIG_DIR.relative_to(S.REPO) / "open_task.yaml"),
            "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6/xarm6_policyport.xml",
            "examples/sampling_c3/urdf/end_effector_xarm6_stick.urdf",
            "examples/sampling_c3/urdf/ee_visualization_model.urdf",
            "examples/sampling_c3/urdf/push_t.sdf",
            "examples/sampling_c3/urdf/ground_oim_xarm6.urdf"))
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, {
                "pydrake.multibody.parsing": parser_module, "pydrake.multibody.plant": plant_module}):
            repo = Path(tmp)
            for path in paths:
                target = repo / path.relative_to(S.REPO)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
            detail = E.check_scene_assets("open_task", "sugar_box", repo=repo)
            self.assertIn("25 start/goal", detail)
            self.assertIn(repo / profile["simulation_model"], parsed)
            self.assertIn(repo / profile["controller_model"], parsed)
            # Each failure concerns selected inputs, not unselected catalogue entries.
            for relative in (profile["physics_metadata_file"], profile["sampling_params_file"],
                             composed["controller"]["sampling_mesh_files"][0]):
                path = repo / relative
                contents = path.read_bytes()
                path.unlink()
                with self.subTest(missing=relative), self.assertRaises((FileNotFoundError, RuntimeError)):
                    E.check_scene_assets("open_task", "sugar_box", repo=repo)
                path.write_bytes(contents)
            mesh = repo / composed["controller"]["sampling_mesh_files"][0]
            mesh.write_bytes(mesh.read_bytes() + b"\n# changed after metadata generation\n")
            with self.assertRaisesRegex(RuntimeError, "hash differs"):
                E.check_scene_assets("open_task", "sugar_box", repo=repo)


    def test_environment_check_failures_are_structured_and_include_runtime_submodules(self):
        imported = []
        def import_module(name):
            imported.append(name)
            if name == "mpl_toolkits.mplot3d":
                raise ImportError("broken Matplotlib namespace")
            return types.SimpleNamespace(__name__=name)
        fake_lcm = types.ModuleType("pydrake.lcm")
        fake_lcm.DrakeLcm = lambda url: (_ for _ in ()).throw(RuntimeError("test multicast unavailable"))
        output = io.StringIO()
        with patch.dict(sys.modules, {"pydrake.lcm": fake_lcm}), \
                patch.object(E.importlib, "import_module", side_effect=import_module), \
                patch.object(E.shutil, "which", side_effect=lambda name: "/tools/" + name), \
                patch.object(E, "version", return_value="1.51.1"), \
                patch.object(E.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "<svg>", "")), \
                patch.object(E, "check_binary", side_effect=RuntimeError("missing --execution_logging")), \
                patch.object(E, "check_scene_assets", return_value="parsed only") as scenes, redirect_stdout(output):
            self.assertEqual(E.main(["--runtime-only", "--require-binaries", "--check-scenes"]), 1)
        report = json.loads(output.getvalue())
        checks = {entry["check"]: entry for entry in report["checks"]}
        self.assertFalse(checks["import mpl_toolkits.mplot3d"]["passed"])
        self.assertIn("broken Matplotlib namespace", checks["import mpl_toolkits.mplot3d"]["detail"])
        self.assertIn("--execution_logging", checks["franka_sim"]["detail"])
        self.assertIn("pydrake.systems.sensors", imported)
        self.assertNotIn("vhacdx", imported)
        self.assertEqual(scenes.call_count, 10)
        for name in S.MESH_OBJECTS:
            self.assertIn("object open_table/" + name, checks)


    def test_environment_legacy_drake_extensions_use_native_model_parser(self):
        parser_module = types.ModuleType("pydrake.multibody.parsing")
        plant_module = types.ModuleType("pydrake.multibody.plant")
        parsed = []
        parser = types.SimpleNamespace(SetAutoRenaming=lambda value: None,
                                       AddModels=lambda path: parsed.append(Path(path)))
        parser_module.Parser = lambda plant: parser
        plant_module.MultibodyPlant = lambda timestep: object()
        with patch.dict(sys.modules, {"pydrake.multibody.parsing": parser_module,
                                      "pydrake.multibody.plant": plant_module}), \
                patch.object(Resolver, "model_assets", side_effect=AssertionError("strict mesh XML parser")):
            self.assertIn("T_shape", E.check_scene_assets("icra_sign"))
        self.assertIn(S.REPO / "examples/sampling_c3/urdf/push_t_oimscale_m01.sdf", parsed)
