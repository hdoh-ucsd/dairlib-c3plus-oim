"""Launcher contracts tested without a Docker daemon or expensive image build."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

DOCKER_DIR = Path(__file__).resolve().parents[1]


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.context = self.root / "docker"
        self.context.mkdir()
        for name in ("shell.sh", "entrypoint.sh", "Dockerfile", "requirements.txt", ".dockerignore"):
            shutil.copy2(DOCKER_DIR / name, self.context / name)
        for name in ("snopt7.6.tar.gz", "gurobi10.0.3_linux64.tar.gz"):
            (self.context / name).write_bytes(b"archive input")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.log = self.root / "calls.jsonl"
        self.env = {key: value for key, value in os.environ.items()
                    if not key.startswith(("DAIRLIB_", "MOCK_")) and key != "MESHCAT_PORT"}
        self.env.update(PATH=f"{self.bin}:{os.environ['PATH']}", MOCK_LOG=str(self.log),
                        MOCK_UID="1234", MOCK_GID="2345")
        self.executable("id", '#!/bin/bash\nif [[ "$1" == -u ]]; then echo "$MOCK_UID"; else echo "$MOCK_GID"; fi\n')
        self.executable("docker", f"#!{sys.executable}\n" + '''
import json, os, sys
args = sys.argv[1:]
with open(os.environ["MOCK_LOG"], "a") as stream:
    stream.write(json.dumps(args) + "\\n")
if args[0] == "info":
    print(os.environ.get("MOCK_PLATFORM", "linux/x86_64"), os.environ.get("MOCK_CPUS", "32"))
    sys.exit(int(os.environ.get("MOCK_DAEMON_EXIT", "0")))
if args[:2] == ["image", "inspect"]:
    if "--format" in args:
        print("sha256:test", os.environ.get("MOCK_IMAGE_UID", os.environ["MOCK_UID"]),
              os.environ.get("MOCK_IMAGE_GID", os.environ["MOCK_GID"]))
    else:
        sys.exit(0 if os.environ.get("MOCK_IMAGE_EXISTS", "1") == "1" else 1)
if args[0] == "run":
    sys.exit(int(os.environ.get("MOCK_RUN_EXIT", "0")))
''')

    def executable(self, name, contents):
        path = self.bin / name
        path.write_text(contents)
        path.chmod(0o755)

    def launch(self, *args, **env):
        self.env.update(env)
        return subprocess.run(["bash", str(self.context / "shell.sh"), *args],
                              env=self.env, capture_output=True, text=True)

    def calls(self, command=None):
        calls = [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []
        return [call for call in calls if command is None or call[0] == command]

    def test_help_never_contacts_docker(self):
        result = self.launch("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--build-only", result.stdout)
        self.assertEqual(self.calls(), [])

    def test_command_arguments_preserved_and_noninteractive(self):
        command = ["python3", "-c", "print('hello; $HOME')", "a b", ""]
        result = self.launch("--", *command)
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self.calls("run")[0]
        self.assertEqual(run[-len(command):], command)
        self.assertIn("-i", run)
        self.assertNotIn("-t", run)
        self.assertIn("DAIRLIB_BAZEL_JOBS=8", run)
        self.assertIn("C3PLUS_CONTAINER_IMAGE_ID=sha256:test", run)
        self.assertIn("dairlib-c3plus-oim-bazel-cache-u1234-g2345:/home/dairlib/.cache/bazel", run)

    def test_build_only_never_runs_a_container(self):
        result = self.launch("--build-only")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.calls("build")), 1)
        self.assertEqual(self.calls("run"), [])
        self.assertIn("USER_UID=1234", self.calls("build")[0])

    def test_default_image_is_built_if_missing(self):
        result = self.launch("true", MOCK_IMAGE_EXISTS="0")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.calls("build")), 1)
        self.assertEqual(self.calls("pull"), [])

    def test_explicit_missing_image_is_pulled(self):
        result = self.launch("true", DAIRLIB_IMAGE="example/toolchain:pin", MOCK_IMAGE_EXISTS="0")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("pull"), [["pull", "example/toolchain:pin"]])
        self.assertEqual(self.calls("build"), [])

    def test_force_build_explicit_tag_never_pulls(self):
        result = self.launch("--build", "true", DAIRLIB_IMAGE="example/toolchain:pin")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.calls("build")), 1)
        self.assertEqual(self.calls("pull"), [])

    def test_root_checkout_runs_as_root_without_changing_image_user(self):
        result = self.launch("--build", "true", MOCK_UID="0", MOCK_GID="0")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("USER_UID=1000", self.calls("build")[0])
        run = self.calls("run")[0]
        self.assertEqual(run[run.index("--user") + 1], "0:0")
        self.assertIn("dairlib-c3plus-oim-bazel-cache-u0-g0:/home/dairlib/.cache/bazel", run)

    def test_nonroot_mismatched_image_is_rejected(self):
        result = self.launch("true", MOCK_IMAGE_UID="1000")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("matching image", result.stderr)
        self.assertEqual(self.calls("run"), [])

    def test_failure_code_is_preserved_and_container_cleaned(self):
        result = self.launch("false", MOCK_RUN_EXIT="7")
        self.assertEqual(result.returncode, 7)
        self.assertEqual(len(self.calls("rm")), 1)
        run = self.calls("run")[0]
        self.assertEqual(self.calls("rm")[0][-1], run[run.index("--name") + 1])

    def test_memory_limit_reduces_default_build_parallelism(self):
        result = self.launch("true", DAIRLIB_MEM="8g", DAIRLIB_CPUS="4", MESHCAT_PORT="7012")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self.calls("run")[0]
        self.assertIn("DAIRLIB_BAZEL_JOBS=2", run)
        self.assertIn("DAIRLIB_BAZEL_RAM_MB=5120", run)
        self.assertIn("127.0.0.1:7012:7000", run)

    def test_default_cpu_quota_respects_small_docker_daemon(self):
        result = self.launch("true", MOCK_CPUS="2")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self.calls("run")[0]
        self.assertEqual(run[run.index("--cpus") + 1], "2")
        self.assertIn("DAIRLIB_BAZEL_JOBS=2", run)

    def test_invalid_runtime_limits_are_rejected(self):
        for overrides in ({"DAIRLIB_MEM": "2g"}, {"DAIRLIB_CPUS": "0"},
                          {"DAIRLIB_BAZEL_JOBS": "0"}, {"MESHCAT_PORT": "65536"}):
            with self.subTest(overrides=overrides):
                original = self.env.copy()
                result = self.launch("true", **overrides)
                self.env = original
                self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls("run"), [])

    def test_changed_docker_inputs_change_default_image(self):
        self.assertEqual(self.launch("true").returncode, 0)
        with (self.context / "requirements.txt").open("a") as stream:
            stream.write("\n# changed dependency input\n")
        self.assertEqual(self.launch("true").returncode, 0)
        first, second = self.calls("run")
        self.assertNotEqual(first[-2], second[-2])

    def test_missing_archive_has_actionable_error(self):
        (self.context / "snopt7.6.tar.gz").unlink()
        result = self.launch("--build-only")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Missing docker/snopt7.6.tar.gz", result.stderr)
        self.assertEqual(self.calls("build"), [])

    def test_daemon_failure_does_not_attempt_pull_or_build(self):
        result = self.launch("true", MOCK_DAEMON_EXIT="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("socket permissions", result.stderr)
        self.assertEqual(len(self.calls()), 1)

    def test_wrong_platform_is_rejected(self):
        result = self.launch("true", MOCK_PLATFORM="linux/aarch64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("x86-64", result.stderr)

    def test_entrypoint_writes_runtime_bazel_settings_and_forwards_status(self):
        self.executable("ip", "#!/bin/bash\nexit 0\n")
        self.env.update(HOME=str(self.root), MOCK_UID="0", DAIRLIB_BAZEL_JOBS="3", DAIRLIB_BAZEL_RAM_MB="6000")
        result = subprocess.run(["bash", str(self.context / "entrypoint.sh"), "bash", "-c", "exit 9"], env=self.env)
        self.assertEqual(result.returncode, 9)
        settings = (self.root / ".bazelrc").read_text()
        self.assertIn("build --jobs=3", settings)
        self.assertIn("build --local_ram_resources=6000", settings)

    def test_entrypoint_does_not_hide_network_setup_failure(self):
        self.executable("ip", "#!/bin/bash\nexit 4\n")
        self.env.update(HOME=str(self.root), MOCK_UID="0")
        result = subprocess.run(["bash", str(self.context / "entrypoint.sh"), "true"], env=self.env)
        self.assertEqual(result.returncode, 4)
        self.assertFalse((self.root / ".bazelrc").exists())


if __name__ == "__main__":
    unittest.main()
