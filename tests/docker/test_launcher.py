"""Launcher contracts tested without a Docker daemon or expensive image build."""
import errno
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest

DOCKER_DIR = Path(__file__).resolve().parents[2] / "docker"


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="dairlib checkout ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.context = self.root / "docker"
        self.context.mkdir()
        for marker in ("MODULE.bazel", ".bazeliskrc", "tools/__main__.py"):
            path = self.root / marker
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("checkout marker\n")
        for name in ("shell.sh", "entrypoint.sh", "Dockerfile", "requirements.txt", ".dockerignore"):
            shutil.copy2(DOCKER_DIR / name, self.context / name)
        # Substitute only the container's absolute cache mount in our copied
        # entrypoint, so its real filesystem checks stay inside the fixture.
        self.cache = self.root / ".cache" / "bazel"
        entrypoint = self.context / "entrypoint.sh"
        entrypoint.write_text(entrypoint.read_text().replace(
            "/home/dairlib/.cache/bazel", shlex.quote(str(self.cache))))
        for name in ("snopt7.6.tar.gz", "gurobi10.0.3_linux64.tar.gz"):
            (self.context / name).write_bytes(b"archive input")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.log = self.root / "calls.jsonl"
        self.env = {key: value for key, value in os.environ.items()
                    if not key.startswith(("DAIRLIB_", "MOCK_", "DOCKER_")) and key != "MESHCAT_PORT"}
        self.env.update(PATH=f"{self.bin}:{os.environ['PATH']}", MOCK_LOG=str(self.log),
                        MOCK_UID="1234", MOCK_GID="2345")
        self.executable("id", '#!/bin/bash\nif [[ "$1" == -u ]]; then echo "$MOCK_UID"; else echo "$MOCK_GID"; fi\n')
        self.executable("sudo", '#!/bin/bash\nexec "$@"\n')
        self.executable("docker", f"#!{sys.executable}\n" + '''
import json, os, sys
args = sys.argv[1:]
with open(os.environ["MOCK_LOG"], "a") as stream:
    stream.write(json.dumps(args) + "\\n")
if args[0] == "info":
    print(os.environ.get("MOCK_PLATFORM", "linux/x86_64"), os.environ.get("MOCK_CPUS", "32"),
          os.environ.get("MOCK_MEM", str(64 * 1024**3)))
    sys.exit(int(os.environ.get("MOCK_DAEMON_EXIT", "0")))
if args[:2] == ["context", "inspect"]:
    print(os.environ.get("MOCK_ENDPOINT", "unix:///var/run/docker.sock"))
if args[:2] == ["buildx", "version"]:
    sys.exit(int(os.environ.get("MOCK_BUILDX_EXIT", "0")))
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

    def test_missing_docker_has_actionable_error(self):
        (self.bin / "docker").unlink()
        for name in ("bash", "dirname"):
            (self.bin / name).symlink_to(shutil.which(name))
        result = self.launch("true", PATH=str(self.bin))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Docker CLI required", result.stderr)

    def test_incomplete_checkout_fails_before_contacting_daemon(self):
        (self.root / "MODULE.bazel").unlink()
        result = self.launch("true")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("complete repository", result.stderr)
        self.assertEqual(self.calls(), [])

    def test_unwritable_checkout_fails_before_contacting_daemon(self):
        self.root.chmod(0o555)
        self.addCleanup(self.root.chmod, 0o755)
        # Root can write mode-555 directories; model an ordinary host user.
        kwargs = {"user": 65534} if os.geteuid() == 0 else {}
        try:
            result = subprocess.run(["/bin/bash", str(self.context / "shell.sh"), "true"],
                                    env=self.env, capture_output=True, text=True, **kwargs)
        except (PermissionError, OSError) as exc:
            if os.geteuid() == 0 and exc.errno in (errno.EINVAL, errno.EPERM):
                self.skipTest("User namespace cannot assign the fixture user's UID")
            raise
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("must be writable", result.stderr)
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
        self.assertIn(f"{self.root}:/home/dairlib/dairlib:rw", run)

    def test_existing_image_does_not_require_buildx(self):
        result = self.launch("true", MOCK_BUILDX_EXIT="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("buildx"), [])

    def test_missing_buildx_fails_before_build(self):
        result = self.launch("--build-only", MOCK_BUILDX_EXIT="1")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Docker Buildx is required", result.stderr)
        self.assertEqual(self.calls("build"), [])

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

    def test_default_memory_uses_three_quarters_of_daemon_memory(self):
        result = self.launch("true", MOCK_MEM=str(8 * 1024**3))
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self.calls("run")[0]
        self.assertEqual(run[run.index("--memory") + 1], "6144m")
        self.assertIn("DAIRLIB_BAZEL_JOBS=1", run)
        self.assertIn("DAIRLIB_BAZEL_RAM_MB=3072", run)

    def test_default_memory_is_capped_at_24_gib(self):
        result = self.launch("true")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self.calls("run")[0]
        self.assertEqual(run[run.index("--memory") + 1], "24576m")

    def test_too_little_daemon_memory_has_actionable_error(self):
        result = self.launch("--build-only", MOCK_MEM=str(4 * 1024**3))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Increase Docker Desktop memory", result.stderr)
        self.assertEqual(self.calls("build"), [])

    def test_invalid_runtime_limits_are_rejected(self):
        for overrides in ({"DAIRLIB_MEM": "2g"}, {"DAIRLIB_CPUS": "0"},
                          {"DAIRLIB_BAZEL_JOBS": "0"}, {"MESHCAT_PORT": "65536"},
                          {"DAIRLIB_CPUS": "32.1"}, {"DAIRLIB_MEM": "65g"},
                          {"DAIRLIB_BAZEL_RAM_MB": "24576"},
                          {"DAIRLIB_MEM": "999999999999999g"},
                          {"DAIRLIB_CACHE_VOLUME": "/some/host/path"}):
            with self.subTest(overrides=overrides):
                original = self.env.copy()
                result = self.launch("--build", "true", **overrides)
                self.env = original
                self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls("run"), [])
        self.assertEqual(self.calls("build"), [])

    def test_fractional_cpu_quota_remains_valid(self):
        result = self.launch("true", DAIRLIB_CPUS="0.5")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DAIRLIB_BAZEL_JOBS=1", self.calls("run")[0])

    def test_macos_shasum_fallback_produces_same_image_tag(self):
        if not shutil.which("shasum"):
            self.skipTest("shasum is not installed on this test host")
        self.assertEqual(self.launch("true").returncode, 0)
        for name in ("bash", "dirname", "shasum"):
            (self.bin / name).symlink_to(shutil.which(name))
        result = self.launch("true", PATH=str(self.bin))
        self.assertEqual(result.returncode, 0, result.stderr)
        first, second = self.calls("run")
        self.assertEqual(first[-2], second[-2])

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
        for platform in ("linux/aarch64", "windows/amd64"):
            with self.subTest(platform=platform):
                result = self.launch("true", MOCK_PLATFORM=platform)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("x86-64", result.stderr)

    def test_remote_context_is_rejected_before_image_work(self):
        result = self.launch("--build-only", MOCK_ENDPOINT="ssh://user@remote")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("local Docker Unix socket", result.stderr)
        self.assertEqual(self.calls("build"), [])

    def test_remote_docker_host_is_rejected(self):
        result = self.launch("true", DOCKER_HOST="tcp://remote:2376")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("local Docker Unix socket", result.stderr)

    def test_explicit_local_context_takes_precedence_over_remote_host(self):
        result = self.launch("true", DOCKER_HOST="tcp://remote:2376", DOCKER_CONTEXT="desktop-linux")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("desktop-linux", self.calls("context")[0])

    def test_entrypoint_writes_runtime_bazel_settings_and_forwards_status(self):
        self.executable("ip", "#!/bin/bash\nexit 0\n")
        self.env.update(HOME=str(self.root), MOCK_UID=str(os.getuid()), MOCK_GID=str(os.getgid()),
                        DAIRLIB_BAZEL_JOBS="3", DAIRLIB_BAZEL_RAM_MB="6000")
        result = subprocess.run(["bash", str(self.context / "entrypoint.sh"), "bash", "-c", "exit 9"], env=self.env)
        self.assertEqual(result.returncode, 9)
        settings = (self.root / ".bazelrc").read_text()
        self.assertIn(f"startup --output_user_root={self.cache}", settings)
        self.assertEqual(self.cache.stat().st_uid, os.getuid())
        self.assertIn("build --jobs=3", settings)
        self.assertIn("build --local_resources=memory=6000", settings)

    @unittest.skipUnless(os.geteuid() == 0, "Requires root to model Docker's image-owned volume")
    def test_root_entrypoint_repairs_only_image_owned_cache_root(self):
        self.executable("ip", "#!/bin/bash\nexit 0\n")
        self.env.update(HOME=str(self.root), MOCK_UID="0", MOCK_GID="0")
        self.cache.mkdir(parents=True)
        self.cache.chmod(0o755)
        cached_file = self.cache / "existing-cache-entry"
        cached_file.write_text("preserve cached build")
        cached_file.chmod(0o640)
        try:
            os.chown(self.cache, 1000, 1000)
        except OSError as exc:
            if exc.errno in (errno.EINVAL, errno.EPERM):
                self.skipTest("User namespace cannot assign the image user's UID/GID")
            raise
        os.chown(cached_file, 1000, 1000)
        result = subprocess.run(["bash", str(self.context / "entrypoint.sh"), "true"],
                                env=self.env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.cache.stat().st_uid, self.cache.stat().st_gid), (0, 0))
        self.assertEqual(self.cache.stat().st_mode & 0o777, 0o755)
        self.assertEqual((cached_file.stat().st_uid, cached_file.stat().st_gid), (1000, 1000))
        self.assertEqual(cached_file.stat().st_mode & 0o777, 0o640)
        self.assertEqual(cached_file.read_text(), "preserve cached build")

    def test_nonroot_entrypoint_rejects_cache_owned_by_another_user(self):
        self.executable("ip", "#!/bin/bash\nexit 0\n")
        self.cache.mkdir(parents=True)
        owner = self.cache.stat().st_uid
        runtime_uid = str(owner + 1)
        self.env.update(HOME=str(self.root), MOCK_UID=runtime_uid)
        marker = self.root / "command-ran"
        result = subprocess.run(["bash", str(self.context / "entrypoint.sh"), "touch", str(marker)],
                                env=self.env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn(f"belongs to UID {owner}; running as UID {runtime_uid}", result.stderr)
        self.assertIn("DAIRLIB_CACHE_VOLUME", result.stderr)
        self.assertEqual(self.cache.stat().st_uid, owner)
        self.assertFalse(marker.exists())
        self.assertFalse((self.root / ".bazelrc").exists())

    def test_entrypoint_does_not_hide_network_setup_failure(self):
        self.executable("ip", "#!/bin/bash\nexit 4\n")
        self.env.update(HOME=str(self.root), MOCK_UID="0")
        result = subprocess.run(["bash", str(self.context / "entrypoint.sh"), "true"], env=self.env)
        self.assertEqual(result.returncode, 4)
        self.assertFalse((self.root / ".bazelrc").exists())


if __name__ == "__main__":
    unittest.main()
