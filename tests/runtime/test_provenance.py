import base64
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from c3plus.runtime import provenance as Provenance

from tests.fixtures.results import WorkflowFixtures

class WorkflowTests(WorkflowFixtures, unittest.TestCase):
    def test_source_capture_reproduces_dirty_tree_without_changing_index(self):
        def decode(entry):
            raw = (entry["content"].encode("utf-8") if entry["encoding"] == "utf-8"
                   else base64.b64decode(entry["content"], validate=True))
            self.assertEqual(len(raw), entry["size_bytes"])
            self.assertEqual(hashlib.sha256(raw).hexdigest(), entry["sha256"])
            return raw

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            repo = root / "source"
            repo.mkdir()

            def git(*args):
                return subprocess.check_output(["git", "--no-optional-locks", *args], cwd=repo,
                                               stderr=subprocess.PIPE)

            git("init", "-q")
            git("config", "core.fileMode", "true")
            (repo / ".gitignore").write_text("generated/\n")
            for name in ("staged.txt", "unstaged.txt", "both.txt", "removed.txt", "renamed.txt", "script.sh"):
                (repo / name).write_text("first\nsecond\n")
            (repo / "binary.dat").write_bytes(b"\0before\xff")
            git("add", ".")
            git("-c", "user.name=Test", "-c", "user.email=test@example.invalid",
                "-c", "commit.gpgsign=false", "commit", "-qm", "base")
            clean = Provenance.capture_source_state(repo)
            self.assertFalse(clean["worktree_dirty"])
            self.assertEqual(decode(clean["tracked_patch"]), b"")
            self.assertEqual(clean["untracked_files"], {})

            (repo / "staged.txt").write_text("staged edit\n")
            (repo / "both.txt").write_text("staged first\nsecond\n")
            (repo / "new_tracked.txt").write_text("staged new file\n")
            git("add", "staged.txt", "both.txt", "new_tracked.txt")
            git("mv", "renamed.txt", "new_name.txt")
            (repo / "both.txt").write_text("staged first\nunstaged second\n")
            (repo / "unstaged.txt").write_text("unstaged edit\n")
            (repo / "new_tracked.txt").write_text("staged new file\nthen unstaged edit\n")
            (repo / "removed.txt").unlink()
            (repo / "script.sh").chmod(0o755)
            (repo / "binary.dat").write_bytes(b"\0after\x80\xfe")
            (repo / "new directory").mkdir()
            (repo / "new directory/module\nname.py").write_text("print('π')\n")
            (repo / "new directory/data.bin").write_bytes(b"\xff\x80\0")
            (repo / "new directory/executable").write_text("#!/bin/sh\nexit 0\n")
            (repo / "new directory/executable").chmod(0o750)
            outside = root / "outside.txt"
            outside.write_text("do not follow this symlink")
            (repo / "outside-link").symlink_to(outside)
            (repo / "generated").mkdir()
            (repo / "generated/ignored.txt").write_text("not part of source capture")
            before_index = (repo / ".git/index").read_bytes()
            before_status = git("status", "--porcelain=v1", "--untracked-files=all", "-z")

            saved = Provenance.capture_source_state(repo)
            self.assertEqual((repo / ".git/index").read_bytes(), before_index)
            self.assertEqual(saved["base_commit"], clean["base_commit"])
            self.assertTrue(saved["worktree_dirty"])
            self.assertEqual(decode(saved["git_status"]), before_status)
            source_patch = decode(saved["tracked_patch"])
            self.assertIn(b"GIT binary patch", source_patch)
            self.assertEqual(set(saved["untracked_files"]), {
                "new directory/module\nname.py", "new directory/data.bin",
                "new directory/executable", "outside-link"})
            self.assertEqual(saved["untracked_files"]["new directory/data.bin"]["encoding"], "base64")
            self.assertEqual(decode(saved["untracked_files"]["outside-link"]), os.fsencode(outside))

            restored = root / "restored"
            subprocess.run(["git", "clone", "-q", "--no-local", str(repo), str(restored)],
                           check=True, capture_output=True)
            subprocess.run(["git", "apply", "--binary", "-"], cwd=restored, input=source_patch,
                           check=True, capture_output=True)
            for name, entry in saved["untracked_files"].items():
                path = restored / name
                path.parent.mkdir(parents=True, exist_ok=True)
                if entry["kind"] == "symlink":
                    os.symlink(decode(entry), os.fsencode(path))
                else:
                    path.write_bytes(decode(entry))
                    path.chmod(int(entry["mode"], 8))
            names = [os.fsdecode(name) for name in git("ls-files", "-z").split(b"\0") if name]
            for name in {*names, *saved["untracked_files"]}:
                original, copy_path = repo / name, restored / name
                self.assertEqual(original.is_symlink(), copy_path.is_symlink(), name)
                self.assertEqual(original.exists(), copy_path.exists(), name)
                if original.is_symlink():
                    self.assertEqual(original.readlink(), copy_path.readlink(), name)
                elif original.exists():
                    self.assertEqual(original.read_bytes(), copy_path.read_bytes(), name)
                    self.assertEqual(original.stat().st_mode & 0o7777, copy_path.stat().st_mode & 0o7777, name)
            self.assertFalse((restored / "renamed.txt").exists())
            self.assertFalse((restored / "generated").exists())
