"""Capture launch-time source bytes and environment versions."""
import base64
import hashlib
from importlib.metadata import PackageNotFoundError, version
import os
from pathlib import Path
import platform
import stat
import subprocess
import sys

def runtime_versions():
    packages = {}
    for name in ("drake", "numpy", "scipy", "matplotlib", "Pillow", "trimesh", "PyYAML"):
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            packages[name] = None
    return {"python": sys.version, "platform": platform.platform(), "packages": packages,
            "container_image": os.environ.get("C3PLUS_CONTAINER_IMAGE"),
            "container_image_id": os.environ.get("C3PLUS_CONTAINER_IMAGE_ID")}

def capture_source_state(repo):
    """Capture launch-time source bytes without changing the worktree or index.

    The binary patch reproduces the net tracked working tree relative to HEAD;
    staging distinctions and ignored files are not part of this source snapshot.
    Existing binary hashes identify executables separately and do not prove they
    were built from this captured checkout.
    """
    repo = Path(repo).resolve()

    def git(*arguments):
        try:
            return subprocess.check_output(["git", "--no-optional-locks", *arguments],
                                           cwd=repo, stderr=subprocess.PIPE)
        except subprocess.CalledProcessError as exc:
            raise RuntimeError("Cannot capture source state: " +
                               exc.stderr.decode("utf-8", errors="replace").strip()) from exc

    def encoded(raw):
        try:
            content, encoding = raw.decode("utf-8"), "utf-8"
        except UnicodeDecodeError:
            content, encoding = base64.b64encode(raw).decode("ascii"), "base64"
        return {"encoding": encoding, "content": content, "size_bytes": len(raw),
                "sha256": hashlib.sha256(raw).hexdigest()}

    if Path(os.fsdecode(git("rev-parse", "--show-toplevel")).strip()).resolve() != repo:
        raise ValueError("Source capture requires the repository root")
    base_commit = git("rev-parse", "HEAD").decode("ascii").strip()
    patch_args = ("diff", "--binary", "--full-index", "--no-ext-diff", "--no-textconv", "HEAD", "--")
    status_args = ("status", "--porcelain=v1", "--untracked-files=all", "-z")
    source_patch = git(*patch_args)
    status = git(*status_args)
    untracked = {}
    for raw_name in git("ls-files", "--others", "--exclude-standard", "-z").split(b"\0"):
        if not raw_name:
            continue
        name = os.fsdecode(raw_name)
        relative = Path(name)
        path = repo / relative
        if relative.is_absolute() or ".." in relative.parts or not path.parent.resolve().is_relative_to(repo):
            raise ValueError(f"Untracked source path escapes the repository: {name}")
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode):
            raw, kind = os.readlink(os.fsencode(path)), "symlink"
        elif stat.S_ISREG(info.st_mode):
            with os.fdopen(os.open(path, os.O_RDONLY | os.O_NOFOLLOW), "rb") as stream:
                raw, kind = stream.read(), "file"
        else:
            raise ValueError(f"Cannot capture non-file untracked source: {name}")
        after = path.lstat()
        if (info.st_ino, info.st_mode, info.st_size, info.st_mtime_ns) != (
                after.st_ino, after.st_mode, after.st_size, after.st_mtime_ns):
            raise RuntimeError(f"Untracked source changed during capture: {name}")
        untracked[name] = {"kind": kind, "mode": format(stat.S_IMODE(info.st_mode), "04o"), **encoded(raw)}
    if (git("rev-parse", "HEAD").decode("ascii").strip() != base_commit or
            git(*patch_args) != source_patch or git(*status_args) != status):
        raise RuntimeError("Repository changed during source capture; retry before launching")
    return {"format": "git-source-state/v1", "base_commit": base_commit,
            "worktree_dirty": bool(status),
            "scope": "Net staged and unstaged tracked changes relative to HEAD, plus nonignored "
                     "untracked files. Git index staging and ignored files are not reproduced.",
            "tracked_patch": encoded(source_patch), "git_status": encoded(status),
            "untracked_files": untracked}
