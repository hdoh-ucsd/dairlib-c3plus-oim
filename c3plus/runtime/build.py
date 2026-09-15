"""Build the three native targets from this checkout."""
import argparse
import shlex
import subprocess

from c3plus.configs.paths import BUILD_TARGETS, REPO


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jobs", type=int, help="Limit concurrent Bazel build jobs")
    parser.add_argument("--dry-run", action="store_true", help="Print the build command without executing it")
    args = parser.parse_args(argv)
    if args.jobs is not None and args.jobs < 1:
        parser.error("--jobs must be positive")
    command = [str(REPO / "build_support/bazel"), "build", *BUILD_TARGETS]
    if args.jobs is not None:
        command.append(f"--jobs={args.jobs}")
    if args.dry_run:
        print(shlex.join(command))
        return 0
    try:
        return subprocess.call(command, cwd=REPO)
    except FileNotFoundError:
        parser.exit(1, "Bazel is missing. Open the Docker environment with ./docker/shell.sh; "
                    "see the root README.md Quick Start.\n")


if __name__ == "__main__":
    raise SystemExit(main())
