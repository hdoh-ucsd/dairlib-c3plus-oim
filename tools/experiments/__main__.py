"""One entry point for building, running, and inspecting scene experiments."""
import argparse
import shlex
import subprocess
import sys

if __package__:
    from .catalog import BUILD_TARGETS, OBSTACLE_COSTS, REPO, SCENES, TOOL_DIR
else:
    from catalog import BUILD_TARGETS, OBSTACLE_COSTS, REPO, SCENES, TOOL_DIR

COMMANDS = {
    "check": ("check_environment.py", "Check dependencies, multicast, and scene assets"),
    "run": ("run_experiment.py", "Run and package one indexed scene/start/goal experiment"),
    "campaign": ("run_grid_campaign.py", "Run a manifest or grid serially, with safe resume"),
    "run_launch": ("run_grid_campaign.py", "Full run: 180 trials at goal 2 with yaw 90/0/-90 degrees"),
    "run_launch_simple_s2": ("run_grid_campaign.py", "Start-2 run: 36 trials at goal 2 with yaw 90/0/-90 degrees"),
    "postprocess": ("postprocess_run.py", "Recompute metrics from an existing run"),
    "render": ("render_run_3d.py", "Render an existing trace to MP4"),
    "visualize_mesh": ("visualize_mesh.py", "Save a PNG of an object in simulation coordinates"),
    "cost-figure": ("cost_fig.py", "Plot cost diagnostics from existing metrics"),
}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        prog="python3 -m tools.experiments", description=__doc__,
        epilog="Use COMMAND --help for command options. See README.md for the experiment workflow.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build", help="Build the three native experiment targets")
    build.add_argument("--jobs", type=int, help="Limit concurrent Bazel build jobs")
    build.add_argument("--dry-run", action="store_true", help="Print the build command without executing it")
    subparsers.add_parser("scenes", help="List supported scenes and start/goal indices")
    for name, (_, help_text) in COMMANDS.items():
        subparsers.add_parser(name, help=help_text, add_help=False)
    # Forward all leaf arguments unchanged, including --help. Only the chosen
    # command loads its scientific/native dependencies.
    if argv and argv[0] in COMMANDS:
        leaf_args = argv if argv[0] in ("run_launch", "run_launch_simple_s2") else argv[1:]
        return subprocess.call([sys.executable, str(TOOL_DIR / COMMANDS[argv[0]][0]), *leaf_args])
    args = parser.parse_args(argv)
    if args.command == "scenes":
        for scene in SCENES:
            print(f"{scene:18} starts=1..5  goals=1..5  obstacle_costs={','.join(OBSTACLE_COSTS)}")
        return 0
    if args.jobs is not None and args.jobs < 1:
        parser.error("--jobs must be positive")
    command = ["bazel", "build", *BUILD_TARGETS]
    if args.jobs is not None:
        command.append(f"--jobs={args.jobs}")
    if args.dry_run:
        print(shlex.join(command))
        return 0
    try:
        return subprocess.call(command, cwd=REPO)
    except FileNotFoundError:
        parser.exit(1, "Bazel is missing. Set up the environment described in docker/README.md first.\n")


if __name__ == "__main__":
    raise SystemExit(main())
