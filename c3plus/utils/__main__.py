"""Build, run and inspect reproducible C3+ experiments."""
import argparse
import subprocess
import sys

from c3plus.configs.catalog import OBSTACLE_COSTS, SCENES
from c3plus.configs.poses import pose_ids

COMMANDS = {
    "check": ("c3plus.runtime.environment", "Check dependencies, multicast, and scene assets"),
    "run": ("c3plus.utils.run", "Run and package one indexed scene/start/goal experiment"),
    "campaign": ("c3plus.utils.campaign", "Launch the full benchmark or a targeted grid, with safe resume"),
    "run_launch": ("c3plus.utils.campaign", "Targeted 180-trial goal-2/yaw comparison"),
    "run_launch_simple_s2": ("c3plus.utils.campaign", "Start-2 run: 36 trials at goal 2 with yaw 90/0/-90 degrees"),
    "eval": ("c3plus.evaluation.run_eval", "Aggregate saved C3+ trials into a comparison table"),
    "postprocess": ("c3plus.evaluation.postprocess", "Recompute metrics from an existing run"),
    "compact": ("c3plus.evaluation.package", "Consolidate a completed run into JSON and MP4, then remove intermediates"),
    "render": ("c3plus.visualization.render", "Render a saved result JSON or trace to MP4"),
    "visualize": ("c3plus.visualization.objects", "Save a PNG of an object in simulation coordinates"),
    "cost-figure": ("c3plus.visualization.costs", "Plot cost diagnostics from existing metrics"),
}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        prog="python3 -m c3plus.utils", description=__doc__,
        epilog="Use COMMAND --help for command options. See README.md for the experiment workflow.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("build", help="Build the three native experiment targets", add_help=False)
    subparsers.add_parser("scenes", help="List supported scenes and start/goal indices")
    for name, (_, help_text) in COMMANDS.items():
        subparsers.add_parser(name, help=help_text, add_help=False)
    if argv and argv[0] == "build":
        from c3plus.runtime.build import main as build
        return build(argv[1:])
    if argv and argv[0] in COMMANDS:
        leaf_args = argv if argv[0] in ("run_launch", "run_launch_simple_s2") else argv[1:]
        return subprocess.call([sys.executable, "-m", COMMANDS[argv[0]][0], *leaf_args])
    args = parser.parse_args(argv)
    if args.command == "scenes":
        for scene in SCENES:
            print(f"{scene:18} starts={','.join(pose_ids('start', scene))}  "
                  f"goals={','.join(pose_ids('goal', scene))}  obstacle_costs={','.join(OBSTACLE_COSTS)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
