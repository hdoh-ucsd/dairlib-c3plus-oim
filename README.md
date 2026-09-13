# dairlib C3+/OIM benchmark

Native sampling-based C3+ pushing on xArm6, matched OIM scenes, passive logs,
and offline evaluation/video replay. This cleanup changes setup and workflow
tools, not controller algorithms or meshes.

Repository: https://github.com/hdoh-ucsd/dairlib-c3plus-oim

Default branch: `integration/c3plus-oim-consolidated`

## Setup

Use Linux x86-64 / Ubuntu 24.04, or WSL2 with Docker integration enabled.
Docker must be installed and running. Use a normal user-owned checkout,
not a root-owned directory.

```bash
git clone --branch integration/c3plus-oim-consolidated https://github.com/hdoh-ucsd/dairlib-c3plus-oim.git
cd dairlib-c3plus-oim
./docker/shell.sh
```

Nikola's [Docker commit](https://github.com/hdoh-ucsd/dairlib-c3plus-oim/commit/cf092c53dab655100dc28b000d9554ad9fb0897c)
provides a toolchain recipe and launcher, not a published registry reference.
The launcher builds `dairlib-c3plus-oim:latest` on first use. Rebuild after
Dockerfile/requirements changes:

```bash
docker build --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" -t dairlib-c3plus-oim:latest docker
```

If given a published image, export `DAIRLIB_IMAGE=registry/name:tag` (prefer a
digest); the launcher pulls it if absent. An older image will not automatically
include this checkout's dependency fixes.

Drake is pinned to 1.51.1; Bazel to 8.4.0 by `.bazeliskrc`.
`docker/requirements.txt` pins Python Drake, analysis/replay and mesh packages.
The Dockerfile supplies FFmpeg, headless rendering libraries, Graphviz and
procman. Bazel builds Drake/C3 from the module pins. These simulated runs do
not need ROS. SNOPT/Gurobi distributions in the Docker build context remain
subject to their license/access/redistribution terms.

Inside the container:

```bash
bazel build //examples/sampling_c3:franka_sim //examples/sampling_c3:franka_osc_controller //examples/sampling_c3:franka_sampling_c3_controller
python3 -m pip check
python3 tools/scene_smoke/check_environment.py --require-binaries --check-scenes
python3 -m unittest discover -s tools/scene_smoke -p test_workflow.py
```

The historical `franka_*` targets support xArm6; the runner selects
`--robot_model=xarm6`. Build binaries in your own environment; frozen executable
snapshots are no longer used.

All processes of one trial run in one isolated container, with LCM multicast
routed over loopback. Bazel uses a named cache volume. Meshcat is published on
localhost port 7000. Override `MESHCAT_PORT`, `DAIRLIB_CPUS`, and `DAIRLIB_MEM`
(defaults 7000, 24 CPUs, 24 GB) if needed. CPU quotas and competing workloads
change achieved controller rate. Do not build or run another trial alongside
an active simulation.

## Run

Run inside the container from the repository root. The reproduction workflow
uses **serial trials, seed 42 only**, and packages each before starting the next.
Output folders must be new; existing/partial runs are never wiped automatically.

One baseline trial, with a 600-second recorder wall-time cap:

```bash
python3 tools/scene_smoke/run_experiment.py --scene single_obstacle --variant baseline --start 1 --goal 3 --seed 42 --cap 600 --out results/reproduce_one
```

Choose `--variant relu` for footprint-aware ReLU ranking (`eps=0.01`, `w=200`).
Both variants use obstacle mode `lcs_contact`; inherited `SAMPLING_C3_*` knobs
are cleared. Start/goal indices are 1–5.

| Scene flag | Manipulated object | Environment |
| --- | --- | --- |
| `open_task` | T-shape | Open table |
| `single_obstacle` | T-shape | One box |
| `shelf_gap` | T-shape | Shelf gap |
| `ycb_clutter` | T-shape | YCB clutter obstacles |
| `icra_sign` | C glyph | ICRA sign obstacles |
| `slalom` | T-shape | Slalom obstacles |

Short dependency checks for both variants in all scenes (not performance tests):

```bash
python3 tools/scene_smoke/run_grid_campaign.py --scenes open_task single_obstacle shelf_gap ycb_clutter icra_sign slalom --variant both --pairs smoke --seed 42 --cap 15 --output-root results/dependency_smoke
```

Reproduce the exact 25 completed full jobs from the last session:

```bash
python3 tools/scene_smoke/run_grid_campaign.py --manifest tools/scene_smoke/campaign_seed42.json --seed 42 --output-root results/reproduce_nightly
```

The manifest preserves order, variant, pose pair, cap and original evaluation
goals, including rounding. For a fresh grid use `--pairs all` (25 pairs per
scene) or `--pairs diagonal` (five). Add `--resume` to skip `RUN_COMPLETE`
folders; incomplete folders are preserved and refused. Create
`STOP_AFTER_CURRENT` in the output root to stop after the active run finishes
and is packaged. Completion means artifacts exist, not that the goal was reached.

### Outputs and interpretation

Each trial keeps simulator/OSC/planner/recorder and packaging logs,
`steps_raw.jsonl`, approximately 10 Hz `state_trace.jsonl`, evaluation CSV/plot,
result JSON/manifest, cost plot, MP4, and `runtime_status.json` with seed
confirmation, binary hashes and logged failures. Videos are sampled offline
replays, not wall-clock screen recordings; default maximum 1,200 frames at
10 fps. Raw traces retain the trajectory.

The last session's local review is
`results/nightly_20260912_seed42_serial/REVIEW.md` with CSV/JSON summaries.
It completed 25 full trials: baseline 2/13 successes, ReLU 0/12, with 13 logged
controller assertions. The other 75 planned trials never started. Old local
results may not exist in a fresh clone. Seed alone does not guarantee identical
outcomes: asynchronous communication, solver behavior and machine load matter.

Cost reconstructions are offline diagnostics, not measured controller
objectives or fully equivalent OIM/MPPI costs. Missing blocks are not zero.
Current aggregate `*_result.json` does **not** match the supplied MPPI trajectory
schema; see [evaluation JSON](docs/evaluation_json.md) for its format and gaps.

## Verification and next stage

See [verification](docs/reproduction_verification.md) for checks and limitations.
Host tests do not establish Docker runtime compatibility: a Docker daemon is
not available in the validation workspace.

After your reproduction/mesh review: fix the T-shape, power drill, hammer,
banana and sugar-box meshes; add their shared multi-object launcher; implement
synchronized metrics recording/export matching the supplied JSON.
These changes are separate from the present setup/documentation cleanup.

Upstream paper/hardware instructions remain in
[the sampling C3 example](examples/sampling_c3/README.md).
Duplicate scene READMEs, historical operational ledgers, forensic-only scripts
and frozen executable snapshots are removed from the operational tree and
recoverable from Git history. Existing results, controller implementations,
and meshes/configurations are retained.
