# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot in six matched OIM scenes.
This checkout contains the Drake simulator, controllers, scene assets, and
[experiment CLI](tools/). Docker is the supported environment for
building and running experiments; host Python, Conda, ROS, and a GPU are not required.

[Quick Start](#quick-start) · [Docker Environment](#docker-environment) ·
[Experiments](#experiments) · [Results](#results) ·
[Troubleshooting](#troubleshooting) · [Architecture](docs/architecture.md)

## Quick Start

You need Git, Bash, and a local Docker engine running **Linux amd64 containers**.
Use Linux, Windows with Docker Desktop's WSL2 integration (run these commands
from a Linux checkout in WSL2), or Intel macOS with Docker Desktop. Native
PowerShell, remote Docker engines, and ARM/Apple Silicon emulation are not
supported by this launcher. Docker needs network access and substantial free
disk space for the first image and native builds. Allocate enough memory to the
Docker engine; the launcher reserves part for the host and requires at least
4 GiB for the container. More memory is recommended for compiling Drake.

**1. HOST — clone the checkout.**

```bash
git clone --branch integration/c3plus-oim-consolidated https://github.com/hdoh-ucsd/dairlib-c3plus-oim.git
cd dairlib-c3plus-oim
git rev-parse HEAD
```

**2. HOST — open the Docker environment.**

```bash
./docker/shell.sh
```

The launcher derives the mount path from this checkout, builds the selected
image if missing, and otherwise reuses it. The checkout is mounted at
`/home/dairlib/dairlib`; the Bazel cache lives in a persistent Docker volume.
You are now in a container shell at the repository root. Run every command in
steps 3–8 **inside this container**.

**3. CONTAINER — build the native simulator and both controllers.**

```bash
python3 -m tools build --jobs 4
```

This compiles `franka_sim`, `franka_osc_controller`, and
`franka_sampling_c3_controller`. The first build also compiles Drake and can take
substantially longer than subsequent builds.

**4. CONTAINER — verify Python dependencies.**

```bash
python3 -m pip check
```

**5. CONTAINER — verify binaries, imports, scenes, objects, and runtime prerequisites.**

```bash
python3 -m tools check --require-binaries --check-scenes
```

This prints a JSON report and exits nonzero if validation fails. Resolve failures
before starting experiments.

**6. CONTAINER — run the inexpensive workflow tests.**

```bash
python3 -m unittest discover -s tests
```

These tests do not launch experiments.

**7. CONTAINER — inspect a trial without writing files or starting simulation.**

```bash
python3 -m tools run \
  --scene open_task --objects T_block \
  --start 2 --goal 2 \
  --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/example_run --dry-run
```

Omitting `--goal-yaw-degrees` preserves the indexed goal's configured orientation.
The dry run prints the resolved settings; it does not check an existing output
folder for compatibility.

**8. CONTAINER — run that trial.**

```bash
python3 -m tools run \
  --scene open_task --objects T_block \
  --start 2 --goal 2 \
  --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/example_run
```

`--cap` is the per-trial recorder wall-time budget in seconds; startup, success
settling, and packaging can add time. For a cheap startup check, use
`--cap 10 --max-frames 40 --out results/smoke/startup_check` instead. Use a fresh
output folder for each run. A completed trial leaves one JSON and one MP4 in
that folder, visible from the host checkout.

## Docker Environment

Use `./docker/shell.sh` on the host to open the supported image. It contains
Ubuntu 24.04, Drake 1.51.1, Bazel 8.4.0, native solver dependencies and the
[pinned Python environment](docker/requirements.txt). The launcher selects a
recipe-derived image tag and keeps build data in a persistent volume. Files in
the checkout are visible through the bind mount; Python/YAML changes normally
need no image rebuild, while native changes require rebuilding the binaries.

To rebuild the image, run `./docker/shell.sh --build` on the host. To enter an
already running container, list its name and open another shell:

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
docker exec -it CONTAINER_NAME bash
```

See [reproduction and environment](docs/reproduction.md) for image overrides,
resource limits, cache ownership and platform support. Bazel's ignored `.build/`
shortcuts point into the cache; their targets may look unavailable on the host.

## Experiments

All commands below run **inside the container, from the repository root**.
Use `python3 -m tools COMMAND --help` for options, or the
[experiment reference](docs/experiments.md) for full tables,
mesh/EE previews, campaign behavior, and data definitions.

| Command / option | Purpose |
| --- | --- |
| `scenes` | List six scenes and their start/goal indices |
| `run --scene NAME --objects NAME [...]` | Run one object or a serial batch of separate trials |
| `--start`, `--goal` | Independent indices 1–5; orientations come from the catalogue unless overridden |
| `--goal-yaw-degrees` | Explicit absolute goal yaw, e.g. 90, 0, or −90 degrees |
| `--obstacle_cost exponential` | Preserved baseline preset; exponential ranking is currently suppressed by `lcs_contact` |
| `--obstacle_cost relu` | Active footprint-aware ReLU ranking, `eps=0.01`, `w=200` |
| `--steps B` | Optional applied-policy budget, including reposition; unlimited by default |
| `visualize` | Save a mesh PNG; `--ee-samples` also overlays candidates and saves their JSON |
| `campaign` | Plan/run a selected scene/start/goal grid or manifest |
| `run_launch`, `run_launch_simple_s2` | Fixed G2, three yaws, both costs: 180 trials or 36 restricted to S2 |

Both obstacle-cost presets use `lcs_contact`; their names do not change these
existing semantics. Seed 42 is the only supported experiment seed.

On `open_task`, run T_block and the four imported objects sequentially:

```bash
python3 -m tools run \
  --scene open_task --objects T_block sugar_box power_drill hammer banana \
  --start 2 --goal 2 --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/object_generalization/multi_object
```

Each object gets its own `<out>/<object>/` folder. Imported objects are supported
only on `open_task`: obstacle-contact integration still uses the built-in
footprints. T_block also supports `single_obstacle`, `shelf_gap`, `ycb_clutter`,
and `slalom`. For `icra_sign`, omit `--objects` to use its configured Cblock.
Previewing a mesh in another scene does not establish pushing support.

Inspect the configured T_block and EE candidates:

```bash
python3 -m tools visualize \
  --scene open_task --start 2 --view top --hide-robot --ee-samples \
  --output results/previews/T_block_ee.png
```

Plan the 180-trial fixed-goal campaign; remove `--dry-run` to execute:

```bash
python3 -m tools run_launch --output-root results/table2 --dry-run
```

Use `run_launch_simple_s2` for the 36-trial subset, with a separate output root.
Campaigns select each scene's default object and do not accept `--objects`.
See the [campaign reference](docs/experiments.md#campaign-run_launch-run_launch_simple_s2-repeat-trials)
for the generic 300-run grid, exact defaults, and resume rules.

## Results

Generated data belongs under `results/`, which is ignored by Git. Each completed
trial keeps one `*_result.json` and one MP4. The JSON embeds recordings, resolved
configurations, diagnostics, hashes, and completion status. Intermediates are
removed only after successful validation; failures preserve them for recovery.
Campaign plans, driver logs, and summaries remain at the campaign root.

The JSON keeps physical execution, planning updates, and recording snapshots
separate. Simulation time and monotonic execution wall time remain distinct;
execution timing is not optimizer solve timing. See the
[result schema reference](docs/evaluation_schema.md) for the established
alignment, success, frequency, and Table-II evaluation definitions. Historical
results are not included in a fresh clone.

## Reproducibility

Keep the result JSON and video together with the recorded commit, image identity,
campaign plan and resource settings. Results retain configuration, asset and
binary hashes, package versions, and available source-state provenance. Seed 42
does not make asynchronous execution identical across machine loads. The
[reproduction guide](docs/reproduction.md) explains image pinning, saved settings
and the limits of exact reconstruction.

## Troubleshooting

For a busy port, use **HOST:** `MESHCAT_PORT=7001 ./docker/shell.sh`. For missing
binaries or native flags, rebuild with **CONTAINER:** `python3 -m tools build`.
Use a fresh output directory for each run. Other environment and cache issues
are covered in [troubleshooting](docs/reproduction.md#troubleshooting).

The existing exporter can reject small raw-quaternion norm drift with
`Invalid exact native execution state`. A prior 10-second startup packaged
successfully, while a 20-second check encountered this limit. Longer-run
packaging is not fully verified; failed runs retain their raw evidence. This
structural cleanup preserves that guard and the evaluation definitions.

## Repository Structure

| Path | Purpose |
| --- | --- |
| [c3plus/](c3plus/) | Configuration, experiments, runtime, recording, evaluation and visualization implementations |
| [tools/](tools/) | User-facing commands: `python3 -m tools COMMAND` |
| [tests/](tests/) | Tests mirroring source responsibilities and integration checks |
| [docs/](docs/architecture.md) | Architecture, experiments, evaluation schema and reproduction |
| [docker/](docker/) | Toolchain recipe, dependency pins and environment launcher |
| [examples/sampling_c3/](examples/sampling_c3/README.md) | Native controllers, simulator, shared configuration, models and method citation |
| [build_support/](build_support/) | Gurobi dependency rules and build overrides |
| [tools/bazel](tools/bazel) | Bazelisk wrapper using the ROS-free build override |
| `.build/` | Ignored Bazel shortcuts; actual build data stays in the cache |
| `results/` | Ignored generated JSON/video, previews and campaign evidence |

Bazel's root module/package markers and hidden configuration files remain in
place for automatic discovery. `build_support/noros.bazelrc` holds the ROS-free
override. Native compatibility demos and other dairlib components remain separate
from the normal benchmark workflow. See [architecture](docs/architecture.md) for
the package boundaries.
