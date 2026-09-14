# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot in six matched OIM scenes.
This checkout contains the Drake simulator, controllers, scene assets, and
[experiment CLI](tools/experiments/). Docker is the supported environment for
building and running experiments; host Python, Conda, ROS, and a GPU are not required.

[Quick Start](#quick-start) · [Docker Environment](#docker-environment) ·
[Experiments](#experiments) · [Results](#results) ·
[Troubleshooting](#troubleshooting) · [Detailed reference](docs/experiment_reference.md)

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
python3 -m tools.experiments build --jobs 4
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
python3 -m tools.experiments check --require-binaries --check-scenes
```

This prints a JSON report and exits nonzero if validation fails. Resolve failures
before starting experiments.

**6. CONTAINER — run the inexpensive workflow tests.**

```bash
python3 -m unittest tools.experiments.test_workflow tools.experiments.test_object_runs tools.experiments.test_run_artifacts -v
```

These tests do not launch experiments.

**7. CONTAINER — inspect a trial without writing files or starting simulation.**

```bash
python3 -m tools.experiments run \
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
python3 -m tools.experiments run \
  --scene open_task --objects T_block \
  --start 2 --goal 2 \
  --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/example_run
```

`--cap` is the per-trial recorder wall-time budget in seconds; startup, success
settling, and packaging can add time. For a cheap startup check, use
`--cap 10 --max-frames 40 --out results/smoke` instead. Use a fresh output folder
for each run. A completed trial leaves one JSON and one MP4 in that folder,
visible from the host checkout.

## Docker Environment

[docker/Dockerfile](docker/Dockerfile) defines the single toolchain image:
Ubuntu 24.04, Drake 1.51.1, Bazel 8.4.0, native solver dependencies, and
[pinned Python packages](docker/requirements.txt). The runtime `python3` uses an
isolated environment at `/opt/push-anything-venv`; native build prerequisites
remain separate. Both solver archives tracked under `docker/` are required for
image construction. Gurobi 10.0.3 is needed at link time; the shipped C3+
projection uses OSQP. Other solver configurations need their own runtime setup.

The default image tag includes a hash of Docker build inputs and the build
UID/GID. Changes to the Dockerfile, entrypoint, requirements, build-context rules,
or solver archives select a new tag automatically. Repeated launches reuse an
existing matching image. `DAIRLIB_IMAGE=TAG_OR_DIGEST ./docker/shell.sh` explicitly
selects another image and pulls it if missing; use only a compatible toolchain.

| What changed | Required action |
| --- | --- |
| Dockerfile, dependencies, or entrypoint | **HOST:** `./docker/shell.sh --build` rebuilds the image and opens a new container |
| Only want to build the image | **HOST:** `./docker/shell.sh --build-only` builds without opening a shell |
| Native C++ or LCM types | **CONTAINER:** `python3 -m tools.experiments build --jobs 4` rebuilds all three binaries |
| Python or YAML in the checkout | Visible on the next invocation through the bind mount; image rebuild normally unnecessary |

The checkout stays on the host. Compiled objects and external sources stay in
`dairlib-c3plus-oim-bazel-cache-u<uid>-g<gid>`, mounted at
`/home/dairlib/.cache/bazel`. The volume survives container exit. Bazel groups its
workspace shortcuts under the ignored `.build/` directory; `.build/bin/` points
to executables, `.build/out/` to build outputs, and `.build/testlogs/` to test logs.
The linked build data stays in the volume. These shortcuts can appear broken
on the host because their targets are inside Docker. Do not delete the volume
if you want to reuse compiled dependencies.

Nonroot hosts use a container user with the host UID/GID, so generated files
have matching ownership. A root host runs as UID 0 with its own cache volume.
Startup can repair the cache root's ownership for root without recursively
changing existing contents. Nonroot cache mismatches fail with a useful message;
use the default volume or a correctly owned `DAIRLIB_CACHE_VOLUME`. The launcher
does not change checkout ownership. Explicit image overrides must match nonroot
host UID/GID labels.

LCM multicast uses the container's loopback interface and needs the granted
`NET_ADMIN` capability. Keep all processes for a trial in the same container.
Meshcat is published at `http://localhost:7000`. Exiting the shell removes the
container; the checkout and cache persist. To open another shell in an existing
container, run on the **HOST**, substituting a listed name:

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
docker exec -it CONTAINER_NAME bash
```

## Experiments

All commands below run **inside the container, from the repository root**.
Use `python3 -m tools.experiments COMMAND --help` for options, or the
[CLI and result reference](docs/experiment_reference.md) for full tables,
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
| `visualize_mesh` | Save a mesh PNG; `--ee-samples` also overlays candidates and saves their JSON |
| `campaign` | Plan/run a selected scene/start/goal grid or manifest |
| `run_launch`, `run_launch_simple_s2` | Fixed G2, three yaws, both costs: 180 trials or 36 restricted to S2 |

Both obstacle-cost presets use `lcs_contact`; their names do not change these
existing semantics. Seed 42 is the only supported experiment seed.

On `open_task`, run T_block and the four imported objects sequentially:

```bash
python3 -m tools.experiments run \
  --scene open_task --objects T_block sugar_box power_drill hammer banana \
  --start 2 --goal 2 --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/multi_object
```

Each object gets its own `<out>/<object>/` folder. Imported objects are supported
only on `open_task`: obstacle-contact integration still uses the built-in
footprints. T_block also supports `single_obstacle`, `shelf_gap`, `ycb_clutter`,
and `slalom`. For `icra_sign`, omit `--objects` to use its configured Cblock.
Previewing a mesh in another scene does not establish pushing support.

Inspect the configured T_block and EE candidates:

```bash
python3 -m tools.experiments visualize_mesh \
  --scene open_task --start 2 --view top --hide-robot --ee-samples \
  --output results/previews/T_block_ee.png
```

Plan the 180-trial fixed-goal campaign; remove `--dry-run` to execute:

```bash
python3 -m tools.experiments run_launch --output-root results/table2 --dry-run
```

Use `run_launch_simple_s2` for the 36-trial subset, with a separate output root.
Campaigns select each scene's default object and do not accept `--objects`.
See the [campaign reference](docs/experiment_reference.md#campaign-run_launch-run_launch_simple_s2-repeat-trials)
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
[result schema reference](docs/experiment_reference.md#results) for the established
alignment, success, frequency, and Table-II evaluation definitions. Historical
results are not included in a fresh clone.

## Reproducibility

Use the same repository commit, image, seed 42, selected configuration, and
resource settings. Runs save the requested image tag and immutable image ID,
configuration and binary hashes, package versions, and available source-state
provenance. Keep the JSON and video together; preserve the campaign plan for a
campaign. Do not assume seed 42 makes asynchronous execution identical across
machine loads.

The Ubuntu base image is pinned by digest; Drake's source version/checksum,
Bazel version, libbot2 source commit, and Python versions are pinned. Ubuntu apt
repositories are not snapshot-pinned and wheel hashes are not locked. The recipe
hash detects local input changes but cannot guarantee byte-identical rebuilds.
Retain the built image or a registry digest when sharing an exact environment.

## Resource Configuration

Defaults adapt to the Docker engine's reported resources: at most 24 CPUs and
75% of its RAM, capped at 24 GiB. The launcher validates limits before building
or starting a container. On Docker Desktop, these are the VM's resources;
increase its allocation in Docker settings if needed.

| HOST environment variable | Meaning |
| --- | --- |
| `DAIRLIB_CPUS` | Container CPU quota; positive number no greater than engine CPUs |
| `DAIRLIB_MEM` | Container RAM, e.g. `12g`; at least 4 GiB and no greater than engine RAM |
| `DAIRLIB_BAZEL_JOBS` | Positive build-worker count; default at most 8, reduced for CPU/RAM limits |
| `DAIRLIB_BAZEL_RAM_MB` | Bazel RAM budget in MiB; default at most 14000, reserving 3072 MiB for the JVM and tools |
| `DAIRLIB_CACHE_VOLUME` | Override the default UID/GID-specific cache volume |
| `MESHCAT_PORT` | Host port forwarding to container port 7000; default 7000 |

For example, on a host whose Docker engine has these resources:

```bash
DAIRLIB_CPUS=4 DAIRLIB_MEM=12g DAIRLIB_BAZEL_JOBS=2 ./docker/shell.sh
```

Inside the container, `build --jobs 4` overrides the environment's default job
count for that build. Heavy C++ compilation can exceed scheduling estimates;
reduce jobs if it runs out of memory. Runtime limits do not limit the separate
Docker image builder; configure that builder through Docker. See
`./docker/shell.sh --help` for all launcher options.

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Docker missing or daemon unavailable | Install/start Docker; enable Linux containers and, on Windows, WSL2 integration. Check `docker info` on the host. |
| Unsupported architecture or remote context | Use a local Linux amd64 Docker engine; bundled Gurobi and Java paths target amd64. |
| Docker Buildx unavailable | Install/enable Docker's Buildx plugin. It is required when the selected image needs building. |
| Checkout or bind mount error | Run from a complete writable clone; enable Docker Desktop access to that host directory. |
| Port 7000 already allocated | **HOST:** `MESHCAT_PORT=7001 ./docker/shell.sh`, then use `http://localhost:7001`; or enter an existing container with `docker exec`. |
| `Build franka_sim from this checkout first` or missing native flags | **CONTAINER:** run `python3 -m tools.experiments build --jobs 4` in the same checkout/container, then repeat `check`. |
| Python imports or `pip check` fail in an old image | **HOST:** `./docker/shell.sh --build`, then repeat container build and validation. |
| Memory limit rejected or compiler killed | Check Docker engine RAM, reduce jobs, or adjust the resource settings above. |
| Output folder exists / different campaign plan | Choose a fresh `--out`/`--output-root`. Campaign `--resume` requires a matching plan; `run` has no resume flag. |

Known exporter limitation: a 20-second validation run reached packaging but
failed with `Invalid exact native execution state`. The current exporter rejects
raw Drake quaternions when their squared norm differs from 1 by more than
`1e-5`; small integration drift crossed that limit. A 10-second startup run
packaged successfully. This is separate from Docker setup and remains unresolved;
longer-run packaging is not fully verified. Failed runs retain their raw data.
The exporter and evaluation definitions were left unchanged by this setup cleanup.

For `FATAL: mkdir('/home/dairlib/.cache/bazel'): Permission denied`, current
startup handles a root cache mismatch automatically. Rebuild and reopen an old
image with `./docker/shell.sh --build`. If recovering an **existing root shell
in an older image**, change only the cache root, then rebuild:

```bash
chown --no-dereference 0:0 /home/dairlib/.cache/bazel
python3 -m tools.experiments build --jobs 4
```

For nonroot shells, use the default UID/GID-specific volume or a cache owned by
that user. Do not recursively change ownership of existing Bazel cache contents.

## Repository Structure

| Path | Purpose |
| --- | --- |
| [docker/](docker/) | Single toolchain recipe, dependency pins, launcher, entrypoint, and launcher tests |
| [tools/experiments/](tools/experiments/) | Supported build/run/check CLI, recording, packaging, previews, and tests |
| [docs/experiment_reference.md](docs/experiment_reference.md) | Detailed CLI options, result schema, configuration mapping, and historical artifact notes |
| [examples/sampling_c3/](examples/sampling_c3/README.md) | Native controllers, simulator, shared configuration, models, and method citation |
| [tools/experiments/scene_configs/](tools/experiments/scene_configs/) | Six evaluation scene definitions and planner mappings |
| [tools/bazel](tools/bazel) | Active Bazelisk wrapper selecting ROS-free settings |
| [tools/workspace/gurobi/](tools/workspace/gurobi/) | Native Gurobi dependency integration |
| `.build/` | Ignored Bazel shortcuts; recreated by the native build command |
| `results/` | Ignored generated JSON/video, previews, and campaign outputs |

Bazel's root files serve different purposes: `MODULE.bazel` declares dependencies,
`BUILD.bazel` marks the root package, and `WORKSPACE` remains a root marker for
compatible tools. `.bazelrc`, `.bazeliskrc`, and `.bazelignore` configure builds,
the Bazel version, and source exclusions. Bazel generates `MODULE.bazel.lock`
beside the module file. These files remain at the root for automatic discovery;
they are not compiled outputs. `noros.bazelrc` is the active ROS-free override,
and `.bazelproject` supplies IDE settings.

Native `--demo_name` configurations and upstream research/CI material remain
for compatibility. They are not alternate setup paths for this benchmark.
Developer-only launcher checks (no daemon required) are
`python3 -m unittest discover -s docker/tests -v` and
`bash -n docker/shell.sh docker/entrypoint.sh`.
