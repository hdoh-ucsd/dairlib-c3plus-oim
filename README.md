# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot in six matched OIM scenes.
This checkout contains the Drake simulator, controllers, scene assets, and
[experiment CLI](c3plus/utils/__main__.py). Docker is the supported environment for
building and running experiments; host Python, Conda, ROS, and a GPU are not required.

[Quick Start](#quick-start) · [Docker Environment](#docker-environment) ·
[Run All Experiments](#run-all-experiments) · [Results](#results) ·
[Troubleshooting](#troubleshooting) · [Repository Structure](#repository-structure)

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
steps 3–7 **inside this container**.

**3. CONTAINER — build the native simulator and both controllers.**

```bash
python3 -m c3plus.utils build --jobs 4
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
python3 -m c3plus.utils check --suite full --require-binaries --check-scenes
```

This prints a JSON report and exits nonzero if validation fails. Resolve failures
before starting experiments.

**6. CONTAINER — run the inexpensive workflow tests.**

```bash
python3 -m unittest discover -s tests
```

These tests do not launch experiments.

**7. CONTAINER — [run all experiments](#run-all-experiments).**

Use the single campaign command below to launch every task, object and
initial/goal pose combination. Add `--dry-run` to inspect the complete selection
first without starting experiments.

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

Bazel's ignored `.build/` shortcuts point into the persistent cache; their
targets may look unavailable on the host. The default cache volume is specific
to the host UID/GID. Exiting the shell removes the container, while the checkout
and cache remain. Keep a trial's processes in the same container for LCM.

Use these environment variables on the **host** to customize the launcher:

| Variable | Meaning |
| --- | --- |
| `DAIRLIB_IMAGE` | Compatible image tag or digest; the default tag follows the Docker recipe inputs |
| `DAIRLIB_CPUS`, `DAIRLIB_MEM` | Container resource limits, e.g. `4` and `12g`; defaults are at most 24 CPUs and 75% of engine RAM, capped at 24 GiB |
| `DAIRLIB_BAZEL_JOBS` | Build worker count; `build --jobs N` overrides it for one build |
| `DAIRLIB_BAZEL_RAM_MB` | Bazel RAM budget in MiB |
| `DAIRLIB_CACHE_VOLUME` | Override the default UID/GID-specific cache volume |
| `MESHCAT_PORT` | Host port for Meshcat; default 7000 |

For example: `DAIRLIB_CPUS=4 DAIRLIB_MEM=12g DAIRLIB_BAZEL_JOBS=2 ./docker/shell.sh`.
Limits must fit the Docker engine's resources; Docker Desktop users may need to
increase its VM allocation. Image overrides must match the host UID/GID for
nonroot users. Use `./docker/shell.sh --build-only` to build without opening a
shell, or `./docker/shell.sh --help` for all launcher options.

## Experiments

### Run all experiments

After the Quick Start setup, run this single command **inside the container,
from the repository root**:

```bash
python3 -m c3plus.utils campaign \
  --suite full --seed 42 --cap 600 --resume \
  --out results/full_campaign
```

`--suite full` automatically selects **all six tasks, all five objects, and
every initial-pose × goal-pose combination**. No per-task commands, object lists
or shell loops are needed.

| Selection | Values |
| --- | --- |
| Tasks | `icra_sign`, `open_table`, `shelf_gap`, `single_obstacle`, `slalom`, `ycb_clutter` |
| Manipulated objects | `T_shape`, `hammer`, `sugar_box`, `power_drill`, `banana` |
| Initial poses | S1, S2, S3, S4, S5 for each task |
| Goal poses | G1, G2, G3, G4, G5 for each task |
| Pose combinations | Every start is paired with every goal: 25 pairs per task/object, including S1→G1 through S5→G5 |
| Total | **6 tasks × 5 objects × 5 starts × 5 goals = 750 runs** |
| Seed | 42 for every trial |
| Cost | `exponential` by default; `--obstacle_cost relu` selects the other existing preset |

The launcher runs serially in task → object → start → goal order and validates
the whole selection before trial 1. `--cap 600` sets each trial's recorder
wall-time budget; startup, success settling and packaging add time. Replace
`results/full_campaign` with your chosen output directory.

To **preview the plan**, append `--dry-run` to the same command. This validates
poses and assets and prints the full manifest plus completed, pending,
invalid/partial and total counts, without writing run data or launching trials.
To **resume**, repeat the command with the same settings and output directory.
`--resume` also works for a fresh campaign. Valid completed JSON/video packages
are skipped; partial or corrupt trials are reported and preserved. A different
selection or configuration requires a new output directory.

Create `<output-directory>/STOP_AFTER_CURRENT` to stop after the current trial
is packaged. Remove that file and repeat the same command to continue.

**Validation limit:** all 750 configurations and 300 start/goal endpoints pass
static preflight, but some 10-second trials still hit the existing strict
native-quaternion export guard. A long campaign can halt there and preserve the
partial run for inspection. The full 600-second-per-trial campaign has not been
runtime validated.

### Pose and object definitions

Each initial and goal pose in [examples/poses/](examples/poses/) contains
`[x, y, yaw]`: position in metres and orientation in radians. The full launcher
uses both poses' saved orientations for every pair.

The local pose files match the [pinned OIM source](https://github.com/NikolaRaicevic2001/Object-Informed-Manipulation-MJX/tree/a31203d8e9a347ba7bf373954a4b7e4059d1e350/examples/poses)
byte-for-byte. That source defines **task-specific** coordinates: every object
within a task uses the same IDs and numeric poses, while some tasks differ.
Those differences are preserved rather than replacing them with one task's
coordinates. [Pose provenance](examples/poses/provenance.json) records the commit
and hashes. Runtime reads only these local files and needs no OIM checkout or
network access. Goal orientation comes from the selected pose unless explicitly
overridden for a targeted debugging run.

`T_shape` maps to the existing native T-shaped profile; `open_table` maps to the
native scene `open_task`. Native assets retain their names. C-block models remain
for native compatibility/history but are not manipulated experiment options.
Tasks select the environment; the object selection supplies its model, sampling
geometry, footprint and support height. The launcher checks all selected
combinations and rejects incompatible poses before running anything.

<details>
<summary>Optional CLI commands: individual runs, smaller grids and previews</summary>

The full campaign above covers the complete benchmark. The following commands
are available for targeted checks and visualization.

### `run`: object selection and obstacle costs

For one experiment or a cheap startup check:

```bash
python3 -m c3plus.utils run \
  --task open_table --object T_shape --start 2 --goal 2 \
  --obstacle_cost exponential --seed 42 --cap 10 --max-frames 40 \
  --out results/smoke/startup_check
```

Add `--dry-run` to inspect it, or use a longer cap and fresh output directory for
a substantive trial. `run` has no resume flag. Use
`python3 -m c3plus.utils COMMAND --help` for each command's complete options.

| Option | Meaning |
| --- | --- |
| `--task NAME` | Required task; `--scene` remains an alias |
| `--object NAME` / `--objects NAME [...]` | One object (default `T_shape`) or a serial multi-object batch; mutually exclusive |
| `--start`, `--goal` | Independent pose IDs 1–5, default 1 |
| `--goal-yaw-degrees` | Optional absolute goal yaw: 90, 0 or −90 degrees; omit to preserve the source pose |
| `--obstacle_cost` | `exponential` (default) or `relu` |
| `--cap SECONDS` | Recorder wall-time budget per trial, default 600 |
| `--steps B` | Applied-policy budget including reposition; unlimited when omitted |
| `--out PATH` | Fresh trial directory, or parent directory for a multi-object batch |
| `--max-frames`, `--port` | Maximum video frames (1200) and trial LCM port (18001) |

Run all five objects on the open table:

```bash
python3 -m c3plus.utils run \
  --task open_table --objects T_shape sugar_box power_drill hammer banana \
  --start 2 --goal 2 --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/object_generalization/open_table
```

All five objects are supported across the six tasks, subject to pose preflight.
One selected object writes directly to `--out`; multiple objects write to
`<out>/<object>/`. All destinations are checked before the batch starts, and a
launch or packaging error stops it. Progress messages appear every ten recorder
snapshots; their `step` counter is separate from physical execution steps.
Recording can continue five wall seconds after first success to capture settling.

Both cost presets preserve `lcs_contact` handling: `exponential` ranking is
currently suppressed in that mode; `relu` uses the existing footprint-aware
ranking with `eps=0.01`, `w=200`. Full-suite selection uses one preset, so changing
the preset does not double the 750 runs.

### `campaign`: smaller grids

For a smaller selection, omit `--suite full` and specify tasks, objects and pairs:

```bash
python3 -m c3plus.utils campaign \
  --tasks open_table shelf_gap --objects T_shape hammer \
  --pairs smoke --obstacle_cost exponential --seed 42 \
  --out results/targeted_check --dry-run
```

Targeted campaigns accept `--pairs smoke|diagonal|all`, `--obstacle_cost both`
for a two-cost comparison, or `--manifest PATH` for a saved job list. The full
suite accepts one cost preset and no manual task/object/pair selectors.
`--output-root` remains an alias for `--out`. The older `run_launch` and
`run_launch_simple_s2` commands select goal G2, three goal yaws and both costs;
they are debugging presets, separate from the full five-object suite.

### `visualize`: mesh and EE previews

Save a PNG of the configured object in simulation space, with candidate
end-effector positions overlaid:

```bash
python3 -m c3plus.utils visualize \
  --task open_table --object T_shape --start 2 \
  --view top --hide-robot --ee-samples --frames \
  --output results/previews/T_shape_ee.png
```

The command exits after saving; no native build, browser or server is needed.
`--ee-samples` defaults to 64 candidates and also saves
`<PNG stem>.ee_samples.json`; use `--ee-samples 100 --sample-seed 42` to change
the preview count. Repeating the command overwrites those preview files.

Use `--view scene|object|top`, `--geometry visual|collision`, or
`--pose goal --goal 2` to inspect another view or placement. `--model PATH`
accepts SDF, URDF or MJCF, and `--mesh PATH` accepts raw OBJ visuals; they are
mutually exclusive. Raw OBJ sampling requires `--position X Y Z`;
`--mesh-scale 0.001` converts millimetres to metres, and `--sample-height Z`
selects the sampling height in world metres. `--dry-run` prints paths and
placement without writing.

Preview samples use an independent seeded generator and do not establish native
IK feasibility, runtime filtering or pushing success. Check
[object profiles](examples/sampling_c3/shared_parameters/experiments.yaml) and
[models](examples/sampling_c3/urdf/) for geometry and sampler settings.

</details>

## Results

Generated data belongs under `results/`, which is already ignored by Git. The
`smoke/`, `table2/`, `object_generalization/`, and `archive/` output categories
start empty; no experiment results are bundled. Each completed trial keeps one
`*_result.json` and one MP4. The JSON embeds recordings, resolved
configurations, diagnostics, hashes, and completion status. Intermediates are
removed only after successful validation; failures preserve them for recovery.
The full campaign keeps `manifest.json`, its driver log and summary at the
campaign root. Trials are stored under
`<out>/<task>/<object>/<cost>_<task>_<object>_sXXgYY_seed42/`. Each trial directory
contains `<run_id>_result.json` and `<run_id>.mp4`; the manifest records the
shared pose-source identity and exact per-trial selection.

The JSON separates physical execution, planning updates and recorder snapshots.
One execution step begins when the simulator first applies commands from a
selected policy; both C3 and reposition count. Unselected plans, OSC ticks and
debug snapshots are not execution steps. For N applied policies, aligned
trajectories contain N+1 observed boundaries, including the terminal state.

| Recorded field | Meaning |
| --- | --- |
| `dynamic.time`, `execution.sim_time` | Simulation time at the execution boundaries |
| `execution.wall_time` | Relative monotonic wall time at those boundaries |
| `execution.step_wall_time`, `dynamic.compute_time` | Execution wall durations; the latter is a C3+ compatibility alias, not optimizer solve timing |
| `hyperparameters.control_dt` | `null` because physical policy durations vary |
| `execution.step_budget` | Explicit `--steps` budget, or `null` when unlimited |
| `recording.snapshot_dynamic` | Original asynchronous snapshot arrays |

Execution frequency is `N / (wall_time[-1] - wall_time[0])`, using valid positive
intervals. Common snapshot success requires simultaneous position error below
0.05 m and orientation error below 0.1 rad. `evaluation.ever_success` and
`evaluation.final_success` describe retained snapshots; native thresholds are
recorded separately and native completion remains unknown.

Compatible external Table-II evaluation uses the first qualifying execution
endpoint for success, steps and elapsed simulation time. Failures use the
configured step budget when present and the full recorded simulation span.
Frequency uses all recorded execution wall intervals. Historical files without
proven physical alignment cannot supply execution-based steps, time or frequency.
The implementation is in [c3plus/evaluation/](c3plus/evaluation/).

Use `python3 -m c3plus.utils eval --help` for saved-run postprocessing and export options.
This invokes the existing postprocessor; `postprocess` remains an alias.
It does not launch a simulation or add a benchmark comparison table.

| Saved-data command | Purpose |
| --- | --- |
| `eval --export-only --run-dir PATH --scene TASK --run-id RUN_ID` | Update the JSON from recorded data and settings, with validation before replacement |
| `compact --run-dir PATH` | Validate the JSON/video package and remove redundant intermediates; safe to repeat |
| `render --result PATH_TO_JSON --out PATH_TO_MP4` | Replay a modern recorded trajectory; matching repository assets are required |
| `cost-figure --run-dir PATH --scene TASK --obstacle_cost exponential` | Legacy diagnostic plot; cannot directly read compacted semantics-version-4 results |

These commands use the `python3 -m c3plus.utils` prefix and cannot recover
execution telemetry that was never recorded. `oim.run_eval` is an external
evaluator and is not installed by this Docker setup.

## Reproducibility

Keep the result JSON and video together with the recorded commit, image identity,
campaign plan and resource settings. Results retain configuration, asset and
binary hashes, package versions, and available source-state provenance. Seed 42
does not make asynchronous execution identical across machine loads. Restore
recorded source changes in a separate checkout and match the saved configuration
and model assets before comparing results. A binary hash identifies a binary;
it does not prove which source built it.

The Ubuntu base digest, Drake, Bazel, libbot2 and Python versions are pinned;
apt repositories are not snapshot-pinned and Python wheel hashes are not locked.
The recipe hash cannot guarantee byte-identical rebuilds. Retain the built image
or its registry digest when sharing an exact environment.

## Troubleshooting

For a busy port, use **HOST:** `MESHCAT_PORT=7001 ./docker/shell.sh`. For missing
binaries or native flags, rebuild with **CONTAINER:** `python3 -m c3plus.utils build`.
Use a fresh output directory for each run; campaign `--resume` requires its
matching manifest.

| Symptom | Action |
| --- | --- |
| Docker unavailable or wrong platform | Start a local Linux amd64 engine; enable WSL2 integration on Windows. Check `docker info` on the host. |
| Python imports fail in an old image | Rebuild with `./docker/shell.sh --build`, then repeat the container build and checks. |
| Compiler killed or resource limit rejected | Check Docker's allocated RAM and reduce build jobs or adjust the resource settings above. |
| Bazel cache permission denied | Rebuild/reopen the image; current startup handles a root cache mismatch. Nonroot users need their default UID/GID-specific volume or a correctly owned custom volume. |

For an existing root shell in an older image, repair only the cache root with
`chown --no-dereference 0:0 /home/dairlib/.cache/bazel`, then rebuild. Do not
recursively change existing cache ownership.

The existing exporter can reject small raw-quaternion norm drift with
`Invalid exact native execution state`. A prior 10-second startup packaged
successfully, while a 20-second check encountered this limit. Longer-run
packaging is not fully verified; failed runs retain their raw evidence. This
structural cleanup preserves that guard and the evaluation definitions.

## Repository Structure

| Path | Purpose |
| --- | --- |
| [c3plus/](c3plus/) | Configuration, experiments, runtime, recording, evaluation and visualization implementations |
| [c3plus/utils/](c3plus/utils/) | CLI, trial plans and campaign orchestration: `python3 -m c3plus.utils COMMAND` |
| [tests/](tests/) | Tests mirroring source responsibilities and integration checks |
| [docker/](docker/) | Toolchain recipe, dependency pins and environment launcher |
| [examples/poses/](examples/poses/) | Exact local comparison poses and pinned source provenance |
| [examples/sampling_c3/](examples/sampling_c3/README.md) | Native controllers, simulator, shared configuration, models and method citation |
| [build_support/](build_support/) | Gurobi dependency rules and build overrides |
| [build_support/bazel](build_support/bazel) | Explicit native build wrapper using the ROS-free build override |
| `.build/` | Ignored Bazel shortcuts; actual build data stays in the cache |
| `results/` | Ignored generated JSON/video, previews and campaign evidence |

The CLI dispatcher, trial plans, run orchestration, campaigns and saved manifests
live in `c3plus/utils/`. Commands dispatch directly to the existing implementation
modules; no separate adapter directory is required.

Bazel's root module/package markers and hidden configuration files remain in
place for automatic discovery. `build_support/noros.bazelrc` holds the ROS-free
override; the Python build command invokes `build_support/bazel`. Use that wrapper
for direct native Bazel commands as well. Native compatibility demos and other
dairlib components remain separate
from the normal benchmark workflow. Within `c3plus/`, `configs/` owns task/object
resolution, `runtime/` owns native process lifecycle, `recording/` captures
telemetry, `evaluation/` validates and packages saved data, and `visualization/`
owns previews and replay. Setup, usage and result-format documentation live in
this README.
