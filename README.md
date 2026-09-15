# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot in six matched OIM scenes.
This checkout contains the Drake simulator, controllers, scene assets, and
[experiment CLI](c3plus/utils/__main__.py). Docker is the supported environment for
building and running experiments; host Python, Conda, ROS, and a GPU are not required.

[Quick Start](#quick-start) · [Docker Environment](#docker-environment) ·
[Experiments](#experiments) · [Results](#results) ·
[Repository Structure](#repository-structure)

In command templates, replace quoted placeholder names with your values,
keeping the quotes. Paths may be absolute or relative to
the repository root. For example, `"output_directory"` could become
`"results/my_run"`; `"max_time"` could become `"300"` (the default cap).

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
git clone --branch main https://github.com/hdoh-ucsd/dairlib-c3plus-oim.git
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

**7. CONTAINER — launch the [experiments](#experiments).**

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
docker exec -it "container_name" bash
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

After the Quick Start setup, run commands **inside the container, from the
repository root**. This single launcher selects the full experiment suite:

```bash
python3 -m c3plus.utils campaign \
  --suite full --seed 42 --cap 300 --resume \
  --out "output_directory"
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
the whole selection before trial 1.

The default cap is **300 simulation seconds per trial** for both `run` and
`campaign`; omit `--cap` to use it, or supply `--cap "max_time"` to override it.
An explicit per-run cap in a custom campaign manifest is retained unless
overridden by `--cap`.

At this cap, the total simulated-time budgets are:

| Selection | Trials | Simulation time if every trial reaches the cap |
| --- | --- | --- |
| All `open_table` objects and start/goal pairs | 125 | **10 hours 25 minutes** |
| Full suite, one obstacle cost | 750 | **62 hours 30 minutes** (2 days 14 hours 30 minutes) |

Real elapsed time depends on simulation speed. For the full suite, estimate
`62.5 / simulation_speed + overhead_hours`, where `simulation_speed` is simulated
seconds per wall-clock second. At 1× speed, allowing 30–60 wall seconds per trial
for startup and packaging gives **68.75–75 hours**; at 0.5× speed it gives
**131.25–137.5 hours**. These are conditional estimates, not measured full-suite
times or upper bounds. Early success can shorten the campaign. Initial builds
are excluded. A 600-second simulation cap doubles the simulated budget to 125 hours.

| Placeholder | Value |
| --- | --- |
| `task` | One task from the table above |
| `object` | One object from the table above; `T_block` is also accepted for `T_shape` |
| `start`, `goal` | Independent pose IDs from 1 to 5 |
| `max_time` | Positive whole seconds of elapsed simulation time **per trial/object**; default `300` |
| `output_directory` | Destination for the trial, batch, campaign, or evaluation output |
| `input_directory` | Existing run, batch, or campaign directory containing recorded results |
| `file_path` | File path, including the extension required by the command |

`--cap` starts with the first `FRANKA_STATE_SIMULATION` timestamp and stops
recording when the received simulation clock has advanced by that many seconds.
Startup and waiting for the first state do not count. A paused simulation pauses
the cap; it is not a wall-clock timeout. A simulator exit is reported as a failure
instead of waiting indefinitely. Delivery and shutdown can add a small simulation
overshoot. Reaching the goal stops the simulation immediately, without a settling
delay. Postprocessing and rendering add real time.

To **preview the plan**, append `--dry-run` to the same command. This validates
poses and assets and prints the full manifest plus completed, pending,
invalid/partial and total counts, without writing run data or launching trials.
To **resume**, repeat the command with the same settings, checkout state, and
output directory. Full-suite manifests compare the Git commit and tracked-edit
hash, including README edits.
`--resume` also works for a fresh campaign. Valid completed JSON/video packages
are skipped; partial or corrupt trials are reported and preserved. A different
selection, cap, configuration, or checkout state requires a new output directory.
Old campaigns with wall-time caps also require a new output directory; their
budgets are not silently reinterpreted. `--resume`
does not retry partial runs or overwrite their recordings.

Create `output_directory/STOP_AFTER_CURRENT` to stop after the current trial
is packaged. Remove that file and repeat the same command to continue.

**Validation limit:** all 750 configurations and 300 start/goal endpoints pass
static preflight. The entire 750-run campaign has not been validated to completion;
launch or packaging failures preserve the partial run for inspection.

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

**The default manipulated object for `icra_sign` is `T_block`**, as for the other
tasks. Its canonical CLI/result name is `T_shape`; `--object T_block` selects
the same T-shaped model, and omitting `--object` selects it automatically.
`Cblock` is not a supported experiment object. The fixed ICRA-sign glyphs are
scene obstacles and do not select the manipulated object.

`open_table` maps to the native scene `open_task`. Native assets retain their names.
Tasks select the environment; the object selection supplies its model, sampling
geometry, footprint and support height. The launcher checks all selected
combinations and rejects incompatible poses before running anything.

<details>
<summary>Optional CLI commands: individual runs, smaller grids and previews</summary>

The full campaign above covers the complete benchmark. The following commands
are available for targeted checks and visualization.

### `run`: object selection and obstacle costs

Run one object at your selected task and pose pair:

```bash
python3 -m c3plus.utils run \
  --task "task" --object "object" \
  --start "start" --goal "goal" \
  --obstacle_cost exponential --seed 42 --cap "max_time" \
  --out "output_directory"
```

Add `--dry-run` to inspect the resolved selection. Use a fresh output directory
for each invocation; `run` has no resume flag. Use
`python3 -m c3plus.utils COMMAND --help` for each command's complete options.

| Option | Meaning |
| --- | --- |
| `--task "task"` | Required task; `--scene` remains an alias |
| `--object "object"` / `--objects "object_1" "object_2" ...` | One object (default `T_shape`) or a serial multi-object batch; mutually exclusive |
| `--start`, `--goal` | Independent pose IDs 1–5, default 1 |
| `--goal-yaw-degrees` | Optional absolute goal yaw: 90, 0 or −90 degrees; omit to preserve the source pose |
| `--obstacle_cost` | `exponential` (default) or `relu` |
| `--cap "max_time"` | Elapsed simulation-time budget per trial, default 300; starts at the first simulator state |
| `--steps "step_budget"` | Applied-policy budget including reposition; unlimited when omitted |
| `--out "output_directory"` | Fresh trial directory, or parent directory for a multi-object batch |
| `--max-frames`, `--port` | Maximum video frames (1200) and trial LCM port (18001) |

Run all five objects serially on one task using the same pose pair and cap:

```bash
python3 -m c3plus.utils run \
  --task "task" --objects T_shape sugar_box power_drill hammer banana \
  --start "start" --goal "goal" \
  --obstacle_cost exponential --seed 42 --cap "max_time" \
  --out "output_directory"
```

All five objects are supported across the six tasks, subject to pose preflight.
To select a subset, replace the five names with your chosen objects, separated
by spaces. One selected object writes directly to `--out`; multiple objects
write to `output_directory/object/`. All destinations are checked before the
batch starts, and a launch or packaging error stops it. Progress messages appear
every ten recorder snapshots; their `step` counter is separate from physical execution steps.
After control begins, the simulator checks the actual object pose on every
physics step, including steps holding the same control command. It stops at the
first state with position error below **0.05 m** and wrapped yaw error below
**0.1 rad** simultaneously, marks the result as successful, and packages the run.

In simulation, the controller keeps planning when contact tips an object; roll
or pitch above 0.5 radians no longer triggers the topple shutdown. Normal time
and step limits, success stopping, and workspace checks still apply. Rebuild the
native targets after updating this behavior. Hardware retains its topple stop.

Before the cap changed to simulation time, the batch pattern above produced
complete JSON/MP4 packages for all five objects
on each task with **start 2, goal 2, seed 42, exponential cost, and no goal-yaw
override**, using these caps:

| `task` | Historical wall-time cap (seconds per object) |
| --- | --- |
| `open_table` | `600` |
| `icra_sign` | `100` |
| `shelf_gap` | `50` |
| `single_obstacle` | `10` |
| `slalom` | `10` |
| `ycb_clutter` | `10` |

These historical checks verify recording and packaging. Their wall-time caps
are not equivalent to the current simulation-time `--cap`. Goal-reaching success
is reported per trial in the JSON.

For the recorded `open_table` S2→G2, seed-42 batch, successful trials reached
their goals after approximately 104, 124, and 182 seconds of execution wall time.
The slowest successful launch completed in approximately 192 seconds before
postprocessing. These are historical wall times, not simulation-time budget
recommendations. The 300-simulation-second default has not been validated across
all 25 pose pairs or all six tasks.

Both cost presets preserve `lcs_contact` handling: `exponential` ranking is
currently suppressed in that mode; `relu` uses the existing footprint-aware
ranking with `eps=0.01`, `w=200`. Full-suite selection uses one preset, so changing
the preset does not double the 750 runs.

### `campaign`: smaller grids

For a smaller selection, omit `--suite full` and specify tasks, objects and pairs:

```bash
python3 -m c3plus.utils campaign \
  --tasks "task_1" "task_2" --objects "object_1" "object_2" \
  --pairs smoke --obstacle_cost exponential --seed 42 --cap "max_time" \
  --out "output_directory" --dry-run
```

Provide one or more task and object names; remove `--dry-run` to execute.
Targeted campaigns accept `--pairs smoke|diagonal|all`, `--obstacle_cost both`
for a two-cost comparison, or `--manifest "file_path"` for a saved job list.
The full suite accepts one cost preset and no manual task/object/pair selectors.
`--output-root` remains an alias for `--out`. The older `run_launch` and
`run_launch_simple_s2` commands select goal G2, three goal yaws and both costs;
they are debugging presets, separate from the full five-object suite.

### `visualize`: mesh and EE previews

Save a PNG of the configured object in simulation space, with candidate
end-effector positions overlaid:

```bash
python3 -m c3plus.utils visualize \
  --task "task" --object "object" --start "start" \
  --view top --hide-robot --ee-samples --frames \
  --output "file_path"
```

Use a `.png` file path, such as `results/previews/object_ee.png`. The command
exits after saving; no native build, browser or server is needed.
`--ee-samples` defaults to 64 candidates and also saves
`png_stem.ee_samples.json`; use `--ee-samples 100 --sample-seed 42` to change
the preview count. Repeating the command overwrites those preview files.

Use `--view scene|object|top`, `--geometry visual|collision`, or
`--pose goal --goal "goal"` to inspect another view or placement.
`--model "file_path"` accepts SDF, URDF or MJCF, and `--mesh "file_path"`
accepts raw OBJ visuals; they are mutually exclusive. Raw OBJ sampling requires
`--position X Y Z`; `--mesh-scale 0.001` converts millimetres to metres, and `--sample-height Z`
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
`output_directory/task/object/cost_task_object_sXXgYY_seed42/`. Each trial directory
contains `run_id_result.json` and `run_id.mp4`; the manifest records the
shared pose-source identity and exact per-trial selection.

The JSON separates physical execution, planning updates and recorder snapshots.
One execution step begins when the simulator first applies commands from a
selected policy; both C3 and reposition count. Unselected plans, OSC ticks and
debug snapshots are not execution steps. For N applied policies, aligned
trajectories contain N+1 observed boundaries, including the terminal state.

Raw native quaternions are preserved in `dynamic.object_pose_3d` and
`recording.execution_native`. When integration drift exceeds the exporter's
legacy unit tolerance, it normalizes a temporary copy only to derive planar yaw.
Previously accepted yaw values, snapshot diagnostics and execution timing remain
unchanged; zero or nonfinite quaternions still fail validation.

| Recorded field | Meaning |
| --- | --- |
| `dynamic.time`, `execution.sim_time` | Simulation time at the execution boundaries |
| `runtime_status.simulation_cap_seconds`, `provenance.c3plus.simulation_cap_seconds` | Requested elapsed simulation-time budget; older runs retain their recorded `wall_cap_seconds` |
| `execution.wall_time` | Relative monotonic wall time at those boundaries |
| `execution.step_wall_time`, `dynamic.compute_time` | Execution wall durations; the latter is a C3+ compatibility alias, not optimizer solve timing |
| `hyperparameters.control_dt` | Mean simulation-time duration of actually applied policies: `(execution.sim_time[-1] - execution.sim_time[0]) / execution.n_steps_executed` |
| `execution.step_budget` | Explicit `--steps` budget, or `null` when unlimited |
| `recording.snapshot_dynamic` | Original asynchronous snapshot arrays |

`control_dt_source` is `mean_physical_execution_interval`. This mean includes
C3, reposition, and the terminal policy interval; actual durations vary. It is
not a fixed planner/physics timestep, and execution frequency continues to use
wall-clock intervals only. A mean cannot be supplied without recorded intervals.
Older execution-aligned JSONs with `control_dt: null` remain readable;
`postprocess --export-only` fills this field from their validated saved boundaries.

C3+ exports omit empty reference-only metadata such as MPPI temperature and
consensus settings, optimizer selectors, and planar limit-surface parameters.
Only those designated null fields are removed; recorded values are retained.
Nulls that mean unlimited steps, unavailable measurements or unknown success
remain, along with all original array positions and saved provenance.

Execution frequency is `N / (wall_time[-1] - wall_time[0])`, using valid positive
intervals. A native goal stop records `execution.terminal_reason: "goal_reached"`,
`success: true`, and the exact reached simulation time in `t_success` and
`first_success_t`. The reached pose is retained as the final native state in
`dynamic.object_pose`. A snapshot and video frame of that exact terminal state
are also saved, including when success occurs before the first debug snapshot.
`evaluation.ever_success` and `evaluation.final_success` describe the retained
snapshots, including this terminal snapshot for new goal-stopped runs.

The aggregate evaluator uses the first qualifying execution
endpoint for success, steps and elapsed simulation time. Failures use the
configured step budget when present and the full recorded simulation span.
Frequency uses all recorded execution wall intervals. Historical files without
proven physical alignment cannot supply execution-based steps, time or frequency.
The implementation is in [c3plus/evaluation/](c3plus/evaluation/).

Evaluate completed trials recursively, grouped by task and method:

```bash
python3 -m c3plus.utils eval \
  --runs-dir "input_directory" \
  --out-dir "output_directory"
```

Set `input_directory` to the output of a completed multi-object batch or
campaign. The evaluator writes `input_directory_name.json` and
`input_directory_name.txt` under `output_directory`, using the input
directory's final path component as the filename. The output may be a subfolder
of the input, such as its `eval/` directory. Omit `--out-dir` to print without
writing files. Use any batch or campaign directory with `--runs-dir`;
`--format text|markdown|latex` selects the table format, and `--diagnostics`
prints compact per-trial metrics. No experiments are launched or source results
modified.

The table reports trial count, success rate, position error `eps_d` (meters),
orientation error `eps_o` (radians), their successful-only means `eps_d^s` and
`eps_o^s`, execution steps, execution frequency, and elapsed simulation execution
time. Both errors use the first execution endpoint that simultaneously satisfies
the recorded position and orientation tolerances, or the final endpoint if the
trial fails. Group values are arithmetic per-trial means. The initial state is
excluded, and missing measurements appear as `-`.

Aggregate JSON schema `c3plus-aggregate-evaluation-v2` uses these endpoint errors.
The previous trajectory averages remain separately in JSON as
`trajectory_mean_position_error` and `trajectory_mean_orientation_error`, with
their `_success` means. These averages cover executed endpoints through first
success, or all endpoints on failure, so even successful trials can have large
trajectory averages. The legacy JSON field `theta` retains its trajectory-mean
meaning; `eps_o_success` is the successful-only endpoint orientation error.
Original run JSON files are unchanged by evaluation.

Failed trials without a configured execution-step budget have unavailable
censored steps. Numeric means use available values; the saved summary records
their counts. “Averaged over” lists experiment settings that vary within a group,
such as object, start, goal, or seed.

`postprocess` remains the separate command for updating one saved run's JSON.
Use `eval --help` or `postprocess --help` for their respective options.

| Saved-data command | Purpose |
| --- | --- |
| `postprocess --export-only --run-dir "input_directory" --scene "task" --run-id "run_id"` | Update one saved run's JSON from recorded data and settings, with validation before replacement |
| `compact --run-dir "input_directory"` | Validate one run's JSON/video package and remove redundant intermediates; safe to repeat |
| `render --result "json_file_path" --out "video_file_path"` | Replay a modern recorded trajectory to MP4; matching repository assets are required |
| `cost-figure --run-dir "input_directory" --scene "task" --obstacle_cost exponential` | Legacy diagnostic plot; cannot directly read compacted semantics-version-4 results |

These commands use the `python3 -m c3plus.utils` prefix and cannot recover
execution telemetry that was never recorded. `oim.run_eval` is an external
evaluator and is not installed by this Docker setup.

The external evaluator requires a numeric `hyperparameters.control_dt`.
New execution-aligned exports supply it; older copies with `null` must be
refreshed with `postprocess --export-only` before sharing them. Use a directory
containing only individual run JSONs for `oim.run_eval --runs-dir`: its loader
also attempts to evaluate summary, manifest and configuration JSONs if present.
Its time calculation uses `len(dynamic.object_pose) - 1`, so the recorded mean
interval yields the complete simulated execution span. Its final-state success
and trajectory-mean error definitions differ from this repository's first-success
endpoint metrics; JSON compatibility does not make those scoring rules identical.

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
