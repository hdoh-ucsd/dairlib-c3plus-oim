# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot in six matched OIM scenes.
Includes the Drake simulator, controllers, scene assets, and the
[experiment tools](tools/experiments/). Run `python3 -m tools.experiments` from
the repository root to build, run, evaluate, and replay exponential/ReLU experiments.

[Setup](#setup) · [Running](#running) · [Results](#results) · [Repository](#repository)

## Setup

Linux x86-64 or WSL2 with Docker integration, Git, Bash, and a running Docker
daemon. No ROS or GPU is required. Initial builds need network access and
substantial disk space. Use the same commit on every machine.

From the host terminal:

```bash
git clone --branch integration/c3plus-oim-consolidated https://github.com/hdoh-ucsd/dairlib-c3plus-oim.git
cd dairlib-c3plus-oim
git rev-parse HEAD
./docker/shell.sh
```

`docker/shell.sh` selects the toolchain image, builds it if missing, and opens
Bash in a new container. Image selection is automatic. Use `--build-only` when
you want to build the image separately. The image provides the toolchain and
Python dependencies; compile the native simulator and controllers inside the
container before running experiments.

Inside the container, from the repository root:

```bash
python3 -m tools.experiments build --jobs 4
python3 -m pip check
python3 -m tools.experiments check --require-binaries --check-scenes
python3 -m unittest tools.experiments.test_workflow tools.experiments.test_object_runs tools.experiments.test_run_artifacts -v
```

`python3 -m tools.experiments build` uses Bazel to compile the simulator, OSC
controller, and sampling controller. Build on first setup, after pulling C++
changes, or when the current container cannot access previous build outputs.
`Build franka_sim from this checkout first` means the runner cannot find an
executable simulator under `bazel-bin/examples/sampling_c3/`; run the build
command in that same container and checkout. Python and YAML changes are read
from the mounted checkout on the next launch.

The image uses Ubuntu 24.04, Drake 1.51.1, Bazel 8.4.0, and
[pinned Python dependencies](docker/requirements.txt). The first native build
compiles Drake; subsequent builds reuse the cache. The launcher defaults to up
to 24 CPUs and 24 GiB RAM. For a smaller host, set
`export DAIRLIB_CPUS=4 DAIRLIB_MEM=12g` before launching, or limit build jobs with
`python3 -m tools.experiments build --jobs 4`.

Both solver archives under `docker/` are required. See [Docker setup](docker/README.md)
for solver requirements, image selection, and resource settings. The checkout is
mounted at `/home/dairlib/dairlib`; results persist on the host and Bazel uses a
separate cache volume. Keep a trial's processes in one container for LCM.
Meshcat is exposed at `http://localhost:7000`.
If port 7000 is occupied, open the container with
`MESHCAT_PORT=7001 ./docker/shell.sh` and use `http://localhost:7001` for Meshcat.

To open another shell in an already running container, run these on the host,
replacing `CONTAINER_NAME` with its name from the list:

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
docker exec -it CONTAINER_NAME bash
```

<details>
<summary>Bazel cache permission error</summary>

Bazel requires its cache root to be owned by the user running it, even for root.
A new Docker volume can inherit UID 1000 from the image while the container runs
as UID 0, producing `FATAL: mkdir('/home/dairlib/.cache/bazel'): Permission denied`.
The updated Docker entrypoint corrects this ownership for root containers without
changing cache contents. A nonroot ownership mismatch reports which UID is needed.
To load the updated entrypoint, rebuild the image and open a new container with
`./docker/shell.sh --build` from the host.

For this mismatch in an existing **root shell using an older image**, run:

```bash
chown --no-dereference 0:0 /home/dairlib/.cache/bazel && \
python3 -m tools.experiments build --jobs 4
```

Keep the cache volume to reuse compiled dependencies. Nonroot shells should use
the launcher's default UID/GID-specific volume or a `DAIRLIB_CACHE_VOLUME` owned
by their container user.

</details>

## Running

Run commands from the repository root inside the Docker environment described in
[Setup](#setup). Replace `/path/to/...` placeholders with your chosen paths.
Use `python3 -m tools.experiments COMMAND --help` for the full option list.

| Command | Purpose |
| --- | --- |
| `scenes` | List scene names and available start/goal indices |
| `run` | Push one object, or run a list of objects serially |
| `visualize_mesh` | Save a mesh PNG, optionally with candidate end-effector (EE) positions |
| `campaign` | Run a scene/start/goal grid or a saved manifest |
| `run_launch`, `run_launch_simple_s2` | Run the fixed-goal orientation campaigns |

### `run`: one or more objects

Run one trial with an explicit object, start, goal, and orientation:

```bash
python3 -m tools.experiments run \
  --scene open_task --objects T_block \
  --start 2 --goal 2 --goal-yaw-degrees 0 \
  --obstacle_cost exponential --seed 42 --cap 600 \
  --out "/path/to/run_directory"
```

| Option | Meaning |
| --- | --- |
| `--scene NAME` | Required scene; supported combinations are listed below |
| `--objects NAME [NAME ...]` | Object selection; omit to use the scene's default object |
| `--start`, `--goal` | Independent position indices from 1 to 5; both default to 1 |
| `--goal-yaw-degrees` | Absolute goal yaw: 90, 0, or −90 degrees; omit to retain the indexed goal orientation |
| `--obstacle_cost` | `exponential` (default) or `relu`; preset semantics are described below |
| `--seed` | 42 is the only supported experiment seed |
| `--cap` | Recorder wall-time budget per trial, default 600 seconds; startup and packaging add time |
| `--out PATH` | Required output directory; existing individual run folders are refused |
| `--max-frames` | Maximum video frames, default 1200 |
| `--port` | Trial LCM port, default 18001 |
| `--dry-run` | Print the resolved plan without writing files or launching processes |

Start positions, goal positions, and orientations are separate selections in
[experiments.yaml](examples/sampling_c3/shared_parameters/experiments.yaml).
Changing goal yaw preserves its indexed position; each start retains its initial
orientation and robot joint pose. Add `--dry-run` to inspect the combination.
For a short startup/recording check, use `--cap 20 --max-frames 80`.

#### `--scene` and `--objects`

| Scene | Environment | Default object | Accepted `--objects` |
| --- | --- | --- | --- |
| `open_task` | Open table | T_block | `T_block`, `sugar_box`, `power_drill`, `hammer`, `banana` |
| `single_obstacle` | One box | T_block | `T_block` |
| `shelf_gap` | Shelf gap | T_block | `T_block` |
| `ycb_clutter` | YCB clutter | T_block | `T_block` |
| `icra_sign` | ICRA sign | Cblock | Omit `--objects` to use Cblock |
| `slalom` | Slalom obstacles | T_block | `T_block` |

Pass several names to run separate trials with one movable object at a time:

```bash
python3 -m tools.experiments run \
  --scene open_task --objects T_block sugar_box power_drill hammer banana \
  --start 2 --goal 2 --goal-yaw-degrees 0 \
  --obstacle_cost exponential --seed 42 --cap 600 \
  --out "/path/to/batch_directory"
```

Trials run in the listed order with the same indexed start, goal, yaw, cost, seed,
and per-trial cap. One selected object writes directly to `--out`; multiple
objects write to `<out>/<object>/`. Explicit `T_block` preserves its existing
two-box geometry and perimeter sampler. The fixed sugar box in `ycb_clutter`
is an obstacle, separate from the movable `sugar_box` selection.

The CLI and [native controller](examples/sampling_c3/franka_sampling_c3_controller.cc)
restrict imported objects to `open_task`. Native obstacle contacts still use
built-in T/C/I/R/A footprints; loading a mesh or changing evaluation geometry does
not supply its footprint to those contacts. Other scenes require that integration
and validation. Previewing an object in a scene does not establish pushing support.

<details>
<summary>Imported-object models and verified support</summary>

The imported models use a common 0.1 kg benchmark mass, uniform-density mesh
inertia, and convex collision approximations: sugar box 1 piece, drill 6,
hammer 8, banana 4. These are modelling assumptions, not measured masses.
The native sampler uses a horizontal mesh section at EE world Z = −0.012 m
and checks the full triangle surface and controller collision pieces.
See [physics_models.json](examples/sampling_c3/urdf/objects/physics_models.json)
for the physical and evaluation models, and
[mesh_import.json](examples/sampling_c3/urdf/objects/mesh_import.json) for geometry repairs.

On 2026-09-14, 20-second checks at S2/G2, yaw 0°, seed 42, and exponential cost
recorded motion and packaged JSON/video without runtime failures for all four
imported objects on `open_task` and T_block on `single_obstacle`.
All 20 imported-object/obstacle-scene selections were rejected by the CLI;
the compiled controller independently rejected all four imported objects with
a `single_obstacle` configuration. These checks establish startup and recording
support, not task success or full-run reliability.

</details>

#### `--obstacle_cost`

| Value | Ranking behavior |
| --- | --- |
| `exponential` | Preserved baseline preset; exponential ranking currently suppressed by `lcs_contact` |
| `relu` | Active footprint-aware ReLU ranking, `eps=0.01`, `w=200` |

Both presets use `lcs_contact` obstacle handling. The `exponential` name replaces
the former `baseline` name; exponential cost plots are diagnostic reconstructions.
The runner clears inherited `SAMPLING_C3_*` variables and saves the effective cost
and scene settings in the result's `runtime_status`.
The generic `campaign` command also accepts `--obstacle_cost both`.

#### Recording, outputs, and retries

The terminal reports progress every 10 recorded snapshots:

```text
[sugar_box] step=0010 sim=0.36s wall=1.2s pos_err=0.842m yaw_err=4.2deg within_goal=no
```

`sim` is simulation time; `wall` includes the wait for simulation startup.
Errors use the latest object pose. `within_goal` requires both errors below
5 cm and 0.1 rad (about 5.73°), and can return to `no` after first success.
Recording ends at the cap or after five additional wall seconds following first
success; this settling interval can extend past the cap. Every snapshot is saved.

JSON export, video rendering, and cleanup are automatic. Each completed trial
keeps one `*_result.json` and one MP4. Resolved configurations, recordings,
diagnostics, hashes, and completion status are described under [Results](#results).

The runner checks every selected destination before starting a batch. `run` has
no `--resume`: after interruption, select only missing objects or choose a fresh
output path. For example, to finish a batch with only banana remaining, use
`--objects banana --out "/path/to/batch_directory/banana"`.
Launch, recording, or packaging errors stop the batch. A packaged trial with
`success: false` can continue to the next object; inspect `runtime_status.failures`
for native process failures. `--dry-run` does not check existing output compatibility.

### `visualize_mesh`: mesh and EE previews

This command saves a PNG and exits using the Docker image's Python dependencies;
it needs no native build, browser, or port. The default scene view includes the
object, arm, tool, table, platform, and obstacles.

```bash
# Default T_block at start S2, with coordinate axes.
python3 -m tools.experiments visualize_mesh \
  --scene open_task --start 2 --frames --output "/path/to/T_block.png"

# Cblock at goal G2 with absolute yaw +90 degrees.
python3 -m tools.experiments visualize_mesh \
  --scene icra_sign --pose goal --goal 2 --goal-yaw-degrees 90 \
  --view object --frames --output "/path/to/Cblock_goal.png"
```

| Option | Meaning |
| --- | --- |
| `--scene` | Scene and default object; default `open_task` |
| `--start`, `--goal`, `--pose` | Indexed placement; indices default to 1 and pose to `start` |
| `--goal-yaw-degrees` | Goal yaw override; requires `--pose goal` |
| `--model PATH` | SDF, URDF, or MJCF XML with one movable root |
| `--mesh PATH` | Raw OBJ visual geometry, retaining its local origin; mutually exclusive with `--model` |
| `--mesh-scale` | Uniform OBJ scale, default 1; `0.001` converts millimetres to metres |
| `--position X Y Z` | Override world position in metres |
| `--rpy-degrees R P Y` | Override world roll, pitch, and yaw; cannot accompany `--goal-yaw-degrees` |
| `--view` | `scene` (default), `object`, or `top` |
| `--geometry` | `visual` (default) or `collision`; raw OBJ previews have only visual geometry |
| `--frames`, `--hide-robot` | Show coordinate axes; hide the arm/tool while preserving scene placement |
| `--width`, `--height` | Image size, default 1280 × 960 |
| `--output PATH` | PNG path; defaults to a generated name under `results/previews/` |
| `--ee-samples [COUNT]` | Overlay EE candidates and save their JSON; 64 when supplied without a count |
| `--sample-seed` | Preview sampling seed, default 42; requires `--ee-samples` |
| `--sample-height Z` | Raw OBJ EE centre height in world metres, default −0.012; requires `--mesh` and `--ee-samples` |
| `--dry-run` | Print resolved files, pose, and output paths without generating images or samples |

Model/mesh overrides inherit the scene's selected placement unless a pose override
is provided. Output directories are created automatically, and repeated commands
overwrite the selected PNG and any companion sample JSON. Open them from the host
or IDE through the mounted checkout. Preview options do not change experiment settings.

#### `--ee-samples`: candidate end-effector positions

Inspect the configured T_block sampler from above:

```bash
python3 -m tools.experiments visualize_mesh --scene open_task --start 2 \
  --view top --hide-robot --ee-samples --output "/path/to/T_block_ee.png"
```

Use `--scene icra_sign` for the configured Cblock sampler. Blue spheres show EE
centres with a 5.55 mm tip radius. `--ee-samples 100 --sample-seed 7` changes only
the preview count and seed. The companion file is `<PNG stem>.ee_samples.json`.

To preview all four imported OBJ meshes and their EE candidates:

```bash
for object in sugar_box power_drill hammer banana; do
  python3 -m tools.experiments visualize_mesh --scene open_task --start 2 \
    --mesh "examples/sampling_c3/urdf/objects/${object}_centered.obj" \
    --position 0.366 0.431 -0.029 --view top --hide-robot --ee-samples \
    --output "/path/to/previews/${object}_ee.png"
done
```

Raw OBJ sampling requires an explicit `--position`. These centred meshes use
metres, with local underside Z = 0 placed on the table at world Z = −0.029 m.
The mesh-section preview samples at EE Z = −0.012 m (17 mm above the table),
offsets candidates outward by 35 mm, and requires at least 27 mm centre clearance
from the full triangle surface. It also checks tip radius, table clearance, and
workspace limits. `--sample-height` changes the section height.

Configured T_block/Cblock previews require their scene's configured model when
using `--ee-samples`. To inspect a different OBJ without candidates, use
`--mesh /path/to/object.obj` and omit `--ee-samples`.
Candidates use an independent seeded generator; they are not logged controller
selections. Runtime buffers, IK, obstacle/path filtering, and goal-stop logic are
excluded. Native imported-object sampling additionally checks controller collision
pieces, so the preview alone does not establish reachability or pushing success.

<details>
<summary>Model sources and EE sample JSON</summary>

The [object profiles](examples/sampling_c3/shared_parameters/experiments.yaml)
select simulation models, controller models, and sampling settings.
In an SDF, `<box><size>` specifies dimensions in metres and `<pose>` places each part.

| Object | Simulation geometry | Configured EE sampler |
| --- | --- | --- |
| T_block | [Two boxes](examples/sampling_c3/urdf/push_t_oimscale_m01.sdf) | [Perimeter sampling](examples/sampling_c3/shared_parameters/profiles/t_shape/sampling_params.yaml) of controller collision boxes |
| Cblock | [Three boxes](examples/sampling_c3/urdf/push_c_glyph.sdf) | [Mesh-normal settings](examples/sampling_c3/shared_parameters/profiles/icra_sign/sampling_params.yaml), using the [sampling mesh](examples/sampling_c3/urdf/c_glyph_base/c_glyph_base.obj) |

The hammer's imported mesh retains two pinched edges. Its exterior check uses the
closed, oriented surface and records this topology in the raw-mesh sample JSON.

| Sample JSON field | Meaning |
| --- | --- |
| `points_world`, `points_object` | Matching EE centres `[x, y, z]` in metres, in world and object-local coordinates |
| `count`, `seed` | Accepted preview count and random seed |
| `sampling_height_world_m` | World Z used for sampling |
| `projection_clearance_m`, `clearance_reference` | Clearance threshold, measured from the EE surface for T_block and centre for Cblock/raw meshes |
| `stats` | Attempted draws, accepted candidates, and rejection counts |
| `sources`, `runtime_omissions` | Source model/settings references and excluded runtime behavior; raw previews also record the mesh hash |
| `native_fresh_samples` | Configured T_block/Cblock controller per-call counts; separate from the requested preview count |

</details>

### `campaign`, `run_launch`, `run_launch_simple_s2`: repeat trials

Campaigns run serially and use each scene's default object. They do not accept
`--objects`; select imported-object batches with `run` above.

| Command | Trials | Selection |
| --- | --- | --- |
| `run_launch` | 180 | Six scenes × starts 1–5 × goal G2 × yaw 90°/0°/−90° × both costs |
| `run_launch_simple_s2` | 36 | The same selection, restricted to start S2 |
| `campaign` | User-selected | Scene/start/goal grid, or exact jobs from `--manifest` |

Choose one named campaign; `--output-root` overrides its default location:

```bash
python3 -m tools.experiments run_launch \
  --output-root "/path/to/full_run_directory" --dry-run

python3 -m tools.experiments run_launch_simple_s2 \
  --output-root "/path/to/s2_run_directory" --dry-run
```

Named campaigns default to `results/reproduce/run_launch/` and
`results/reproduce/run_launch_simple_s2/`, seed 42, and a 600-second recorder cap
per trial. Each trial resets to its selected start and uses the same goal in the
controller, recorder, evaluation, and replay. Both costs retain the preset
semantics described for `run --obstacle_cost` above.

Plan the full 300-run grid; remove `--dry-run` to execute any of these plans:

```bash
python3 -m tools.experiments campaign \
  --scenes open_task single_obstacle shelf_gap ycb_clutter icra_sign slalom \
  --obstacle_cost both --pairs all --seed 42 \
  --output-root "/path/to/output_directory" --dry-run
```

For a 12-trial startup/packaging check, change `--pairs all` to
`--pairs smoke --cap 15` in this command. Short checks do not measure success rates.

| Option / control | Meaning |
| --- | --- |
| `--scenes NAME [...]` | Generic grid scenes; defaults to `single_obstacle icra_sign` |
| `--obstacle_cost` | Generic grid costs: `exponential`, `relu`, or `both` (default) |
| `--pairs` | Generic grid: `smoke` (default, 1 pair), `diagonal` (5), or `all` (25 per scene/cost) |
| `--manifest PATH` | Exact job list; cannot accompany `--scenes`, `--obstacle_cost`, or `--pairs` |
| `--seed`, `--cap` | Seed 42; override per-trial wall caps, including manifest caps |
| `--output-root PATH` | Campaign output directory; required for generic `campaign` |
| `--port-base` | LCM ports start at this value + 1; default base 19000 |
| `--dry-run` | Print the plan without writing files or checking existing-output compatibility |
| `--resume` | Require a matching plan, skip completed JSON/video packages, and refuse partial run folders; legacy `RUN_COMPLETE` markers remain supported |
| `STOP_AFTER_CURRENT` | Create this file in the output root to stop after packaging the current trial; remove before resuming |

All trials use the checkout lock. Each campaign saves `campaign_plan.json`, its
driver log, and `summary.csv`; the summary records completion, native failures,
success, timing, errors, and video paths. Generic runs use
`<output-root>/<obstacle_cost>/<scene>/sMMgNN/`. Named campaigns append
`_yaw_p090`, `_yaw_000`, or `_yaw_m090`, with costs consecutive per scene/start/yaw.

If a saved output root contains a different campaign plan, choose a fresh path.
`--resume` does not bypass that mismatch or restart a partial trial. Keep the same
checkout path, options, and selected native settings when resuming; changing
selections, caps, ports, or older configuration layouts requires a new output root.

<details>
<summary>Historical manifest and default grid</summary>

```bash
python3 -m tools.experiments campaign \
  --manifest tools/experiments/campaign_seed42.json \
  --output-root "/path/to/manifest_output_directory" --dry-run
```

This manifest preserves 25 ordered `single_obstacle` jobs: 13 exponential and
12 ReLU, ending at exponential S3/G3. Rounded evaluation/replay goals must match
the native goal within 0.0001 m and 0.0001 rad; `controller_goal` is recorded
separately. `--cap` overrides the saved caps. Without selection flags, generic
`campaign` plans four smoke jobs across `single_obstacle`, `icra_sign`, and both
costs. Old `variant`/`baseline` plans require a new output root; no aliases are provided.

</details>

## Results

| Output | Contents |
| --- | --- |
| `*_result.json` | Summary, trajectories, metrics, raw recordings, resolved configurations, runtime status, diagnostic excerpts, hashes, and package completion |
| `*.mp4` | Validated video replay |

CSV/JSONL files, YAML snapshots, text logs, plots, caches, and the old completion
marker are intermediate artifacts. Once the JSON and video pass validation,
the runner consolidates their essential information and removes those files.
It writes and reads back the consolidated JSON before deleting intermediates.
Validation failures leave the original files untouched. If deletion is interrupted,
the verified JSON and remaining files allow cleanup to resume.
Unknown files and snapshot meshes without identical repository copies prevent
cleanup. Videos are never embedded as base64 in the JSON.

To compact an existing completed run without rerunning simulation or rendering:

```bash
python3 -m tools.experiments compact \
  --run-dir results/latest_json/banana
```

This performs the same validation and cleanup. A repeated invocation is safe;
it can also finish interrupted cleanup. Campaign plans, driver logs, and summary
tables remain at the campaign root, separate from individual experiment folders.

`*_result.json` uses `schema`, `run`, `hyperparameters`, `static`, and `dynamic`
for metadata and trajectories, with explicit `evaluation` and `native_controller`
sections. `run.algorithm` identifies C3+. Object identity comes from saved
experiment settings; the T-shaped object is named `T_block`.

| Field | Meaning |
| --- | --- |
| `evaluation.ever_success` | Any retained snapshot simultaneously meets position error `< 0.05 m` and orientation error `< 0.1 rad` |
| `evaluation.final_success` | The final retained snapshot meets both common evaluation thresholds |
| `evaluation.first_success_t` | First qualifying entry in the actual `dynamic.time` sequence, or `null` |
| `evaluation.thresholds` | Recorded common evaluation thresholds, intentionally separate from native C3+ thresholds |
| `native_controller.success_thresholds` | Values read from the saved `config/goal.yaml` (currently `0.02 m` and `0.1 rad`) |
| `native_controller.success` | `null`: no native controller completion signal is recorded; evaluation success does not imply native completion |
| `evaluation.weights`, `evaluation.costs` | Saved offline diagnostic weights and component/total series; these do not describe the native optimization objective |

Complete native settings are preserved as exact text, parsed data, and hashes in
`provenance.configuration.files`. Other native metadata lives in `provenance.c3plus`;
its `configurations` mapping is retained when no verified duplicate snapshot exists.
`hyperparameters` contains compact scalar settings such as `horizon`, `n_admm`,
and `obstacle_cost`, so native configuration blobs do not enter evaluation grouping.
Diagnostic weights live in `evaluation.weights` and remain `null` when unavailable;
current defaults are never substituted. Legacy `hyperparameters.costs` is relocated
on export; `dynamic.evaluation_costs` and `dynamic.evaluation_total` remain aliases.
Existing top-level success/error fields remain available to campaign
summaries: `success` means common evaluation ever-success, `t_success` is its
first retained-snapshot time, and legacy `first_success_t` is the recorder's
object-message event time, which can differ. `schema.legacy_fields` documents
these distinctions; `schema.semantics_version` is `3` while the trajectory
layout version remains unchanged.

The schema describes the projections needed to compare algorithms:

- `n_snapshots = M` recorded states define `n_recorded_intervals = M - 1`
  observed intervals. Legacy `n_control_steps` and `steps_run` retain these
  respective counts; neither counts executed controls. No initial state is
  invented; use `dynamic.time`, which preserves variable sample times.
- Execution time is `dynamic.time[-1] - dynamic.time[0]`. With at least one
  interval and strictly increasing finite times, `hyperparameters.control_dt` is this span divided by
  `steps_run`, with source `mean_observed_state_interval`; otherwise it is `null`.
  This mean is neither the physics timestep nor a planner prediction period.
- Object and tip velocities are finite-difference estimates using actual time
  intervals and wrapped yaw. Initial estimates and estimates at duplicate or
  decreasing timestamps are `null`. The source states are asynchronous snapshots.
- `qpos` projects robot joints plus object `[x, y, yaw, z]`; `qvel` combines
  recorded joint velocities and estimated object velocities. These arrays are
  not the full Drake state. `object_pose_3d` retains the recorded quaternion/XYZ.
- Per-step planner compute timing is unavailable, so `dynamic.compute_time` is
  omitted; legacy all-null arrays are accepted. Observed state frequency is not
  optimizer frequency. Applied transition controls and measured contact forces
  remain `null`. Published joint efforts are retained separately as
  `dynamic.robot_joint_effort`. Missing diagnostic values also use JSON `null`.

New runs export and compact this automatically. To update an existing modern
result's export semantics without simulation or rendering, use its saved data:

```bash
python3 -m tools.experiments postprocess --export-only \
  --run-dir /path/to/saved_run \
  --scene open_task \
  --run-id RUN_ID
```

Only the result JSON is replaced, after validation and atomic read-back checks.
Compacted runs use embedded recordings and settings; uncompressed runs use
their saved CSV, raw samples, and `evaluation_scene_config.yaml`. Supply
`--scene-config /path/to/saved_config.yaml` if that saved file is elsewhere.
This command does not upgrade historical `archive` metadata-only bundles.

With the compatible evaluator installed in your separate OIM environment, evaluate
saved runs without writing reports:

```bash
python -m oim.run_eval \
  --runs-dir /absolute/path/to/dairlib/results/latest_json \
  --group-by object --no-save
```

The evaluator prefers recorded timestamps for execution time and retains fixed
`steps_run * control_dt` as the legacy fallback for runs such as MPPI. Missing
compute timing produces `mean_frequency_hz = None`, displayed as `-`. Grouping,
filtering, and ablation accept scalar settings; the method stays `c3plus` unless
an ablation is explicitly selected. Success at the final pose and the existing
aggregate duration penalty for failed runs remain evaluator conventions, separate from the raw
per-trial time span and C3+ ever-success field.

Save the recorded repository revision, image ID, campaign plan, JSON, and video
when sharing results. Embedded runtime status includes binary hashes,
configuration hashes, and Python package versions. New runs capture a HEAD-based
binary patch covering staged and unstaged tracked changes, plus nonignored
untracked file contents and modes, before launch. The temporary
`config/source_state.json` is embedded in the final JSON; `provenance.source_state`
records its hash, base commit, scope, and JSON pointer. This captures source
bytes, not ignored build products or Git staging distinctions; executable hashes
identify binaries separately. Older runs without a captured patch explicitly
report it as unavailable rather than using today's checkout.
Keep CPU/memory settings
consistent; seed 42 alone cannot ensure identical outcomes across asynchronous
execution and machine load.
Temporary launcher and packaging logs begin with `[COMMAND]` and `[CWD]`;
commands, diagnostic excerpts, and the effective environment are retained in the JSON.

Historical results remain outside Git in the local `results/archive/` directory;
they are not included in a fresh clone. Its `index.json` contains the run catalog,
cleanup report, and shared provenance. The original shared documents are retained
under `source_files`.

The archive has 125 run folders, each containing one JSON and one MP4. The 94
folders under `runs/<scene>/` preserve their original `*_result.json` filenames
and summary fields; a JSON stem may differ from the video's. The 31 folders under
`media/<scene>/` have a `<video_stem>_result.json` containing recovered metadata.
These files add an `archive` section with format `historical-run-archive/v1`:
recovered command metadata, video identity and hashes, original source-file
contents, and cleanup status. Saved commands, logs, metrics, and configurations
are embedded under `archive.source_files` with their exact text, sizes, and
hashes. Legacy summaries retain their original schema, and missing metrics or
trajectories are not invented.

Folder and MP4 names retain this pattern:

```text
<scene>__<cost>__sMMgNN__seed<N>__cap<seconds>s__<batch>[__trial<N>]
```

Unrecorded values remain `unknown`. `historical-baseline` preserves an old preset
whose logs do not establish exponential ranking. Recovered command metadata
distinguishes inferred settings from observed partial commands; it does not claim
that complete original commands were saved. Keep each result JSON with its video
when sharing or archiving runs.

<details>
<summary>Replay or inspect a saved result</summary>

Modern result JSONs contain the trajectories and metric series needed for later
analysis. For a compacted modern run, render directly from its JSON; historical
archive metadata alone does not provide a replayable trajectory:

```bash
python3 -m tools.experiments render \
  --result results/latest_json/banana/exponential_open_task_banana_s02g02_yaw_000_seed42_result.json \
  --out results/banana_replay.mp4 --max-frames 1200
```

The renderer reads the goal and model references from the result; repository
assets must still be available and match any recorded asset hashes. Older runs
without these hashes require the original repository revision. Explicit model
overrides remain supported.
`render --trace` still accepts legacy JSONL traces. Cost plots are now optional:

```bash
python3 -m tools.experiments cost-figure \
  --run-dir results/latest_json/banana \
  --scene open_task --obstacle_cost exponential
```

These commands use the Python/rendering environment and do not start controllers.
An explicitly requested diagnostic plot is an additional artifact in that folder.

Videos default to 10 fps and at most 1200 frames, so playback may not be real
time. Rendering cleans automatically created frame/mesh directories; explicit
`--frames-dir` and `--assets-tmp` directories are retained. Managed packaging
uses the run's `tmp/` directory.

The recorder evaluates high-rate object messages; postprocessing uses retained
debug-message snapshots, so their success times can differ. Use
`evaluation.final_success` to assess the ending pose. Cost plots are diagnostics;
unavailable cost blocks are JSON `null` (NaN in intermediate CSV), and
`evaluation.costs.total` sums only the available blocks.
See [postprocess_run.py](tools/experiments/postprocess_run.py)
for the scene schema, output fields, and metric definitions.

</details>

## Repository

| Path | Purpose |
| --- | --- |
| [docker/](docker/README.md) | Toolchain, dependency pins, launcher, and container tests |
| [tools/experiments/](tools/experiments/) | CLI, recording, evaluation, replay, and workflow tests |
| [tools/experiments/scene_configs/](tools/experiments/scene_configs/) | Six YAMLs defining evaluation geometry and planner mappings |
| [tools/workspace/gurobi/](tools/workspace/gurobi/) | Gurobi dependency integration required by Bazel |
| [tools/bazel](tools/bazel) | Bazelisk wrapper selecting ROS-free build settings by default |
| [examples/sampling_c3/](examples/sampling_c3/README.md) | Native demos, controller parameters, models, and upstream method documentation |
| `results/` | Generated runs and figures; keep new outputs out of source control |

Keep source, configurations, and required models in Git; archive experiment
outputs separately. `results/` and root `results_*` directories are ignored.
Untracking existing results preserves local files and leaves Git history intact.

<details>
<summary>Scene configuration and tool responsibilities</summary>

`__main__.py` dispatches commands; `catalog.py` composes and snapshots native
configurations. `run_experiment.py` packages one run; `run_grid_campaign.py` manages
campaigns. `launch_run.sh` launches and cleans up native processes, while
`record_metrics.py` records LCM data. The remaining tools implement `check`,
`postprocess`, `compact`, `render`, `visualize_mesh`, and `cost-figure`; direct Python script calls also work.

The shared [experiment catalogue](examples/sampling_c3/shared_parameters/experiments.yaml)
stores each configurable value once:

| Section | Contents |
| --- | --- |
| `start_positions`, `goal_positions` | Separate named XY coordinates, in metres |
| `orientations` | Reused quaternions in `[w, x, y, z]` order |
| `robot_joint_presets` | Initial robot joint configurations, in radians |
| `defaults`, `object_profiles` | Common controller/simulator/goal settings, object heights, and default indexed references |
| `scenes` | Scene selection and the few scene-specific overrides |

Keep coordinates and quaternion values in their respective tables; profiles
and scene overrides select them by name. Controller profiles and scene geometry
remain separate shared YAMLs; models live under `examples/sampling_c3/urdf/`.
At launch, the tools write `controller.yaml`, `simulation.yaml`, and `goal.yaml`
inside the run's `config/` directory, then pass the controller file to all native
processes through `--controller_params`. The source catalogue is saved as
`config/source_experiments.yaml`; selected shared YAMLs live under
`config/repository/`. After packaging, these settings are embedded in the result
and the temporary directory is removed. The `...tM` and `...sMgN` demo names remain run identifiers;
their former 150 directories are unnecessary. Three legacy `push_t_bt010_*`
entrypoints remain for `bash_run.sh`.

Plans report the starting pose as `[w, x, y, z, X, Y, Z]`, goal position in metres,
and goal yaw in radians. Configuration hashes prevent resuming a campaign with
changed selected settings; use a new output root for older campaign plans.

Scene YAMLs supply evaluation geometry and planner overrides. Their optional
`planner` block references `obstacles.polygons` by zero-based index:

```yaml
# shelf_gap: obstacle slots 0–2 use the large shelf; 3–4 use the small one.
planner:
  box_polygons: [0, 0, 0, 1, 1]
```

`box_polygons` assigns rectangles to consecutive native slots;
`polygon_overrides` maps explicit slots to polygon indices (YCB: `{2: 2, 3: 3}`).
ICRA also sets `object_footprint: c_glyph` and `obstacle_top_z: 0.046`.
Unassigned slots, including the robot base, retain native disc geometry.
`catalog.planner_environment()` validates and serializes these settings.
Changing physical scene geometry requires updating its native model/configuration
alongside the evaluator YAML. Start and goal changes belong in the experiment catalogue.

</details>

`check` emits JSON and exits nonzero on failure; resolve failures before running
campaigns. `--runtime-only` skips optional asset-generation imports. Workflow
tests need no simulations or native build; Drake-dependent renderer tests skip
when its runtime is absent. The configuration refactor was checked against all
150 indexed configurations, with native typed YAML loading for representative
snapshots and a Bazel dependency check. Run the setup checks in your container
before starting a campaign; these checks do not establish experiment outcomes.
