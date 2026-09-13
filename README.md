# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot in six matched OIM scenes.
Includes the Drake simulator, controllers, scene assets, and the
[experiment tools](tools/experiments/). Run `python3 -m tools.experiments` from
the repository root to build, run, evaluate, and replay exponential/ReLU experiments.

[Setup](#setup) · [Obstacle costs](#obstacle-costs) · [Running](#running) ·
[Multi-object runs](#multi-object-runs) · [Mesh and EE previews](#preview-object-geometry) ·
[Results](#results) · [Repository](#repository)

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
you want to build the image separately.

Inside the container, from the repository root:

```bash
python3 -m tools.experiments build
python3 -m pip check
python3 -m tools.experiments check --require-binaries --check-scenes
python3 -m unittest tools.experiments.test_workflow tools.experiments.test_object_runs -v
```

`python3 -m tools.experiments build` uses Bazel to compile the simulator, OSC
controller, and sampling controller. Run it after pulling C++ changes, including
the new configuration-path and goal-yaw support. Python and YAML changes are
read from the mounted checkout on the next launch.

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

## Obstacle costs

| `--obstacle_cost` | Ranking |
| --- | --- |
| `exponential` | Preserved baseline preset; exponential ranking currently suppressed |
| `relu` | Footprint-aware ReLU cost, `eps=0.01`, `w=200` |

Both use `lcs_contact` obstacle handling and seed 42, the only supported seed.
The `exponential` name replaces the former `baseline` name. In the current
controller, `lcs_contact` suppresses its exponential ranking term; ReLU ranking
remains active. Exponential cost plots are diagnostic reconstructions.
The runner clears inherited `SAMPLING_C3_*` settings and records the effective
cost and scene settings in `runtime_status.json`.

## Running

Use `python3 -m tools.experiments COMMAND --help` for all options.

| Scene | Default object | Environment |
| --- | --- | --- |
| `open_task` | Tblock | Open table |
| `single_obstacle` | Tblock | One box |
| `shelf_gap` | Tblock | Shelf gap |
| `ycb_clutter` | Tblock | YCB clutter |
| `icra_sign` | Cblock | ICRA sign |
| `slalom` | Tblock | Slalom obstacles |

### One experiment

```bash
python3 -m tools.experiments run \
  --scene open_task --obstacle_cost exponential --start 1 --goal 1 \
  --seed 42 --cap 600 --out results/reproduce/one
```

| Option | Meaning |
| --- | --- |
| `--scene` | One of the six scenes above; `scenes` lists them |
| `--start`, `--goal` | Independent start-position and goal-position indices, each from 1 to 5 |
| `--goal-yaw-degrees` | Optional absolute world yaw: 90, 0, or −90 degrees; preserves the indexed goal position |
| `--obstacle_cost` | `exponential` or `relu` |
| `--cap` | Recorder wall-time budget in seconds, default 600; startup and packaging add time |
| `--out` | New output directory; existing runs are never overwritten |
| `--objects` | One or more of `sugar_box`, `power_drill`, `hammer`, `banana`; supported on `open_task` |
| `--dry-run` | Print the resolved demo, goals, and settings without writing files or starting processes |

Recording, metrics, plots, and MP4 rendering are automatic. `RUN_COMPLETE` means
packaging finished; check `*_result.json` for task success and
`runtime_status.json` for process failures. Recording ends at the wall-time
budget or after five additional wall seconds following first success; that
settling interval can extend past the budget.

Start positions, goal positions, and orientations are defined separately in
[experiments.yaml](examples/sampling_c3/shared_parameters/experiments.yaml).
Choose their combination in the command; there is no YAML file per start/goal
pair. For example, start S2, goal position G2, and goal yaw −90°:

```bash
python3 -m tools.experiments run \
  --scene single_obstacle --obstacle_cost relu \
  --start 2 --goal 2 --goal-yaw-degrees -90 \
  --out results/reproduce/s2_g2_minus90 --dry-run
```

Omitting `--goal-yaw-degrees` uses the goal's original benchmark orientation.
Each start retains its configured initial orientation and robot joint pose.
An actual run saves the resolved native configuration and its shared YAML
dependencies in the run's `config/` directory.

### Multi-object runs

`run --objects` runs a batch of separate pushing trials, with one object in the
scene at a time. The supported objects are `sugar_box`, `power_drill`, `hammer`,
and `banana`, currently on the open table (`--scene open_task`).

Inside Docker, rebuild after updating the native code, then launch all four at
start S2, goal position G2, and absolute goal yaw 0°:

```bash
python3 -m tools.experiments build
python3 -m tools.experiments run --scene open_task \
  --objects sugar_box power_drill hammer banana \
  --start 2 --goal 2 --goal-yaw-degrees 0 --obstacle_cost exponential \
  --cap 600 --out results/open_table_objects
```

Trials run serially in the order listed. All receive the same start, goal,
orientation, cost preset, and seed 42. `--cap 600` applies to each trial;
startup and packaging add time. Append `--dry-run` to inspect the plans without
creating files or launching processes; existing output folders are checked only
when launching. Change the cost with `--obstacle_cost relu`
or the goal yaw with `--goal-yaw-degrees 90` or `-90`.

Each object gets its own complete run folder:

```text
results/open_table_objects/
  sugar_box/
  power_drill/
  hammer/
  banana/
```

Each folder contains `runtime_status.json`, the saved `config/`, logs, traces,
`*_result.json`, metrics, plots, MP4, and `RUN_COMPLETE` after packaging. Filenames
include the object, for example
`exponential_open_task_sugar_box_s02g02_yaw_000_seed42_result.json`.

To run one object, use one name; outputs go directly into `--out`:

```bash
python3 -m tools.experiments run --scene open_task --objects banana \
  --start 2 --goal 2 --goal-yaw-degrees 0 --obstacle_cost exponential \
  --cap 600 --out results/banana_trial2
```

Choose a fresh output folder when repeating a trial. Before launching a batch,
the runner refuses any selected object folder that already exists. `run` has no
`--resume`: after an interruption, select only objects whose folders do not yet
exist, or use a new `--out` to retry. Launch, recording, or packaging errors stop
the batch; inspect the affected folder's logs before retrying.
If only banana remains, use `--objects banana --out results/open_table_objects/banana`.
A packaged trial with `success: false` still allows the next object to run.
Omitting `--objects` retains the scene's original Tblock/Cblock selection.
The `campaign`, `run_launch`, and `run_launch_simple_s2` commands retain their
scene-selected objects; use `run --objects` for these four imported objects.

The imported models use a common **0.1 kg benchmark mass**, uniform-density
mesh inertia, and convex collision approximations (sugar box 1 piece, drill 6,
hammer 8, banana 4). These are modelling assumptions, not measured masses.
The native sampler uses the horizontal mesh section at an EE height of −0.012 m
and checks the full triangle surface and collision pieces. Model details are
in [physics_models.json](examples/sampling_c3/urdf/objects/physics_models.json).
Each run saves the selected models, meshes, settings, asset hashes, object
identity, and evaluation footprint. Geometric and configuration checks do not
establish pushing success; the first run is an experiment.

### Preview object geometry

Inside Docker, from the repository root, save a PNG of an object and its scene.
The command uses the image's Python dependencies and exits after saving; no
native Bazel build, browser, or port is needed.

```bash
# Tblock at start S2.
python3 -m tools.experiments visualize_mesh --scene open_task --start 2 \
  --frames --output results/previews/Tblock.png

# Cblock in the ICRA sign scene.
python3 -m tools.experiments visualize_mesh --scene icra_sign --start 2 \
  --view object --frames --output results/previews/Cblock.png

# Tblock at goal G2 with absolute world yaw +90 degrees.
python3 -m tools.experiments visualize_mesh --scene open_task \
  --pose goal --goal 2 --goal-yaw-degrees 90 --frames
```

Open the saved image in your IDE or on the host through the mounted checkout.
Without `--output`, images go to `results/previews/`. Parent directories are
created automatically; an existing output image is overwritten.
The [image tool](tools/experiments/visualize_mesh.py) includes the configured
object, robot, tool, table, platform, and scene obstacles in its default scene view.

Tblock and Cblock are the default scene objects. Sugar box, power drill, hammer,
and banana can be selected for open-table experiments with `run --objects` and
inspected as meshes below. The sugar box already
in `ycb_clutter` is a separate, fixed box obstacle.

To inspect their design, open the simulation models below: `<box><size>` gives
dimensions in metres, and `<pose>` places each part relative to its link.
The [object profiles](examples/sampling_c3/shared_parameters/experiments.yaml)
select the simulation model, controller model, and sampling settings.

| Object | Simulation geometry | EE sampling settings |
| --- | --- | --- |
| Tblock | [Two boxes](examples/sampling_c3/urdf/push_t_oimscale_m01.sdf) | [Perimeter sampler](examples/sampling_c3/shared_parameters/profiles/t_shape/sampling_params.yaml) |
| Cblock | [Three boxes](examples/sampling_c3/urdf/push_c_glyph.sdf) | [Mesh-normal sampler](examples/sampling_c3/shared_parameters/profiles/icra_sign/sampling_params.yaml) |

Preview candidate end-effector (EE) positions from above, hiding the robot to
expose the object and samples:

```bash
python3 -m tools.experiments visualize_mesh --scene open_task --start 2 \
  --view top --hide-robot --ee-samples --output results/previews/Tblock_ee.png
python3 -m tools.experiments visualize_mesh --scene icra_sign --start 2 \
  --view top --hide-robot --ee-samples --output results/previews/Cblock_ee.png
```

These commands draw 64 candidates with seed 42. Use `--ee-samples 100
--sample-seed 7` for a different count and seed. To inspect candidates at a goal,
add `--pose goal --goal 2 --goal-yaw-degrees -90` and choose a new `--output`.
The preview seed and count affect only these images and their sample JSONs.

Blue spheres mark candidate EE centres using the physical tip radius of 5.55 mm.
Tblock samples random perimeter points from the controller's collision boxes;
Cblock uses normals from its [sampling mesh](examples/sampling_c3/urdf/c_glyph_base/c_glyph_base.obj).
Sampling applies the selected object pose, native geometric rules, and workspace
limits with an independent seeded random generator. These are candidate previews,
not logged samples or controller selections; runtime buffers, IK, obstacle/path
filtering, and goal-stop logic are excluded. Raw OBJ samples use the separate
mesh-section method described below.

Each PNG has a companion `<PNG stem>.ee_samples.json`; for example,
`results/previews/Cblock_ee.png` and `results/previews/Cblock_ee.ee_samples.json`.
Open either in your IDE. Repeating the command overwrites both files.

| Sample JSON field | Meaning |
| --- | --- |
| `points_world`, `points_object` | Matching EE centres as `[x, y, z]` in metres, in world and object-local coordinates |
| `count`, `seed` | Accepted preview count and random seed |
| `sampling_height_world_m` | World Z used when sampling |
| `projection_clearance_m`, `clearance_reference` | Clearance threshold and whether it measures from the EE surface (Tblock) or centre (Cblock and raw meshes) |
| `stats` | Attempted draws, accepted candidates, and rejection counts |
| `sources`, `runtime_omissions` | Source model/settings paths and runtime behavior excluded from the preview |

For configured Tblock/Cblock previews, `native_fresh_samples` records the controller's per-call counts;
`--ee-samples` controls the preview count. `--dry-run` prints the resolved
selection without generating candidates or writing the PNG or JSON.

To inspect all four imported objects on the open table (`open_task`):

```bash
for object in sugar_box power_drill hammer banana; do
  python3 -m tools.experiments visualize_mesh --scene open_task --start 2 \
    --mesh "examples/sampling_c3/urdf/objects/${object}_centered.obj" \
    --position 0.366 0.431 -0.029 --view top --hide-robot --ee-samples \
    --output "results/previews/objects/${object}_ee.png"
done
```

These meshes use metres and have their underside at local Z = 0; the table
surface is at world Z = −0.029 m. Raw mesh sampling requires an explicit
`--position`. It samples the horizontal mesh section at the EE centre height,
offsets candidates along outward normals by 35 mm, and checks that each centre
is outside the full triangle surface with at least 27 mm clearance. The 5.55 mm
tip radius, table clearance, and scene workspace limits are also checked.
The default EE height is −0.012 m (17 mm above the table); change it with
`--sample-height Z`. The JSON records the checks and source mesh hash.

The imported geometry and repairs are recorded in
[mesh_import.json](examples/sampling_c3/urdf/objects/mesh_import.json).
The hammer retains two pinched edges; the exterior check uses its closed,
oriented surface and records this topology in the sample JSON.
These previews validate geometry; native sampling also checks the controller's
convex collision pieces. Neither preview establishes controller reachability or
pushing success.

To preview another raw OBJ without EE samples:

```bash
python3 -m tools.experiments visualize_mesh \
  --mesh examples/sampling_c3/urdf/ycb_hulls/mustard_bottle.obj \
  --position 0.4 0 0.07 --rpy-degrees 0 0 0 --view object --frames
```

| Preview option | Meaning |
| --- | --- |
| `--output PATH` | PNG destination; defaults to `results/previews/` |
| `--view scene`, `object`, `top` | Camera view; default `scene` |
| `--geometry visual`, `collision` | Geometry to draw; default `visual` |
| `--width`, `--height` | Image dimensions in pixels; defaults 1280 × 960 |
| `--model PATH` | SDF, URDF, or MJCF XML with one movable root |
| `--mesh PATH` | Raw OBJ visual geometry, retaining its local origin |
| `--mesh-scale` | Uniform OBJ scale; `0.001` converts millimetres to metres |
| `--position X Y Z` | Override world position in metres |
| `--rpy-degrees R P Y` | Override world roll, pitch, and yaw in degrees |
| `--frames` | Show coordinate axes |
| `--hide-robot` | Hide the arm and tool in the image; preserve table, objects, obstacles, and world positions |
| `--ee-samples [COUNT]` | Draw candidate EE positions; 64 when supplied without a count, none when omitted |
| `--sample-seed` | Candidate sampling seed; default 42 |
| `--sample-height Z` | Raw OBJ EE centre height in world metres; requires `--mesh` and `--ee-samples` |
| `--dry-run` | Print selected files, pose, and planned image path without writing files |

Model and mesh overrides use the scene's selected start/goal placement unless
pose overrides are provided. Use `--geometry collision` with a simulation model
to inspect its collision shapes; raw OBJ previews have only visual geometry.
Preview options do not change assets or experiment
settings. Direct invocation also works: `python3 tools/experiments/visualize_mesh.py --help`.

### Campaigns

Run the fixed-goal orientation campaigns inside Docker, from the repository root:

```bash
# Rebuild once after updating to support composed configurations and goal yaw.
python3 -m tools.experiments build

# Full run: all five starts (180 trials).
python3 -m tools.experiments run_launch

# Simplified run: start 2 only (36 trials).
python3 -m tools.experiments run_launch_simple_s2
```

Choose either campaign. Both cover all six scenes, each scene's goal position G2,
absolute world-frame yaw **90°, 0°, −90°**, and both cost presets, with seed 42 and a
600-second recorder budget per trial. Each trial resets to its selected start;
the controller, recorder, evaluation, and replay use the same goal. The current
`exponential` preset retains the behavior described under [Obstacle costs](#obstacle-costs).

Outputs default to `results/reproduce/run_launch/` and
`results/reproduce/run_launch_simple_s2/`. Add `--dry-run` to print the full plan
without writing files or launching processes, or `--resume` to continue the same
campaign. `--cap`, `--port-base`, and `--output-root` can override their defaults.
Run folders use `<obstacle_cost>/<scene>/sMMg02_yaw_p090/`, `..._yaw_000/`, and
`..._yaw_m090/`; cost presets run consecutively for each scene/start/yaw.

If you see `Output root already contains a different campaign plan`, choose a
fresh output folder. Plans saved before the YAML refactor use older metadata,
even when the trial selections are unchanged. For example:

```bash
python3 -m tools.experiments run_launch_simple_s2 \
  --output-root results/reproduce/run_launch_simple_s2_v2
```

`--resume` requires a matching plan and skips completed trials; it does not
bypass a plan mismatch or restart a partial trial. Keep the same checkout path,
options, and selected native settings when resuming. `--dry-run` prints the plan;
existing output compatibility is checked when launching or resuming.

For other selections, use the generic `campaign` command below.

Check startup and packaging across all six scenes and both costs (12 short runs):

```bash
python3 -m tools.experiments campaign \
  --scenes open_task single_obstacle shelf_gap ycb_clutter icra_sign slalom \
  --obstacle_cost both --pairs smoke --seed 42 --cap 15 \
  --output-root results/reproduce/smoke
```

Plan the full 300-run comparison; remove `--dry-run` to execute:

```bash
python3 -m tools.experiments campaign \
  --scenes open_task single_obstacle shelf_gap ycb_clutter icra_sign slalom \
  --obstacle_cost both --pairs all --seed 42 \
  --output-root results/reproduce/full_grid --dry-run
```

| Option / control | Effect |
| --- | --- |
| `--pairs smoke`, `diagonal`, `all` | 1, 5, or 25 start/goal pairs per scene and cost |
| `--resume` | Skip runs with `RUN_COMPLETE`; refuse partial directories, which you must move aside before retrying |
| `STOP_AFTER_CURRENT` | Create this file in the output root to stop after packaging the active run; remove before resuming |
| `--port-base` | LCM ports begin at this value + 1; default first port is 19001 |

Trials run serially under a checkout lock. Each campaign saves its plan,
driver log, and `summary.csv`, updated from saved results after each completed
or resumed trial. The summary records completion status, process failures,
success, timing, final position/orientation errors (metres/radians), and video paths.
Generic grid runs live under `<output-root>/<obstacle_cost>/<scene>/sMMgNN/`.
Use a new output root when changing selections, caps, or ports. Short smoke
runs check dependencies and packaging; they do not measure success rates.

<details>
<summary>Historical campaign and defaults</summary>

```bash
python3 -m tools.experiments campaign \
  --manifest tools/experiments/campaign_seed42.json \
  --output-root results/reproduce/seed42 --dry-run
```

The manifest contains 25 ordered `single_obstacle` runs: 13 exponential and
12 ReLU, ending at exponential start 3 / goal 3. Remove `--dry-run` to execute.
Its rounded evaluation/replay goals and caps are preserved. Goals must match
the native controller within 0.0001 m and 0.0001 rad; they do not override it.
`controller_goal` is recorded separately.

`--cap` overrides manifest caps. A manifest cannot be combined with `--scenes`,
`--obstacle_cost`, or `--pairs`. Without selection flags, a campaign runs four
smoke jobs across `single_obstacle`, `icra_sign`, and both costs. Old
`variant`/`baseline` campaign plans need a new output root; no aliases are provided.

</details>

## Results

| Output | Contents |
| --- | --- |
| `runtime_status.json`, `evaluation_scene_config.yaml` | Provenance, effective settings, goals, process status, and saved scene configuration |
| `config/` | Resolved controller, simulator, and goal YAMLs, plus snapshots of selected shared settings |
| `state_trace.jsonl`, `steps_raw.jsonl`, `*.log` | Recorded states, controller messages, and process/packaging logs |
| `*_metrics.csv`, `*_result.json`, `*_manifest.yaml` | Metrics, task result, and evaluation manifest |
| `*_eval_metrics.png`, `*_cost_diagnostics.png`, `*.mp4` | Diagnostic plots and sampled video replay |
| `RUN_COMPLETE` | Recording and every packaging phase completed |

Save the commit, image ID, environment-check report, campaign plan, and complete
run folders when sharing results. Runtime status includes binary hashes,
configuration hashes, and Python package versions. Keep CPU/memory settings
consistent; seed 42 alone cannot ensure identical outcomes across asynchronous
execution and machine load.
New launcher and packaging logs begin with `[COMMAND]` and `[CWD]`; the effective
environment is saved in `runtime_status.json`.

Historical videos are kept outside Git in the local `results/archive/` directory;
they are not included in a fresh clone. Open `index.csv` there to find
a run, then read its `command.json` for settings recovered from logs, evidence,
and the mapping from original filenames. Full run bundles live under
`runs/<scene>/`; older videos without raw run data live under `media/<scene>/`.
Folder and MP4 names use this pattern:

```text
<scene>__<cost>__sMMgNN__seed<N>__cap<seconds>s__<batch>[__trial<N>]
```

Unrecorded values are labeled `unknown`. `historical-baseline` preserves an old
preset whose logs do not establish exponential ranking. Exact original commands
were not saved; `command.json` distinguishes recovered settings from observed
partial commands. Original logs and metrics keep their contents and identifiers.
`cleanup_report.json` records removed files and duplicate videos. Keep each video
with its complete run folder when archiving future results; use `results/reproduce/`
for new runs. That working directory was excluded from the historical cleanup.

<details>
<summary>Regenerate metrics, cost figures, or video from a saved run</summary>

For the open-task example above:

```bash
python3 -m tools.experiments postprocess \
  --run-dir results/reproduce/one \
  --scene open_task --run-id exponential_open_task_s01g01_seed42 \
  --scene-config results/reproduce/one/evaluation_scene_config.yaml \
  --demo matched_open_table_xarm6_t1
python3 -m tools.experiments cost-figure \
  --run-dir results/reproduce/one --scene open_task --obstacle_cost exponential
python3 -m tools.experiments render \
  --trace results/reproduce/one/state_trace.jsonl \
  --out results/reproduce/one/replay.mp4 \
  --object-sdf examples/sampling_c3/urdf/push_t_oimscale_m01.sdf \
  --goal 0.381 -0.4 3.1416 --max-frames 1200
```

For other scenes, use the models in [catalog.py](tools/experiments/catalog.py)
and the saved evaluation goal in `runtime_status.json`. These commands need
the Python/rendering environment but do not start the native controllers.

Videos default to 10 fps and at most 1200 frames, so playback may not be real
time. Rendering cleans automatically created frame/mesh directories; explicit
`--frames-dir` and `--assets-tmp` directories are retained. Managed packaging
uses the run's `tmp/` directory.

The recorder evaluates high-rate states; postprocessing uses controller-step
snapshots, so their success times can differ. Reported success means a sampled
state met both thresholds (5 cm position error and 0.1 rad orientation error);
check the final errors to assess the ending pose. Cost plots are diagnostics;
unavailable cost blocks are recorded as NaN, and `evaluation_total` sums only
the available blocks. See [postprocess_run.py](tools/experiments/postprocess_run.py)
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
`postprocess`, `render`, `visualize_mesh`, and `cost-figure`; direct Python script calls also work.

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
`config/repository/`. The `...tM` and `...sMgN` demo names remain run identifiers;
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
