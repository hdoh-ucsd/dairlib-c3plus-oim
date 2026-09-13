# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot in six matched OIM scenes.
Includes the Drake simulator, controllers, scene assets, and the
[experiment tools](tools/experiments/). Run `python3 -m tools.experiments` from
the repository root to build, run, evaluate, and replay exponential/ReLU experiments.

[Setup](#setup) · [Obstacle costs](#obstacle-costs) · [Running](#running) ·
[Results](#results) · [Repository](#repository)

## Setup

Linux x86-64 or WSL2 with Docker integration, Git, Bash, and a running Docker
daemon. No ROS or GPU is required. Initial builds need network access and
substantial disk space. Use the same commit on every machine.

```bash
git clone --branch integration/c3plus-oim-consolidated https://github.com/hdoh-ucsd/dairlib-c3plus-oim.git
cd dairlib-c3plus-oim
git rev-parse HEAD
./docker/shell.sh --build-only
./docker/shell.sh
```

Inside the container, from the repository root:

```bash
python3 -m tools.experiments build
python3 -m pip check
python3 -m tools.experiments check --require-binaries --check-scenes
python3 -m unittest tools.experiments.test_workflow -v
```

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

| Scene | Object | Environment |
| --- | --- | --- |
| `open_task` | T-shape | Open table |
| `single_obstacle` | T-shape | One box |
| `shelf_gap` | T-shape | Shelf gap |
| `ycb_clutter` | T-shape | YCB clutter |
| `icra_sign` | C glyph | ICRA sign |
| `slalom` | T-shape | Slalom obstacles |

### One experiment

```bash
python3 -m tools.experiments run \
  --scene open_task --obstacle_cost exponential --start 1 --goal 1 \
  --seed 42 --cap 600 --out results/reproduce/one
```

| Option | Meaning |
| --- | --- |
| `--scene` | One of the six scenes above; `scenes` lists them |
| `--start`, `--goal` | Native start/goal indices, each from 1 to 5 |
| `--obstacle_cost` | `exponential` or `relu` |
| `--cap` | Recorder wall-time budget in seconds, default 600; startup and packaging add time |
| `--out` | New output directory; existing runs are never overwritten |
| `--dry-run` | Print the resolved demo, goals, and settings without writing files or starting processes |

Recording, metrics, plots, and MP4 rendering are automatic. `RUN_COMPLETE` means
packaging finished; check `*_result.json` for task success and
`runtime_status.json` for process failures.

### Campaigns

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

Trials run serially under a checkout lock. Each campaign saves its plan and
driver log, with runs under `<output-root>/<obstacle_cost>/<scene>/sMMgNN/`.
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
| `state_trace.jsonl`, `steps_raw.jsonl`, `*.log` | Recorded states, controller messages, and process/packaging logs |
| `*_metrics.csv`, `*_result.json`, `*_manifest.yaml` | Metrics, task result, and evaluation manifest |
| `*_eval_metrics.png`, `*_cost_diagnostics.png`, `*.mp4` | Diagnostic plots and sampled video replay |
| `RUN_COMPLETE` | Recording and every packaging phase completed |

Save the commit, image ID, environment-check report, campaign plan, and complete
run folders when sharing results. Runtime status includes binary hashes and
Python package versions. Keep CPU/memory settings consistent; seed 42 alone cannot
ensure identical outcomes across asynchronous execution and machine load.
New launcher and packaging logs begin with `[COMMAND]` and `[CWD]`; the effective
environment is saved in `runtime_status.json`.

Historical videos are organized in `results/archive/`. Open `index.csv` to find
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
snapshots, so their success times can differ. Cost plots are diagnostics.
See [evaluation format](docs/evaluation_json.md) for schema and comparison limits.

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

`__main__.py` dispatches commands; `catalog.py` shares scene metadata and goal
lookup. `run_experiment.py` packages one run; `run_grid_campaign.py` manages
campaigns. `launch_run.sh` launches and cleans up native processes, while
`record_metrics.py` records LCM data. The remaining tools implement `check`,
`postprocess`, `render`, and `cost-figure`; direct Python script calls also work.

The simulator and controller goal come from
`examples/sampling_c3/<demo>/parameters/`; models live under
`examples/sampling_c3/urdf/`. Diagonal start/goal pairs use `...tM` demos and
other pairs use `...sMgN`. Plans report position in meters and yaw in radians.

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
Changing a physical scene requires updating its native model/configuration
alongside the YAML; the YAML alone does not change the simulation or controller goal.

</details>

`check` emits JSON and exits nonzero on failure; resolve failures before running
campaigns. `--runtime-only` skips optional asset-generation imports. Workflow
tests need no simulations or native build; Drake-dependent renderer tests skip
when its runtime is absent. See [verification](docs/reproduction_verification.md)
for completed checks and remaining Docker validation.
