# dairlib C3+/OIM benchmark

Sampling-based C3+ pushing with an xArm6 robot across six matched OIM scenes.
Contains the Drake simulator, controllers, scene assets and the
[experiment CLI](c3plus/utils/__main__.py). Docker is the supported environment;
no host Python, Conda, ROS or GPU is required.

## Requirements

Git, Bash, and a local Docker engine running **Linux amd64** containers (Linux,
WSL2 on Windows, or Intel macOS). Remote engines and ARM emulation are not
supported. The engine needs at least 4 GiB for the container — more to compile
Drake — plus network access and free disk for the first build.

## Quick Start

```bash
# HOST
git clone --branch main https://github.com/hdoh-ucsd/dairlib-c3plus-oim.git
cd dairlib-c3plus-oim
./docker/shell.sh
```

You are now in a container shell at the repository root, with the checkout
mounted at `/home/dairlib/dairlib`. Run everything below **inside it**.

```bash
# CONTAINER
python3 -m c3plus.utils build --jobs 4                                  # first build also compiles Drake
python3 -m c3plus.utils check --suite full --require-binaries --check-scenes
python3 -m unittest discover -s tests
```

`check` prints a JSON report and exits nonzero on failure; resolve failures
before running experiments.

## Docker environment

| Command | Purpose |
| --- | --- |
| `./docker/shell.sh` | Open the environment (builds the image if missing) |
| `./docker/shell.sh --build` | Force an image rebuild |
| `./docker/shell.sh --build-only` | Build the image without opening a shell |
| `./docker/shell.sh --help` | All launcher options |
| `docker exec -it "container_name" bash` | Extra shell into a running container |

Python and YAML edits need no image rebuild; native C++ changes require
`python3 -m c3plus.utils build`. Exiting removes the container; the checkout and
the Bazel cache volume remain. **Keep one trial's processes in the same
container** — they talk over LCM inside its network namespace.

Host environment variables: `DAIRLIB_IMAGE`, `DAIRLIB_CPUS`, `DAIRLIB_MEM`,
`DAIRLIB_BAZEL_JOBS`, `DAIRLIB_BAZEL_RAM_MB`, `DAIRLIB_CACHE_VOLUME`,
`MESHCAT_PORT`. For example:

```bash
DAIRLIB_CPUS=4 DAIRLIB_MEM=12g DAIRLIB_BAZEL_JOBS=2 ./docker/shell.sh
```

## Experiments

### Full benchmark

```bash
python3 -m c3plus.utils campaign --suite full --seed 42 --cap 300 --resume --out results/full
```

Add `--oim-out results/full/oim` to also emit OIM-contract files as trials finish.

| Selection | Values |
| --- | --- |
| Tasks | `icra_sign`, `open_table`, `shelf_gap`, `single_obstacle`, `slalom`, `ycb_clutter` |
| Objects | `T_shape` (default), `hammer`, `sugar_box`, `power_drill`, `banana` |
| Poses | Starts S1–S5 × goals G1–G5 = 25 pairs per task/object |
| Total | 6 tasks × 5 objects × 25 pairs = **750 runs** |
| Cost | `exponential` (default) or `--obstacle_cost relu` |

Append `--dry-run` to validate and print the manifest without running anything.
`--resume` skips validated complete trials and preserves partial ones. A changed
selection, cap or checkout state needs a new output directory — full-suite
manifests hash the commit and tracked edits, **including edits to this README**.
Create `output_directory/STOP_AFTER_CURRENT` to stop after the current trial.

### One trial

```bash
python3 -m c3plus.utils run \
  --task open_table --object T_shape --start 2 --goal 2 \
  --obstacle_cost exponential --seed 42 --cap 300 \
  --out results/my_run
```

| Option | Meaning |
| --- | --- |
| `--task` | Required; `--scene` is an alias |
| `--object` / `--objects A B ...` | One object, or several run serially into `out/object/` |
| `--start`, `--goal` | Pose IDs 1–5 |
| `--cap` | Elapsed **simulation** seconds per trial (default 300) |
| `--steps` | Applied-policy budget; unlimited when omitted |
| `--no-video` | Skip MP4 rendering; keep all recordings and metrics |
| `--no-record` | Skip the recorder: logs only, no result JSON, nothing to evaluate |
| `--oim-out DIR` | Also write an OIM-contract file for `oim.run_eval` into `DIR` |
| `--goal-yaw-degrees` | Absolute goal yaw 90, 0 or −90; omit to keep the pose's own |
| `--dry-run` | Print the resolved selection without running |

`--out` must be a fresh directory; `run` has no resume. A trial stops the moment
position error is below **0.05 m** and wrapped yaw error below **0.1 rad**.

### Smaller grids

Edit [campaign.yaml](campaign.yaml) to keep a selection in a file:

```yaml
tasks: [open_table, shelf_gap]
objects: [T_shape, hammer]
pairs: diagonal          # all | diagonal | smoke, or [[2, 2], [1, 3]]
obstacle_cost: exponential
cap: 300
```

```bash
python3 -m c3plus.utils campaign --out results/subset --dry-run
```

`campaign` reads that file whenever you give no selection flag. Passing any of
`--tasks`, `--objects`, `--pairs` or `--obstacle_cost` selects from the command
line instead and ignores the file, so a one-off run needs no edit:

```bash
python3 -m c3plus.utils campaign \
  --tasks open_table shelf_gap --objects T_shape hammer \
  --pairs smoke --obstacle_cost exponential --seed 42 --cap 300 \
  --out results/subset --dry-run
```

`--spec "file.yaml"` selects a different spec file, `--obstacle_cost both` runs
a two-cost comparison, and `--manifest "file.json"` supplies an exact job list.

Use `python3 -m c3plus.utils COMMAND --help` for any command's full options.

## Results

Output goes under `results/` (gitignored), as
`output_directory/task/object/cost_task_object_sXXgYY_seed42/`. Each completed
trial keeps one `*_result.json` and one MP4; a campaign also writes
`manifest.json`, a driver log and a summary at its root.

### Evaluate

```bash
python3 -m c3plus.utils eval --runs-dir results/full --out-dir results/full/eval
```

Reports trial count, success rate, position error `eps_d`, orientation error
`eps_o`, their successful-only means, execution steps, frequency and elapsed
simulation time. `--format text|markdown|latex` selects the table format;
`--diagnostics` adds per-trial metrics. Omit `--out-dir` to print only.

### Export for OIM `run_eval`

Add `--oim-out DIR` to `run` or `campaign` to write these files as trials
complete, or convert saved runs afterwards:

```bash
python3 -m c3plus.evaluation.oim_export --runs-dir results/full --out-dir results/full/oim
```

Either way it writes a small OIM-contract file per trial (`xarm6_open_table/scene/<run_id>.json`)
so C3+ scores in the same table as ADMM and MPPI. It renames the task and object
to OIM's spellings and validates every file before writing.

Two things to get right for a comparable sweep:

- **Pass `--steps N`.** `hyperparameters.steps` is the cap that censors a failed
  trial, so it must be one constant across the sweep. Without it the field is
  omitted and failures censor at their own length, which rewards giving up early.
- **Score C3+ in its own `run_eval` invocation.** A shared invocation credits
  every failure with the slowest execution time across all loaded runs.

`control_dt` and `compute_time` describe the same execution steps on different
clocks — simulation seconds and wall seconds — so their ratio is the simulator's
realtime rate, not an inconsistency. `T = steps_run × control_dt` is therefore
simulated task duration. True solver time is not recorded, so
`f_bar = 1/mean(compute_time)` is the achieved rate of the whole step, not a
solver-only figure.

### Other saved-data commands

| Command | Purpose |
| --- | --- |
| `postprocess --run-dir DIR --scene TASK --run-id ID` | Recompute one run's JSON from recorded data |
| `compact --run-dir DIR` | Validate a package and remove intermediates |
| `render --result FILE.json --out FILE.mp4` | Replay a recorded trajectory to MP4 |
| `visualize --task T --object O --start N --output FILE.png` | Object/EE-sample preview; no build or server needed |

## Repository structure

| Path | Purpose |
| --- | --- |
| [c3plus/](c3plus/) | Configuration, runtime, recording, evaluation, visualization |
| [c3plus/utils/](c3plus/utils/) | CLI, trial plans, campaign orchestration |
| [tests/](tests/) | Tests mirroring the source layout |
| [docker/](docker/) | Toolchain recipe, dependency pins, environment launcher |
| [campaign.yaml](campaign.yaml) | Default campaign selection: tasks, objects, pose pairs |
| [examples/poses/](examples/poses/) | Start/goal poses, byte-matched to the pinned OIM source |
| [examples/sampling_c3/](examples/sampling_c3/README.md) | Native controllers, simulator, configuration, models |
| [build_support/](build_support/) | Bazel wrapper and build overrides |
| `results/` | Generated output (gitignored) |

Method: *Approximating Global Contact-Implicit MPC via Sampling and Local
Complementarity* — [paper](https://arxiv.org/abs/2505.13350) ·
[project](https://approximating-global-ci-mpc.github.io/).
