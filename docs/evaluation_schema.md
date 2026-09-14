# Evaluation schema and recorded data

The definitions below are the existing scientific contract. Moving Python files
does not change their meaning. See [experiments](experiments.md) for launch options
and [reproduction](reproduction.md) for saved configuration and source provenance.

## Event and time domains

| Term | Meaning |
| --- | --- |
| Execution step | One selected outer policy whose actuator commands are actually applied by the simulator; C3 and reposition policies both count |
| Planning update | One completed `SamplingC3Controller::ComputePlan()` update; an unadopted update does not become an execution step |
| ADMM iteration | An inner optimization iteration within a candidate solve; a planning update may perform several candidate solves |
| Recorder snapshot | An asynchronous debug observation paired with the latest object state; retained separately from execution boundaries |
| OSC tick | One low-level control update while tracking the selected trajectory; it is not a new outer policy by itself |
| Simulation integration timestep | The numerical plant integration interval, read from saved simulation settings; it is distinct from all the counts above |

Execution simulation time measures simulated motion. Execution wall time measures
elapsed steady-clock time at the native policy boundaries. Recorder wall time
limits collection, and process/packaging wall times describe other phases.
These clocks are not interchangeable. The N+1 boundary alignment and the exact
steps, T, and frequency definitions are specified below.

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
python3 -m tools compact \
  --run-dir results/object_generalization/latest_json/banana
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
| `evaluation.first_success_t` | First qualifying retained snapshot time, or `null`; see `recording.snapshot_dynamic.time` |
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
on export; snapshot diagnostic arrays remain under `recording.snapshot_dynamic` in new files.
Existing top-level success/error fields remain available to campaign
summaries: `success` means common evaluation ever-success, `t_success` is its
first retained-snapshot time, and legacy `first_success_t` is the recorder's
object-message event time, which can differ. `schema.legacy_fields` documents
these distinctions. New physical execution records use `schema.semantics_version: 4`
and `execution.alignment: "physical_policy_boundaries_v1"`.

New JSON separates three counts:

| Domain | Source and meaning |
| --- | --- |
| `execution.n_steps_executed` | Selected policies whose actuator commands the simulator actually applies; both C3 and reposition count |
| `planning.n_updates` | Completed `SamplingC3Controller::ComputePlan()` updates; policies superseded before adoption count only here |
| `recording.n_snapshots` | Retained asynchronous `C3_DEBUG_CURR` snapshots, with their complete original arrays in `recording.snapshot_dynamic` |

The simulator observes the exact pre-update plant state when the first actuator
command from a new `TRACKING_TRAJECTORY_ACTOR` policy is applied. The individual
C3/reposition trajectory channels also carry unselected candidates and do not
define execution steps. Informational `source_plan_utime` metadata follows the
selected policy through the executor; effort values and command timestamps are unchanged.
Rebuild the native programs together after this LCM schema change.

For N applied policies, `dynamic.object_pose`, the native object/joint state arrays,
`dynamic.time`, `execution.sim_time`, and `execution.wall_time` contain N+1 states or
boundaries. State zero is captured immediately before the first applied policy.
Each next policy closes the preceding one; a directly observed terminal state
closes the final held policy at shutdown or the explicitly requested step budget.
Missing terminal states or ambiguous source identities prevent an execution projection.
The current instrumentation requires the configured zero actuator delay.

`execution.wall_time` uses `std::chrono::steady_clock`, relative to the first
boundary. `execution.step_wall_time` contains its N positive differences, and
`frequency_hz = 1 / mean(step_wall_time) = N / (wall_time[-1] - wall_time[0])`.
For C3+, `dynamic.compute_time` is an explicit compatibility alias for these
execution wall durations, **not optimizer solve timing**. Unrecorded solve timing
is `planning.solve_time_s: null`. `planning.admm_iterations_per_solve` describes
the inner ADMM iterations per candidate solve (currently 3); a planning update
can run several candidate solves.

Physical execution durations vary, so `hyperparameters.control_dt` is `null`
with source `variable_physical_policy_duration`. `dynamic.time` remains simulation
time and mirrors the exact execution boundaries; the old snapshot times remain
unchanged in `recording.snapshot_dynamic.time`. `hyperparameters.steps` and
`execution.step_budget` are the explicit `--steps` budget, or `null` when unlimited.
Legacy `n_control_steps` and `steps_run` retain their deprecated snapshot/interval counts.

The raw snapshots, diagnostics, and post-success settling trajectory are retained.
Historical files remain snapshot-aligned; they cannot be converted into physical
execution trajectories without the original boundary telemetry. The previous
planner-dispatch frequency recording is also insufficient for that conversion.

New runs export and compact this automatically. To update an existing modern
result's export semantics without simulation or rendering, use its saved data:

```bash
python3 -m tools postprocess --export-only \
  --run-dir results/saved_run \
  --scene open_task \
  --run-id RUN_ID
```

Only the result JSON is replaced, after validation and atomic read-back checks.
Compacted runs use embedded recordings and settings; uncompressed runs use
their saved CSV, raw samples, and `evaluation_scene_config.yaml`. Supply
`--scene-config /path/to/saved_config.yaml` if that saved file is elsewhere.
This command does not upgrade historical `archive` metadata-only bundles.

Optional external evaluation (not part of this repository's Docker setup): with
the compatible evaluator installed in an OIM environment, point it at the saved
results. This checkout does not install `oim.run_eval`. The following is an
external analysis command, not an alternative experiment launcher:

```bash
python -m oim.run_eval \
  --runs-dir /path/to/exported_results \
  --group-by object --no-save
```

For aligned C3+ files, Table-II success and steps use the first simultaneous goal
crossing at an execution endpoint k; T is `execution.sim_time[k] - execution.sim_time[0]`.
On failure, steps use the configured budget (unavailable if unlimited), and T uses
the full recorded physical execution span. Frequency uses all recorded genuine
execution wall intervals. Later settling observations do not overturn first success.
Legacy C3+ files without proven alignment show execution steps, T, and f as
unavailable; their observation diagnostics remain accessible. MPPI/ADMM evaluation
retains its existing behavior. C3+ outcome identifiers/counts and derived time spacing
are excluded from “averaged over”; legitimate varying settings remain visible.

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

Generated results remain outside Git and are not included in a fresh clone.
The local collections are organized as follows:

| Directory | Contents |
| --- | --- |
| `results/smoke/` | Packaged startup and execution-alignment checks |
| `results/table2/` | Table-II campaign outputs and retained partial runs |
| `results/object_generalization/` | Imported-object and five-object checks |
| `results/archive/` | Historical experiments, older validation checks and failure evidence |

The historical archive's `index.json` contains its run catalog, cleanup report,
and shared provenance. Original shared documents are retained under `source_files`.
Its `runs/` and `media/` collections have 125 run folders, each containing one
JSON and one MP4. The 94
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
python3 -m tools render \
  --result results/object_generalization/latest_json/banana/exponential_open_task_banana_s02g02_yaw_000_seed42_result.json \
  --out results/banana_replay.mp4 --max-frames 1200
```

The renderer reads the goal and model references from the result; repository
assets must still be available and match any recorded asset hashes. Older runs
without these hashes require the original repository revision. Explicit model
overrides remain supported.
`render --trace` still accepts legacy JSONL traces.

Developer/debug compatibility: `cost-figure` currently expects legacy
snapshot-aligned results or retained intermediate metrics. It cannot read a
compacted semantics-version-4 result directly. For a compatible legacy run:

```bash
python3 -m tools cost-figure \
  --run-dir results/legacy_run \
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
See [postprocess.py](../c3plus/evaluation/postprocess.py)
for the scene schema, output fields, and metric definitions.

</details>
