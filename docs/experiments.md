# Experiments

Use the [root Quick Start](../README.md#quick-start) to enter the Docker environment.
This guide covers selections, campaigns, previews, and configuration. See
[evaluation and result definitions](evaluation_schema.md) for recorded data, and
[reproduction](reproduction.md) for the environment and provenance requirements.

## CLI reference

Run commands from the repository root inside the Docker environment in the
[root Quick Start](../README.md#quick-start). Output paths below are examples; choose
a fresh directory for each run or campaign.
Use `python3 -m tools COMMAND --help` for the full option list.

| Command | Purpose |
| --- | --- |
| `scenes` | List scene names and available start/goal indices |
| `run` | Push one object, or run a list of objects serially |
| `visualize` | Save a mesh PNG, optionally with candidate end-effector (EE) positions |
| `campaign` | Run a scene/start/goal grid or a saved manifest |
| `run_launch`, `run_launch_simple_s2` | Run the fixed-goal orientation campaigns |

### `run`: one or more objects

Run one trial with an explicit object, start, and goal, retaining the configured
goal orientation:

```bash
python3 -m tools run \
  --scene open_task --objects T_block \
  --start 2 --goal 2 \
  --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/example_run
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
| `--steps` | Optional maximum number of applied outer policies, including reposition; unlimited when omitted |
| `--out PATH` | Required output directory; existing individual run folders are refused |
| `--max-frames` | Maximum video frames, default 1200 |
| `--port` | Trial LCM port, default 18001 |
| `--dry-run` | Print the resolved plan without writing files or launching processes |

Start positions, goal positions, and orientations are separate selections in
[experiments.yaml](../examples/sampling_c3/shared_parameters/experiments.yaml).
Changing goal yaw preserves its indexed position; each start retains its initial
orientation and robot joint pose. Add `--dry-run` to inspect the combination.
For a short startup/recording check, use `--cap 10 --max-frames 40`.
See the [root troubleshooting section](../README.md#troubleshooting) for the
current native-state export limitation encountered in a longer smoke test.

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
python3 -m tools run \
  --scene open_task --objects T_block sugar_box power_drill hammer banana \
  --start 2 --goal 2 \
  --obstacle_cost exponential --seed 42 --cap 600 \
  --out results/object_generalization/multi_object
```

Trials run in the listed order with the same indexed start, goal, yaw, cost, seed,
and per-trial cap. One selected object writes directly to `--out`; multiple
objects write to `<out>/<object>/`. Explicit `T_block` preserves its existing
two-box geometry and perimeter sampler. The fixed sugar box in `ycb_clutter`
is an obstacle, separate from the movable `sugar_box` selection.

The CLI and [native controller](../examples/sampling_c3/franka_sampling_c3_controller.cc)
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
See [physics_models.json](../examples/sampling_c3/urdf/objects/physics_models.json)
for the physical and evaluation models, and
[mesh_import.json](../examples/sampling_c3/urdf/objects/mesh_import.json) for geometry repairs.

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
With an explicit `--steps B`, simulation also stops before policy B+1 is applied.
This limit counts physical policy adoptions, not planner updates or snapshots.
For example, append `--steps 1000` to configure a Table-II step budget; the default
remains unlimited. The cap or success/settling condition can end the run earlier.

JSON export, video rendering, and cleanup are automatic. Each completed trial
keeps one `*_result.json` and one MP4. Resolved configurations, recordings,
diagnostics, hashes, and completion status are described under [Evaluation schema](evaluation_schema.md#results).

The runner checks every selected destination before starting a batch. `run` has
no `--resume`: after interruption, select only missing objects or choose a fresh
output path. For example, to finish a batch with only banana remaining, use
`--objects banana --out results/object_generalization/multi_object/banana`.
Launch, recording, or packaging errors stop the batch. A packaged trial with
`success: false` can continue to the next object; inspect `runtime_status.failures`
for native process failures. `--dry-run` does not check existing output compatibility.

### `visualize`: mesh and EE previews

This command saves a PNG and exits using the Docker image's Python dependencies;
it needs no native build, browser, or port. The default scene view includes the
object, arm, tool, table, platform, and obstacles.

```bash
# Default T_block at start S2, with coordinate axes.
python3 -m tools visualize \
  --scene open_task --start 2 --frames --output results/previews/T_block.png

# Cblock at goal G2 with absolute yaw +90 degrees.
python3 -m tools visualize \
  --scene icra_sign --pose goal --goal 2 --goal-yaw-degrees 90 \
  --view object --frames --output results/previews/Cblock_goal.png
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
python3 -m tools visualize --scene open_task --start 2 \
  --view top --hide-robot --ee-samples --output results/previews/T_block_ee.png
```

Use `--scene icra_sign` for the configured Cblock sampler. Blue spheres show EE
centres with a 5.55 mm tip radius. `--ee-samples 100 --sample-seed 42` changes only
the preview count and seed. The companion file is `<PNG stem>.ee_samples.json`.

To preview all four imported OBJ meshes and their EE candidates:

```bash
for object in sugar_box power_drill hammer banana; do
  python3 -m tools visualize --scene open_task --start 2 \
    --mesh "examples/sampling_c3/urdf/objects/${object}_centered.obj" \
    --position 0.366 0.431 -0.029 --view top --hide-robot --ee-samples \
    --output "results/previews/${object}_ee.png"
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

The [object profiles](../examples/sampling_c3/shared_parameters/experiments.yaml)
select simulation models, controller models, and sampling settings.
In an SDF, `<box><size>` specifies dimensions in metres and `<pose>` places each part.

| Object | Simulation geometry | Configured EE sampler |
| --- | --- | --- |
| T_block | [Two boxes](../examples/sampling_c3/urdf/push_t_oimscale_m01.sdf) | [Perimeter sampling](../examples/sampling_c3/shared_parameters/profiles/t_shape/sampling_params.yaml) of controller collision boxes |
| Cblock | [Three boxes](../examples/sampling_c3/urdf/push_c_glyph.sdf) | [Mesh-normal settings](../examples/sampling_c3/shared_parameters/profiles/icra_sign/sampling_params.yaml), using the [sampling mesh](../examples/sampling_c3/urdf/c_glyph_base/c_glyph_base.obj) |

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
python3 -m tools run_launch \
  --output-root results/table2 --dry-run

python3 -m tools run_launch_simple_s2 \
  --output-root results/table2_s2 --dry-run
```

Named campaigns default to `results/reproduce/run_launch/` and
`results/reproduce/run_launch_simple_s2/`, seed 42, and a 600-second recorder cap
per trial. Each trial resets to its selected start and uses the same goal in the
controller, recorder, evaluation, and replay. Both costs retain the preset
semantics described for `run --obstacle_cost` above.

Plan the full 300-run grid; remove `--dry-run` to execute any of these plans:

```bash
python3 -m tools campaign \
  --scenes open_task single_obstacle shelf_gap ycb_clutter icra_sign slalom \
  --obstacle_cost both --pairs all --seed 42 \
  --output-root results/full_grid --dry-run
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
python3 -m tools campaign \
  --manifest c3plus/experiments/manifests/campaign_seed42.json \
  --output-root results/manifest_run --dry-run
```

This manifest preserves 25 ordered `single_obstacle` jobs: 13 exponential and
12 ReLU, ending at exponential S3/G3. Rounded evaluation/replay goals must match
the native goal within 0.0001 m and 0.0001 rad; `controller_goal` is recorded
separately. `--cap` overrides the saved caps. Without selection flags, generic
`campaign` plans four smoke jobs across `single_obstacle`, `icra_sign`, and both
costs. Old `variant`/`baseline` plans require a new output root; no aliases are provided.

</details>

## Configuration reference

[configs/catalog.py](../c3plus/configs/catalog.py) queries scene and object profiles;
[configs/resolver.py](../c3plus/configs/resolver.py) composes selected native settings
and writes isolated snapshots. [configs/paths.py](../c3plus/configs/paths.py)
defines checkout and asset paths. The [architecture guide](architecture.md)
explains orchestration, runtime, recording, evaluation, and visualization owners.

The shared [experiment catalogue](../examples/sampling_c3/shared_parameters/experiments.yaml)
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
configuration directories remain for native `--demo_name` compatibility.

Plans report the starting pose as `[w, x, y, z, X, Y, Z]`, goal position in metres,
and goal yaw in radians. Configuration hashes prevent resuming a campaign with
changed selected settings; use a new output root for older campaign plans.

The [six scene YAMLs](../c3plus/configs/scenes/) supply evaluation geometry and
planner overrides. Their optional `planner` block references `obstacles.polygons`
by zero-based index:

```yaml
# shelf_gap: obstacle slots 0–2 use the large shelf; 3–4 use the small one.
planner:
  box_polygons: [0, 0, 0, 1, 1]
```

`box_polygons` assigns rectangles to consecutive native slots;
`polygon_overrides` maps explicit slots to polygon indices (YCB: `{2: 2, 3: 3}`).
ICRA also sets `object_footprint: c_glyph` and `obstacle_top_z: 0.046`.
Unassigned slots, including the robot base, retain native disc geometry.
`configs.resolver.planner_environment()` validates and serializes these settings.
Changing physical scene geometry requires updating its native model/configuration
alongside the evaluator YAML. Start and goal changes belong in the experiment catalogue.

`check` emits JSON and exits nonzero on failure; resolve failures before running
campaigns. `--runtime-only` skips optional asset-generation imports. Run
`python3 -m unittest discover -s tests` for the mirrored test suite.
Tests use temporary fixtures without starting scientific campaigns; configuration
checks parse native assets without advancing physics. Run the setup checks in
your container before starting a campaign. These checks do not establish
experiment outcomes.

## Historical object ablation

The [September 2026 object-ablation report](history/object_ablation.md) preserves
an earlier investigation of object mass, geometry, goal distance, and Q weights.
It reported that shrinking the object while holding mass fixed confounded the
early geometry comparison. Those small historical trial sets used older demos
and sample-based metrics; they are not new validation of the current benchmark.
The report retains its original commands and media references, some of which
point to collections outside this checkout.
