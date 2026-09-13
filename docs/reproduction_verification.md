# Reproduction verification — 2026-09-12

Nikola's commit `cf092c53dab655100dc28b000d9554ad9fb0897c` was incorporated by
fast-forward. It adds `docker/Dockerfile` and `docker/shell.sh`, relocates solver
archives, and removes the old Docker/Compose setup. The only image tag named
in that commit is local `dairlib-c3plus-oim:latest`; no registry/digest is given.

## Changes

Added pinned Python Drake/PyYAML/analysis/mesh dependencies, FFmpeg and headless
rendering libraries. The shell accepts a published image override or a command,
and avoids renaming root during image creation. Container cache directories are
created with the intended ownership. Native build targets and algorithms are
unchanged.

Run/evaluation tools now resolve paths from the checkout rather than a previous
machine/worktree. A serial seed-42 runner packages after simulation cleanup,
refuses existing output directories, records binary hashes and seed/failures,
and prevents concurrent managed runs using a checkout-local lock. Campaign
resume skips only complete jobs and a stop marker finishes the current job.
The 25-job manifest preserves the previous session's exact evaluation goals.

## Checks

Host Python: `/root/miniconda3/envs/push_anything_ADMM/bin/python3`, Drake 1.51.1.
Existing native binaries were used; no controller rebuild/source change was
needed for this cleanup.

| Check | Scope / interpretation |
| --- | --- |
| Workflow unit tests | 11 passing: paths, grid counts/order, 25-job manifest, seed/settings isolation, configurations, failures, parameter validation, no overwrite, stop/resume |
| CHOMP validation | 12 checks passing, including historical offline-Drake distance agreement |
| Shell syntax / Git whitespace | `bash -n` and `git diff --check` pass |
| Runtime dependency check | 28/29 pass: imports, pinned Drake, native libraries, LCM, all 150 demo configurations/model references; Graphviz SVG fails |
| Short serial runs | 12/12 packaged; 11 without a logged controller failure, ReLU/slalom tripped the existing topple guard at sim-time 5.048 s |
| Dependency resolution | Pinned requirements resolve in a pip dry run; no system packages were installed on the host |

Verification runs are **dependency smoke tests**, not new success-rate or timing
benchmarks. They test startup, data capture and complete artifact packaging;
the two-frame MP4 verifies rendering/encoding, not all trajectory viewpoints.
Parsing all 150 demo configurations is not equivalent to completing all 300
full scene/pair/variant experiments.

All 12 runs used start 1/goal 1, seed 42, a 15-second recorder wall cap and a
two-frame replay limit. No controller behavior was changed to hide the slalom
topple. Raw logs/metrics/video are preserved, including the failure.

Logs, raw traces, metrics, plots, videos and the verification summary are kept
under `results/reproduction_validation_20260912/`. Prior nightly results and
other existing result directories are preserved unchanged.

## Still unverified / next stage

The host's Graphviz plugin configuration is zero-sized: `dot` exists but SVG
generation fails. The functional dependency check reports this failure; the
Docker recipe now rebuilds/checks the plugin configuration. The startup diagram
errors did not prevent logs, evaluation plots or MP4 packaging. The host was
not modified to repair Graphviz, and the Docker repair is not execution-tested.

Neither a Linux/Windows Docker CLI nor a Docker daemon/socket is available in
this workspace. **The Docker image has not been built or run here.** Host smoke
tests and a dependency resolver are not proof of image compatibility. The host
also lacks three optional mesh-generation imports; the image recipe now pins
them, but their installed execution inside Docker remains unverified.

On a Docker-enabled machine, rebuild the image, run the README build/dependency
checks, then run its 12-job serial smoke command. Save `docker image inspect`
output alongside results to identify the image. Use consistent CPU resources
and avoid competing simulations/builds during scientific comparisons.

The five-object mesh fixes, multi-object launcher and synchronized JSON recorder
are next-stage work after reproduction review. The supplied Windows JSON was
read through PowerShell; [its schema and current gaps](evaluation_json.md) are
documented, not represented as already implemented.

## Cleanup

Removed 164 redundant/historical Markdown files and 52 obsolete forensic/probe/
old-launcher files, plus three frozen executable snapshots. Core controllers,
meshes, simulation configs, reusable evaluation tools, upstream documentation
and experiment artifacts remain. Removed tracked files are recoverable from
commit `cf092c53d` or its parent; the executable snapshots also have a local
backup at `/tmp/c3plus-retired-binaries-KLZoAf/`.

Cleanup and these verification artifacts are versioned together. Older local
campaign directories are not included in this change set. Recorded run commit
IDs identify the pre-cleanup checkout; binary hashes identify the actual
executables used. Temporary caches are excluded from the published artifacts.
