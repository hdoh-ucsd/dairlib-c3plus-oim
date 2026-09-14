# Reproduction and environment

The [root Quick Start](../README.md#quick-start) is the normal setup and execution
path: open `./docker/shell.sh` on the host, then run `python3 -m tools` inside the
container. This guide explains that same environment, its resource settings,
and the limits of reconstructing a saved experiment.

## Docker Environment

[docker/Dockerfile](../docker/Dockerfile) defines the single toolchain image:
Ubuntu 24.04, Drake 1.51.1, Bazel 8.4.0, native solver dependencies, and
[pinned Python packages](../docker/requirements.txt). The runtime `python3` uses an
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
| Native C++ or LCM types | **CONTAINER:** `python3 -m tools build --jobs 4` rebuilds all three binaries |
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

Inside the container, `python3 -m tools build --jobs 4` overrides the environment's
default job count for that build. Heavy C++ compilation can exceed scheduling estimates;
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
| `Build franka_sim from this checkout first` or missing native flags | **CONTAINER:** run `python3 -m tools build --jobs 4` in the same checkout/container, then repeat `check`. |
| Python imports or `pip check` fail in an old image | **HOST:** `./docker/shell.sh --build`, then repeat container build and validation. |
| Memory limit rejected or compiler killed | Check Docker engine RAM, reduce jobs, or adjust the resource settings above. |
| Output folder exists / different campaign plan | Choose a fresh `--out`/`--output-root`. Campaign `--resume` requires a matching plan; `run` has no resume flag. |

Known exporter limitation: a 20-second validation run reached packaging but
failed with `Invalid exact native execution state`. The current exporter rejects
raw Drake quaternions when their squared norm differs from 1 by more than
`1e-5`; small integration drift crossed that limit. A 10-second startup run
packaged successfully. This is separate from Docker setup and remains unresolved;
longer-run packaging is not fully verified. Failed runs retain their raw data.
The package reorganization preserves this guard and the existing evaluation
definitions.

For `FATAL: mkdir('/home/dairlib/.cache/bazel'): Permission denied`, current
startup handles a root cache mismatch automatically. Rebuild and reopen an old
image with `./docker/shell.sh --build`. If recovering an **existing root shell
in an older image**, change only the cache root, then rebuild:

```bash
chown --no-dereference 0:0 /home/dairlib/.cache/bazel
python3 -m tools build --jobs 4
```

For nonroot shells, use the default UID/GID-specific volume or a cache owned by
that user. Do not recursively change ownership of existing Bazel cache contents.

## Reusing recorded settings

Resolve configuration from the recorded run before comparing it with current
source. A completed JSON embeds the native YAML snapshots, evaluation geometry,
runtime settings, source-state envelope, and available asset/binary hashes.
The source snapshot records the HEAD commit plus captured tracked changes and
nonignored untracked files. Older dirty runs without that capture cannot be
reconstructed from today's working tree.

Restore the recorded revision and captured source changes in a separate checkout,
and retain the recorded image or its registry digest when available. Match the
selected model assets and runtime resource settings. A saved binary hash
identifies a binary; it does not by itself prove which source built it.
Configuration hashes must agree before a campaign can resume. Use a new output
root for a new selection or changed settings.

`postprocess --export-only`, `compact`, and `render` consume saved data. They do
not repeat the experiment or recover native boundary telemetry that was never
recorded. See the [evaluation schema](evaluation_schema.md) for their limits.
The default seed is fixed at 42; timing, scheduling and numerical environment
still affect asynchronous runs.
