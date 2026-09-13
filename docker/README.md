# Docker toolchain

Run commands from the repository root. The main [README](../README.md) describes
the experiment workflow; this directory defines its container environment.

```bash
./docker/shell.sh --build-only
./docker/shell.sh python3 -m tools.experiments build
./docker/shell.sh python3 -m tools.experiments check --require-binaries --check-scenes
./docker/shell.sh
```

`--build-only` builds the toolchain image. The separate `build` command compiles
the three simulation/controller binaries.
The first builds require network access, substantial free disk space, and may
take much longer than an ordinary incremental build.

Use a local Linux x86-64 Docker daemon, or Docker Desktop with WSL2 integration
and a Linux checkout. ARM emulation and remote Docker daemons are not supported
by this launcher. Bash and coreutils are required on the host; host Python and
host solver installations are not required. Both licensed solver archives are
tracked under `docker/` and must be present before a local image build.
The Gurobi 10.0.3 archive is required at link time because `.bazelrc` enables
Gurobi, including for the shipped scenes that use the OSQP-based C3+ projection.
Configurations selecting other solvers need their corresponding runtime setup.

## Files and dependency boundaries

| File | Purpose |
| --- | --- |
| `Dockerfile` | Ubuntu 24.04 toolchain, Drake prerequisites, solvers, procman, isolated Python runtime |
| `requirements.txt` | Pinned direct and transitive runtime/mesh Python dependencies |
| `shell.sh` | Image selection/build, host identity, limits, cache and command forwarding |
| `entrypoint.sh` | Container-local LCM loopback route and Bazel settings |
| `.dockerignore` | Limit build context to the actual image inputs |
| `tests/test_launcher.py` | Daemon-free checks of launcher and entrypoint behavior |

Drake 1.51.1 and its source archive checksum match `MODULE.bazel`; Bazel 8.4.0 is
selected by `.bazeliskrc` after mounting the checkout. Procman's libbot2 source
is fixed at [f007e6e](https://github.com/RobotLocomotion/libbot2/commit/f007e6e902f237782b32d739325b2c0290095866).

The runtime interpreter is `/opt/push-anything-venv/bin/python3`. It uses an
isolated virtual environment, with only Ubuntu's `lcm` package linked into it.
Drake's native build continues to use `/usr/bin/python3` and its own prerequisites.
This separation prevents Ubuntu's Matplotlib namespace and build/lint packages
from interfering with pinned runtime dependencies. `python3 -m pip check` is a
valid check in the new image; the Dockerfile also imports the actual Drake and
Matplotlib 3-D modules. Older images with `--system-site-packages` can fail these
checks and should be rebuilt.

Ubuntu's base tag and apt repositories are not snapshot-pinned, and wheel hashes
are not locked. These inputs can still change across image rebuilds. Keep the
resulting image or registry digest for an exact environment; run manifests
record the requested image tag and immutable image ID. The default tag hashes
the local Docker inputs and identifies changes to the recipe, rather than
guaranteeing an identical image after a rebuild.

## Resources, ownership, and output

The default runtime limit is 24 GiB RAM and up to 24 of the Docker daemon's CPUs.
Configure Docker Desktop/WSL with enough memory, or select a smaller container:

```bash
export DAIRLIB_CPUS=4
export DAIRLIB_MEM=12g
./docker/shell.sh python3 -m tools.experiments build
```

Bazel defaults to at most eight workers, reduced automatically for smaller CPU
and RAM limits. Its RAM budget is at most 14000 MiB, leaving room for the 2 GiB
JVM and other tools. Heavy C++ compilation can still exceed these estimates;
`DAIRLIB_BAZEL_JOBS` and `DAIRLIB_BAZEL_RAM_MB` provide explicit overrides. Runtime
container limits do not limit the separate Docker image builder; configure that
builder through Docker itself. `./docker/shell.sh --help` lists every override.

The checkout is mounted at `/home/dairlib/dairlib`. Generated results and Bazel's
workspace symlinks appear in the checkout; compiled objects and external sources
live in the named `dairlib-c3plus-oim-bazel-cache-u<uid>-g<gid>` volume. The cache
survives container exit. Set `DAIRLIB_CACHE_VOLUME` to use a separate cache; its
ownership must match the container user.

Normal users get an image built for their UID/GID. A root-owned checkout runs
with container UID 0 and has its own cache; the launcher never changes checkout
permissions or ownership. Explicit `DAIRLIB_IMAGE` overrides must provide the
matching UID/GID labels for nonroot hosts; `--build` creates such an image.

LCM multicast stays on the container's loopback interface, requiring `NET_ADMIN`
within that container. Run all processes for an experiment together in one
container. Meshcat is available at `http://localhost:7000`. If Docker reports
that port 7000 is already allocated, run `MESHCAT_PORT=7001 ./docker/shell.sh`
and open Meshcat at `http://localhost:7001`. This lets the existing container
continue using port 7000. Container exit removes the container and returns the
command's exit status.

## Rebuilds and validation

Changes to Dockerfile, entrypoint, requirements, build-context rules, or solver
archives select a new default image tag. Source-code changes need a native
`build`, not a toolchain image rebuild. To explicitly rebuild the local recipe:

```bash
./docker/shell.sh --build-only
```

Setting `DAIRLIB_IMAGE` uses that tag/digest, pulling it if absent. Use
`DAIRLIB_IMAGE=my-toolchain:local ./docker/shell.sh --build-only` to build a custom
local tag. An existing legacy `dairlib-c3plus-oim:latest` is not silently reused.

Run the lightweight launcher tests without Docker:

```bash
python3 -m unittest discover -s docker/tests -v
bash -n docker/shell.sh docker/entrypoint.sh
```

The revised Python layout was checked by installing all requirements into a
fresh isolated Python 3.12 venv inside a disposable existing toolchain container:
`pip check`, Drake plant/VTK imports, Matplotlib 3-D imports, and the linked LCM
binding passed. Launcher/entrypoint tests also cover command quoting, exit codes,
root and nonroot ownership, image builds/pulls, cache selection, and limits.
This validation does not establish a complete fresh image/native build; run the
build and environment commands above before starting a reproduction campaign.
