#!/usr/bin/env bash
# Build a toolchain image, then run a shell or command against this checkout.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
    cat <<'EOF'
Usage: ./docker/shell.sh [--build | --build-only] [--] [COMMAND [ARG...]]

No command opens Bash. Commands and arguments are forwarded without evaluation.
  ./docker/shell.sh --build-only
  ./docker/shell.sh python3 -m c3plus.utils check
  ./docker/shell.sh python3 -m c3plus.utils build

--build       Build the local Dockerfile, then open a shell/run COMMAND.
--build-only  Build the local Dockerfile and exit (does not compile C++ targets).
--help        Show this help without contacting Docker.

By default the image tag includes a hash of the Docker inputs and the host
UID/GID; changed toolchain inputs automatically select a new image.

Environment overrides:
  DAIRLIB_IMAGE          Explicit image tag/digest; pull if missing unless --build.
  DAIRLIB_CACHE_VOLUME   Named Bazel volume (default includes host UID/GID).
  DAIRLIB_CPUS           Container CPU quota (default: up to 24 daemon CPUs).
  DAIRLIB_MEM            Container RAM, integer with optional b/k/m/g (default:
                         75% of daemon RAM, capped at 24 GiB; minimum 4 GiB).
  DAIRLIB_BAZEL_JOBS     Build workers (default: at most 8, lowered for CPU/RAM caps).
  DAIRLIB_BAZEL_RAM_MB   Bazel RAM budget (default: at most 14000, lowered for RAM cap).
  MESHCAT_PORT          Local port forwarded to container port 7000 (default: 7000).

Requires a local Linux x86-64 Docker engine (native Linux or Docker Desktop on
WSL2/Intel macOS), Bash, sha256sum or shasum, and a writable checkout. Build
inputs include the tracked SNOPT/Gurobi archives in docker/. First toolchain
build and first C++ build need network access and substantial time/disk space.
EOF
}

die() { echo "docker/shell.sh: $*" >&2; exit 2; }

BUILD=0
BUILD_ONLY=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --build) BUILD=1; shift ;;
        --build-only) BUILD=1; BUILD_ONLY=1; shift ;;
        --) shift; break ;;
        *) break ;;
    esac
done
if [[ "$BUILD_ONLY" == 1 && $# -gt 0 ]]; then
    die '--build-only does not accept a container command.'
fi

command -v docker >/dev/null 2>&1 || die 'Docker CLI required; install Docker first.'
for marker in MODULE.bazel .bazeliskrc c3plus/utils/__main__.py; do
    [[ -s "${REPO_ROOT}/${marker}" ]] || \
        die "Expected a complete repository at ${REPO_ROOT}; missing ${marker}. Run this checkout's docker/shell.sh."
done
[[ -w "$REPO_ROOT" && -x "$REPO_ROOT" ]] || \
    die "Checkout ${REPO_ROOT} must be writable to create Bazel symlinks and results."
if ! DAEMON_INFO="$(docker info --format '{{.OSType}}/{{.Architecture}} {{.NCPU}} {{.MemTotal}}')"; then
    die 'Cannot access the Docker daemon. Start Docker and check your socket permissions.'
fi
read -r PLATFORM DAEMON_CPUS DAEMON_MEM <<< "$DAEMON_INFO"
case "$PLATFORM" in
    linux/x86_64|linux/amd64) ;;
    *) die "Expected a Linux x86-64 Docker daemon; found ${PLATFORM}." ;;
esac

# Bind paths refer to this machine, so a remote daemon cannot use this checkout.
# DOCKER_CONTEXT takes precedence over DOCKER_HOST, as it does in the Docker CLI.
if [[ -n "${DOCKER_CONTEXT:-}" ]]; then
    DOCKER_ENDPOINT="$(docker context inspect "$DOCKER_CONTEXT" --format '{{.Endpoints.docker.Host}}')" || \
        die 'Cannot inspect the selected Docker context.'
elif [[ -n "${DOCKER_HOST:-}" ]]; then
    DOCKER_ENDPOINT="$DOCKER_HOST"
else
    DOCKER_ENDPOINT="$(docker context inspect --format '{{.Endpoints.docker.Host}}')" || \
        die 'Cannot inspect the selected Docker context.'
fi
[[ "$DOCKER_ENDPOINT" == unix://* ]] || \
    die "A local Docker Unix socket is required to mount this checkout; context endpoint is ${DOCKER_ENDPOINT}. Select a local Linux engine (use a WSL2 shell on Windows)."

# Validate limits before pulling or building an image. Docker Desktop often has
# much less RAM available than the host, so use the daemon's reported resources.
[[ "$DAEMON_CPUS" =~ ^[1-9][0-9]{0,5}$ ]] || die 'Docker did not report a usable CPU count.'
[[ "$DAEMON_MEM" =~ ^[1-9][0-9]{0,14}$ ]] || die 'Docker did not report usable total memory.'
DEFAULT_CPUS="$DAEMON_CPUS"
(( DEFAULT_CPUS > 24 )) && DEFAULT_CPUS=24
CPUS="${DAIRLIB_CPUS:-$DEFAULT_CPUS}"
[[ "$CPUS" =~ ^[0-9]{1,6}([.][0-9]{1,6})?$ && "$CPUS" =~ [1-9] ]] || die 'DAIRLIB_CPUS must be positive.'
CPU_JOBS=$((10#${CPUS%%.*}))
CPU_FRACTION="${CPUS#*.}"
if (( CPU_JOBS > DAEMON_CPUS )) || \
        { (( CPU_JOBS == DAEMON_CPUS )) && [[ "$CPUS" == *.* && "$CPU_FRACTION" =~ [1-9] ]]; }; then
    die "DAIRLIB_CPUS exceeds the Docker daemon's ${DAEMON_CPUS} CPUs."
fi
DEFAULT_MEM_MB=$((DAEMON_MEM / 1024 / 1024 * 3 / 4))
(( DEFAULT_MEM_MB > 24576 )) && DEFAULT_MEM_MB=24576
MEM="${DAIRLIB_MEM:-${DEFAULT_MEM_MB}m}"
if [[ "$MEM" =~ ^([1-9][0-9]{0,14})([bBkKmMgG]?)$ ]]; then
    MEM_COUNT="${BASH_REMATCH[1]}"
    case "${BASH_REMATCH[2]}" in
        g|G) MEM_UNIT=1073741824 ;;
        m|M) MEM_UNIT=1048576 ;;
        k|K) MEM_UNIT=1024 ;;
        ''|b|B) MEM_UNIT=1 ;;
    esac
    # Compare before multiplying to avoid overflow on unreasonable overrides.
    (( MEM_COUNT <= DAEMON_MEM / MEM_UNIT )) || \
        die 'DAIRLIB_MEM exceeds the memory available to the Docker daemon. Increase Docker Desktop memory or choose a smaller limit.'
    MEM_MB=$((MEM_COUNT * MEM_UNIT / 1024 / 1024))
else
    die 'DAIRLIB_MEM must be an integer with optional b/k/m/g suffix (for example 16g).'
fi
(( MEM_MB >= 4096 )) || die 'Allow at least 4 GiB container RAM. Increase Docker Desktop memory, or set DAIRLIB_MEM within the daemon memory limit.'
# Reserve 3 GiB for the JVM and tools; heavy C++ actions can use 2 GiB each.
DEFAULT_JOBS=$(((MEM_MB - 3072) / 2048))
(( DEFAULT_JOBS > 8 )) && DEFAULT_JOBS=8
(( DEFAULT_JOBS > CPU_JOBS )) && DEFAULT_JOBS="$CPU_JOBS"
(( DEFAULT_JOBS < 1 )) && DEFAULT_JOBS=1
DEFAULT_RAM=$((MEM_MB - 3072))
(( DEFAULT_RAM > 14000 )) && DEFAULT_RAM=14000
JOBS="${DAIRLIB_BAZEL_JOBS:-$DEFAULT_JOBS}"
RAM="${DAIRLIB_BAZEL_RAM_MB:-$DEFAULT_RAM}"
[[ "$JOBS" =~ ^[1-9][0-9]{0,5}$ && "$RAM" =~ ^[1-9][0-9]{0,8}$ ]] || \
    die 'DAIRLIB_BAZEL_JOBS and DAIRLIB_BAZEL_RAM_MB must be positive integers.'
(( RAM < MEM_MB )) || die 'DAIRLIB_BAZEL_RAM_MB must be below the container RAM limit.'
MESHCAT_PORT="${MESHCAT_PORT:-7000}"
[[ "$MESHCAT_PORT" =~ ^[1-9][0-9]{0,4}$ ]] && (( MESHCAT_PORT <= 65535 )) || \
    die 'MESHCAT_PORT must be a TCP port between 1 and 65535.'

RUN_UID="$(id -u)"
RUN_GID="$(id -g)"
BUILD_UID="$RUN_UID"
BUILD_GID="$RUN_GID"
# Keep a normal user in the image. Root-owned checkouts run as root at runtime.
if [[ "$BUILD_UID" == 0 ]]; then BUILD_UID=1000; BUILD_GID=1000; fi
VOLUME="${DAIRLIB_CACHE_VOLUME:-dairlib-c3plus-oim-bazel-cache-u${RUN_UID}-g${RUN_GID}}"
[[ "$VOLUME" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || \
    die 'DAIRLIB_CACHE_VOLUME must be a named Docker volume, not a filesystem path.'

check_inputs() {
    local filename
    for filename in Dockerfile requirements.txt entrypoint.sh .dockerignore \
        snopt7.6.tar.gz gurobi10.0.3_linux64.tar.gz; do
        [[ -s "${SCRIPT_DIR}/${filename}" ]] || \
            die "Missing docker/${filename}; restore tracked Docker inputs from your checkout."
    done
}

if [[ -n "${DAIRLIB_IMAGE:-}" ]]; then
    IMAGE="$DAIRLIB_IMAGE"
else
    check_inputs
    if command -v sha256sum >/dev/null 2>&1; then
        HASH=(sha256sum)
    elif command -v shasum >/dev/null 2>&1; then
        HASH=(shasum -a 256)
    else
        die 'Install sha256sum or shasum to identify the local Docker recipe.'
    fi
    # Hash contents with relative paths, so moving the checkout keeps the tag.
    FINGERPRINT="$(cd "$SCRIPT_DIR" && "${HASH[@]}" Dockerfile requirements.txt \
        entrypoint.sh .dockerignore snopt7.6.tar.gz gurobi10.0.3_linux64.tar.gz | "${HASH[@]}")"
    IMAGE="dairlib-c3plus-oim:toolchain-${FINGERPRINT:0:12}-u${BUILD_UID}-g${BUILD_GID}"
fi

if [[ "$BUILD" == 1 ]] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    if [[ "$BUILD" == 0 && -n "${DAIRLIB_IMAGE:-}" ]]; then
        docker pull "$IMAGE"
    else
        check_inputs
        docker buildx version >/dev/null 2>&1 || \
            die 'Docker Buildx is required to build the toolchain. Install the Docker buildx plugin (included with Docker Desktop).'
        echo "==> building toolchain ${IMAGE} (C++ targets are built separately)"
        docker build --build-arg USER_UID="$BUILD_UID" --build-arg USER_GID="$BUILD_GID" \
            -t "$IMAGE" "$SCRIPT_DIR"
    fi
fi
if [[ "$BUILD_ONLY" == 1 ]]; then
    echo "Toolchain image ready: ${IMAGE}"
    exit 0
fi

read -r IMAGE_ID IMAGE_UID IMAGE_GID <<< "$(docker image inspect --format \
    '{{.Id}} {{index .Config.Labels "org.dairlib.uid"}} {{index .Config.Labels "org.dairlib.gid"}}' "$IMAGE")"
if [[ "$RUN_UID" != 0 && ( "$IMAGE_UID" != "$RUN_UID" || "$IMAGE_GID" != "$RUN_GID" ) ]]; then
    die "Image ${IMAGE} does not declare your UID/GID (${RUN_UID}:${RUN_GID}). Use --build to create a matching image, or unset DAIRLIB_IMAGE."
fi

NAME="dairlib-shell-$$"
cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

TTY_FLAGS=(-i)
[[ -t 0 && -t 1 ]] && TTY_FLAGS=(-i -t)
USER_FLAGS=()
[[ "$RUN_UID" == 0 ]] && USER_FLAGS=(--user 0:0)
if [[ $# -eq 0 ]]; then set -- bash; fi
echo "==> ${IMAGE}; cache=${VOLUME}; CPUs=${CPUS}; RAM=${MEM}; Bazel jobs=${JOBS}"

status=0
docker run --rm "${TTY_FLAGS[@]}" ${USER_FLAGS[@]+"${USER_FLAGS[@]}"} \
    --name "$NAME" --hostname dairlib --cap-add NET_ADMIN \
    -p "127.0.0.1:${MESHCAT_PORT}:7000" \
    -v "${REPO_ROOT}:/home/dairlib/dairlib:rw" \
    -v "${VOLUME}:/home/dairlib/.cache/bazel" \
    -e HOME=/home/dairlib \
    -e LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=0' \
    -e "DAIRLIB_BAZEL_JOBS=${JOBS}" -e "DAIRLIB_BAZEL_RAM_MB=${RAM}" \
    -e "C3PLUS_CONTAINER_IMAGE=${IMAGE}" -e "C3PLUS_CONTAINER_IMAGE_ID=${IMAGE_ID}" \
    --cpus "$CPUS" --memory "$MEM" --shm-size 2g \
    -w /home/dairlib/dairlib "$IMAGE" "$@" || status=$?
exit "$status"
