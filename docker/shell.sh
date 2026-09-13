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
  ./docker/shell.sh python3 -m tools.experiments check
  ./docker/shell.sh python3 -m tools.experiments build

--build       Build the local Dockerfile, then open a shell/run COMMAND.
--build-only  Build the local Dockerfile and exit (does not compile C++ targets).
--help        Show this help without contacting Docker.

By default the image tag includes a hash of the Docker inputs and the host
UID/GID; changed toolchain inputs automatically select a new image.

Environment overrides:
  DAIRLIB_IMAGE          Explicit image tag/digest; pull if missing unless --build.
  DAIRLIB_CACHE_VOLUME   Named Bazel volume (default includes host UID/GID).
  DAIRLIB_CPUS           Container CPU quota (default: up to 24 daemon CPUs).
  DAIRLIB_MEM            Container RAM, integer with optional b/k/m/g (default: 24g).
  DAIRLIB_BAZEL_JOBS     Build workers (default: at most 8, lowered for CPU/RAM caps).
  DAIRLIB_BAZEL_RAM_MB   Bazel RAM budget (default: at most 14000, lowered for RAM cap).
  MESHCAT_PORT          Local port forwarded to container port 7000 (default: 7000).

Requires Linux x86-64 Docker, Bash, coreutils, and a writable checkout. Build
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
if ! DAEMON_INFO="$(docker info --format '{{.OSType}}/{{.Architecture}} {{.NCPU}}')"; then
    die 'Cannot access the Docker daemon. Start Docker and check your socket permissions.'
fi
read -r PLATFORM DAEMON_CPUS <<< "$DAEMON_INFO"
case "$PLATFORM" in
    linux/x86_64|linux/amd64) ;;
    *) die "Expected a Linux x86-64 Docker daemon; found ${PLATFORM}." ;;
esac

RUN_UID="$(id -u)"
RUN_GID="$(id -g)"
BUILD_UID="$RUN_UID"
BUILD_GID="$RUN_GID"
# Keep a normal user in the image. Root-owned checkouts run as root at runtime.
if [[ "$BUILD_UID" == 0 ]]; then BUILD_UID=1000; BUILD_GID=1000; fi

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
    # Hash contents with relative paths, so moving the checkout keeps the tag.
    FINGERPRINT="$(cd "$SCRIPT_DIR" && sha256sum Dockerfile requirements.txt \
        entrypoint.sh .dockerignore snopt7.6.tar.gz gurobi10.0.3_linux64.tar.gz | sha256sum)"
    IMAGE="dairlib-c3plus-oim:toolchain-${FINGERPRINT:0:12}-u${BUILD_UID}-g${BUILD_GID}"
fi

if [[ "$BUILD" == 1 ]] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    if [[ "$BUILD" == 0 && -n "${DAIRLIB_IMAGE:-}" ]]; then
        docker pull "$IMAGE"
    else
        check_inputs
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

[[ "$DAEMON_CPUS" =~ ^[1-9][0-9]*$ ]] || die 'Docker did not report a usable CPU count.'
DEFAULT_CPUS="$DAEMON_CPUS"
(( DEFAULT_CPUS > 24 )) && DEFAULT_CPUS=24
CPUS="${DAIRLIB_CPUS:-$DEFAULT_CPUS}"
MEM="${DAIRLIB_MEM:-24g}"
[[ "$CPUS" =~ ^[0-9]+([.][0-9]+)?$ && "$CPUS" =~ [1-9] ]] || die 'DAIRLIB_CPUS must be positive.'
if [[ "$MEM" =~ ^([1-9][0-9]*)([bBkKmMgG]?)$ ]]; then
    MEM_MB="${BASH_REMATCH[1]}"
    case "${BASH_REMATCH[2]}" in
        g|G) MEM_MB=$((MEM_MB * 1024)) ;;
        m|M) ;;
        k|K) MEM_MB=$((MEM_MB / 1024)) ;;
        ''|b|B) MEM_MB=$((MEM_MB / 1024 / 1024)) ;;
    esac
else
    die 'DAIRLIB_MEM must be an integer with optional b/k/m/g suffix (for example 24g).'
fi
(( MEM_MB >= 4096 )) || die 'Allow at least 4g container RAM; 24g is recommended for the C++ build.'
# Reserve 3 GiB for the JVM and tools; heavy C++ actions can use 2 GiB each.
DEFAULT_JOBS=$(((MEM_MB - 3072) / 2048))
(( DEFAULT_JOBS > 8 )) && DEFAULT_JOBS=8
CPU_JOBS="${CPUS%%.*}"
(( DEFAULT_JOBS > CPU_JOBS )) && DEFAULT_JOBS="$CPU_JOBS"
(( DEFAULT_JOBS < 1 )) && DEFAULT_JOBS=1
DEFAULT_RAM=$((MEM_MB - 3072))
(( DEFAULT_RAM > 14000 )) && DEFAULT_RAM=14000
JOBS="${DAIRLIB_BAZEL_JOBS:-$DEFAULT_JOBS}"
RAM="${DAIRLIB_BAZEL_RAM_MB:-$DEFAULT_RAM}"
[[ "$JOBS" =~ ^[1-9][0-9]*$ && "$RAM" =~ ^[1-9][0-9]*$ ]] || \
    die 'DAIRLIB_BAZEL_JOBS and DAIRLIB_BAZEL_RAM_MB must be positive integers.'
(( RAM < MEM_MB )) || die 'DAIRLIB_BAZEL_RAM_MB must be below the container RAM limit.'

VOLUME="${DAIRLIB_CACHE_VOLUME:-dairlib-c3plus-oim-bazel-cache-u${RUN_UID}-g${RUN_GID}}"
MESHCAT_PORT="${MESHCAT_PORT:-7000}"
[[ "$MESHCAT_PORT" =~ ^[1-9][0-9]*$ ]] && (( MESHCAT_PORT <= 65535 )) || \
    die 'MESHCAT_PORT must be a TCP port between 1 and 65535.'

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
docker run --rm "${TTY_FLAGS[@]}" "${USER_FLAGS[@]}" \
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
