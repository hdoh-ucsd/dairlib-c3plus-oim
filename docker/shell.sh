#!/usr/bin/env bash
#
# Drop into a shell in the dairlib-c3plus-oim container.
#
#   ./docker/shell.sh
#
# Starts no simulation on its own. It may build the toolchain image first.
# You get a prompt in the
# repo and run whatever you like by hand, e.g.
#
#   bazel build //examples/sampling_c3:franka_sampling_c3_controller
#
# Isolation, for reference:
#   * bridge network (not host) -> LCM multicast stays inside this container
#   * bazel output in a named volume -> no toolchain state on the host
#   * container uid == your uid -> bind-mounted files stay yours
#   * cpu/memory caps -> a run cannot starve the machine
#
# Env overrides: DAIRLIB_IMAGE, MESHCAT_PORT, DAIRLIB_CPUS, DAIRLIB_MEM.
# Optional arguments run a command instead of opening a shell.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE="${DAIRLIB_IMAGE:-dairlib-c3plus-oim:latest}"
VOLUME=dairlib-c3plus-oim-bazel-cache
MESHCAT_PORT="${MESHCAT_PORT:-7000}"
CPUS="${DAIRLIB_CPUS:-24}"
MEM="${DAIRLIB_MEM:-24g}"

# Build the toolchain image once, matching your uid/gid.
if ! command -v docker >/dev/null 2>&1; then
    echo 'Docker CLI/daemon required; install/start Docker before running this launcher.' >&2
    exit 1
fi
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    if [[ -n "${DAIRLIB_IMAGE:-}" ]]; then
        docker pull "${IMAGE}"
    else
        BUILD_UID="$(id -u)"
        BUILD_GID="$(id -g)"
        # Never rename root when launching from a root-owned WSL checkout.
        if [[ "${BUILD_UID}" == 0 ]]; then BUILD_UID=1000; BUILD_GID=1000; fi
        echo "==> building ${IMAGE} (one time, a few minutes)"
        docker build \
            --build-arg USER_UID="${BUILD_UID}" \
            --build-arg USER_GID="${BUILD_GID}" \
            -t "${IMAGE}" "${SCRIPT_DIR}"
    fi
fi

# Force-remove the container when this script exits, so closing the terminal or
# dropping an SSH session cannot leave a container running in the background.
NAME="dairlib-shell-$$"
trap 'docker rm -f "${NAME}" >/dev/null 2>&1 || true' EXIT INT TERM HUP

# Allocate a TTY only when there is one, so `echo cmd | ./docker/shell.sh` and
# CI-style non-interactive use both work.
TTY_FLAGS=(-i)
[[ -t 0 && -t 1 ]] && TTY_FLAGS=(-i -t)

status=0
if [[ $# -eq 0 ]]; then set -- bash; fi
docker run --rm "${TTY_FLAGS[@]}" \
    --name "${NAME}" \
    --hostname dairlib \
    --cap-add NET_ADMIN \
    -p "127.0.0.1:${MESHCAT_PORT}:7000" \
    -v "${REPO_ROOT}:/home/dairlib/dairlib:rw" \
    -v "${VOLUME}:/home/dairlib/.cache/bazel" \
    -e LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=0" \
    --cpus "${CPUS}" \
    --memory "${MEM}" \
    --shm-size 2g \
    -w /home/dairlib/dairlib \
    "${IMAGE}" "$@" || status=$?
exit "${status}"
