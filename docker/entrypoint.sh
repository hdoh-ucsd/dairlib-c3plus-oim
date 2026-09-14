#!/usr/bin/env bash
set -euo pipefail

# Every simulation process must live in this one network namespace. Fail early
# if --cap-add NET_ADMIN is missing instead of silently losing LCM messages.
if [[ "$(id -u)" == 0 ]]; then
    ip link set lo multicast on
    ip route replace 224.0.0.0/4 dev lo
else
    sudo ip link set lo multicast on
    sudo ip route replace 224.0.0.0/4 dev lo
fi

jobs="${DAIRLIB_BAZEL_JOBS:-8}"
ram="${DAIRLIB_BAZEL_RAM_MB:-14000}"
for value in "$jobs" "$ram"; do
    if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
        echo 'DAIRLIB_BAZEL_JOBS and DAIRLIB_BAZEL_RAM_MB must be positive integers.' >&2
        exit 2
    fi
done

# Docker initializes a new volume from the image, where this directory belongs
# to the image's nonroot user. A root-owned checkout runs as UID 0 instead.
# Bazel requires the cache root to belong to its effective user, even when that
# user can otherwise write to it. Change only the mount root, never its contents.
bazel_cache=/home/dairlib/.cache/bazel
runtime_uid="$(id -u)"
if ! mkdir -p "$bazel_cache"; then
    echo "Cannot create Bazel cache $bazel_cache for UID $runtime_uid." >&2
    exit 2
fi
cache_uid="$(stat -c %u -- "$bazel_cache")"
if [[ "$cache_uid" != "$runtime_uid" ]]; then
    if [[ "$runtime_uid" == 0 ]]; then
        chown --no-dereference "0:$(id -g)" -- "$bazel_cache"
    else
        echo "Bazel cache $bazel_cache belongs to UID $cache_uid; running as UID $runtime_uid." >&2
        echo 'Use the default UID/GID-specific cache volume, or set DAIRLIB_CACHE_VOLUME to a volume owned by your container user.' >&2
        exit 2
    fi
fi

# This file lives in the disposable container, outside the host checkout.
# CFS quotas do not reliably change nproc, so explicitly cap Bazel concurrency.
cat > "$HOME/.bazelrc" <<EOF
startup --output_user_root=$bazel_cache
startup --host_jvm_args=-Xmx2g
build --jobs=$jobs
build --local_ram_resources=$ram
build --verbose_failures
EOF

if [[ $# -eq 0 ]]; then set -- bash; fi
exec "$@"
