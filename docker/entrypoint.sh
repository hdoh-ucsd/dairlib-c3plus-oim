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

# This file lives in the disposable container, outside the host checkout.
# CFS quotas do not reliably change nproc, so explicitly cap Bazel concurrency.
cat > "$HOME/.bazelrc" <<EOF
startup --output_user_root=/home/dairlib/.cache/bazel
startup --host_jvm_args=-Xmx2g
build --jobs=$jobs
build --local_ram_resources=$ram
build --verbose_failures
EOF

if [[ $# -eq 0 ]]; then set -- bash; fi
exec "$@"
