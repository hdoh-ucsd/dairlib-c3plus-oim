#!/usr/bin/env bash
set -Eeuo pipefail

# One-command Docker runner for the three active C3+/xArm6 demos.
#
# Usage:
#   ./run_c3.sh open_table
#   ./run_c3.sh single_obstacle
#   ./run_c3.sh shelf_gap
#   ./run_c3.sh open_table 42
#
# Run this from the WSL HOST, not from inside Docker.

REPO="${C3_REPO:-/root/dairlib-oim-c3plus-baseline}"
IMAGE="${C3_IMAGE:-dairlib-c3plus-oim:mplns-fixed-mwpeco}"
EXPECTED_IMAGE_ID="${C3_EXPECTED_IMAGE_ID:-sha256:35210d15124a5009892e37530683ea95c2e73e26d7352346bae676cb7b99ddb2}"
CACHE="${C3_BAZEL_CACHE:-c3-bazel-root-2d3fa9461272}"
MESHCAT_PORT="${MESHCAT_PORT:-7000}"
CPUS="${C3_CPUS:-4}"
MEMORY="${C3_MEMORY:-16g}"
MEMORY_SWAP="${C3_MEMORY_SWAP:-20g}"
SEED="${2:-42}"
SCENARIO="${1:-}"

usage() {
  cat <<'EOF'
Usage:
  ./run_c3.sh open_table [seed]
  ./run_c3.sh single_obstacle [seed]
  ./run_c3.sh shelf_gap [seed]

Examples:
  ./run_c3.sh open_table
  ./run_c3.sh single_obstacle 42
  MESHCAT_PORT=7001 ./run_c3.sh shelf_gap 42
EOF
}

case "$SCENARIO" in
  open_table)       DEMO="push_t_bt010_open_table_xarm6" ;;
  single_obstacle)  DEMO="push_t_bt010_single_obstacle_xarm6" ;;
  shelf_gap)        DEMO="push_t_bt010_shelf_gap_xarm6" ;;
  -h|--help|"") usage; exit 0 ;;
  *) echo "[ERROR] Unknown scenario: $SCENARIO" >&2; usage >&2; exit 2 ;;
esac

[[ "$SEED" =~ ^[0-9]+$ ]] || { echo "[ERROR] Seed must be a non-negative integer." >&2; exit 2; }
command -v docker >/dev/null || { echo "[ERROR] docker is unavailable." >&2; exit 1; }
[[ -d "$REPO/.git" ]] || { echo "[ERROR] Repository not found: $REPO" >&2; exit 1; }

ACTUAL_IMAGE_ID="$(docker image inspect --format '{{.Id}}' "$IMAGE" 2>/dev/null || true)"
[[ -n "$ACTUAL_IMAGE_ID" ]] || { echo "[ERROR] Docker image not found: $IMAGE" >&2; exit 1; }
[[ "$ACTUAL_IMAGE_ID" == "$EXPECTED_IMAGE_ID" ]] || {
  echo "[ERROR] Image identity changed." >&2
  echo "Expected: $EXPECTED_IMAGE_ID" >&2
  echo "Actual:   $ACTUAL_IMAGE_ID" >&2
  exit 1
}

PARAMS="$REPO/examples/sampling_c3/$DEMO/parameters/sampling_c3_controller_params.yaml"
[[ -f "$PARAMS" ]] || { echo "[ERROR] Demo configuration missing: $PARAMS" >&2; exit 1; }

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_ID="${SCENARIO}_seed${SEED}_${STAMP}"
RESULT_DIR="$REPO/results/docker_smoke/$RUN_ID"
mkdir -p "$RESULT_DIR"

git -C "$REPO" rev-parse HEAD > "$RESULT_DIR/git_commit.txt"
git -C "$REPO" status --short > "$RESULT_DIR/git_status_before.txt"
printf '%s\n' "$ACTUAL_IMAGE_ID" > "$RESULT_DIR/docker_image_id.txt"
printf '%s\n' "$DEMO" > "$RESULT_DIR/demo_name.txt"
printf '%s\n' "$SEED" > "$RESULT_DIR/seed.txt"

CONTAINER="c3-${SCENARIO//_/-}-${STAMP}"
LCM_URL='udpm://239.255.76.67:7667?ttl=0'

cleanup_host() {
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup_host EXIT INT TERM HUP

echo "============================================================"
echo "C3+ Docker experiment"
echo "Scenario : $SCENARIO"
echo "Demo     : $DEMO"
echo "Seed     : $SEED"
echo "Image    : $ACTUAL_IMAGE_ID"
echo "Results  : $RESULT_DIR"
echo "Meshcat  : http://localhost:$MESHCAT_PORT"
echo "============================================================"

docker run --rm -i \
  --name "$CONTAINER" \
  --hostname c3-container \
  --user 0:0 \
  --network bridge \
  --cap-add NET_ADMIN \
  -p "127.0.0.1:${MESHCAT_PORT}:7000" \
  --mount "type=bind,src=$REPO,dst=/home/dairlib/dairlib" \
  --mount "type=volume,src=$CACHE,dst=/home/dairlib/.cache/bazel,volume-nocopy" \
  -e HOME=/home/dairlib \
  -e USER=root \
  -e LOGNAME=root \
  -e "LCM_DEFAULT_URL=$LCM_URL" \
  -e "SAMPLING_C3_SEED=$SEED" \
  -e "C3_DEMO=$DEMO" \
  -e "C3_RUN_ID=$RUN_ID" \
  --cpus "$CPUS" \
  --memory "$MEMORY" \
  --memory-swap "$MEMORY_SWAP" \
  --shm-size 2g \
  -w /home/dairlib/dairlib \
  "$IMAGE" \
  /bin/bash -s <<'INNER'
set -Eeuo pipefail

DEMO="$C3_DEMO"
RUN="/home/dairlib/dairlib/results/docker_smoke/$C3_RUN_ID"
LCM_URL="$LCM_DEFAULT_URL"

SIM_LOG="$RUN/sim.log"
OSC_LOG="$RUN/osc.log"
C3_LOG="$RUN/c3.log"
STATUS_LOG="$RUN/process_status.txt"

SIM_PID=""
OSC_PID=""
C3_PID=""

echo "[0/3] Ensuring experiment binaries are built inside Docker..."
bazel build \
  --jobs=4 \
  --local_resources=memory=10000 \
  //examples/sampling_c3:franka_sim \
  //examples/sampling_c3:franka_osc_controller \
  //examples/sampling_c3:franka_sampling_c3_controller \
  >"$RUN/build.log" 2>&1

BAZEL_BIN="$(bazel info bazel-bin)"
SIM_EXE="$BAZEL_BIN/examples/sampling_c3/franka_sim"
OSC_EXE="$BAZEL_BIN/examples/sampling_c3/franka_osc_controller"
C3_EXE="$BAZEL_BIN/examples/sampling_c3/franka_sampling_c3_controller"

for exe in "$SIM_EXE" "$OSC_EXE" "$C3_EXE"; do
  [[ -x "$exe" ]] || {
    echo "[FAIL] Built executable not found: $exe"
    tail -n 120 "$RUN/build.log" || true
    exit 9
  }
done
echo "      build/cache check passed"

cleanup_children() {
  set +e
  for pid in "$C3_PID" "$OSC_PID" "$SIM_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "$C3_PID" "$OSC_PID" "$SIM_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup_children EXIT INT TERM HUP

echo "[1/3] Starting simulator..."
stdbuf -oL -eL \
  "$SIM_EXE" \
    --demo_name="$DEMO" \
    --robot_model=xarm6 \
    --matched_mu=true \
    --lcm_url="$LCM_URL" \
  >"$SIM_LOG" 2>&1 &
SIM_PID=$!

sleep 3
if ! kill -0 "$SIM_PID" 2>/dev/null; then
  echo "[FAIL] Simulator exited during startup."
  cat "$SIM_LOG"
  exit 10
fi
echo "      simulator pid=$SIM_PID"

echo "[2/3] Starting xArm6 OSC/executor..."
stdbuf -oL -eL \
  "$OSC_EXE" \
    --demo_name="$DEMO" \
    --robot_model=xarm6 \
    --is_simulation=true \
    --xarm6_five_joint=true \
    --prelift_release=true \
    --lcm_url="$LCM_URL" \
  >"$OSC_LOG" 2>&1 &
OSC_PID=$!

sleep 2
if ! kill -0 "$OSC_PID" 2>/dev/null; then
  echo "[FAIL] OSC controller exited during startup."
  cat "$OSC_LOG"
  exit 11
fi
echo "      OSC pid=$OSC_PID"

echo "[3/3] Starting C3+ planner (seed=$SAMPLING_C3_SEED)..."
stdbuf -oL -eL \
  "$C3_EXE" \
    --demo_name="$DEMO" \
    --robot_model=xarm6 \
    --is_simulation=true \
    --lcm_url="$LCM_URL" \
  >"$C3_LOG" 2>&1 &
C3_PID=$!
echo "      C3+ pid=$C3_PID"

printf 'sim_pid=%s\nosc_pid=%s\nc3_pid=%s\n' \
  "$SIM_PID" "$OSC_PID" "$C3_PID" > "$STATUS_LOG"

echo
echo "[RUNNING] All three processes started."
echo "[RUNNING] Press Ctrl+C to stop."
echo "[RUNNING] Logs:"
echo "          $SIM_LOG"
echo "          $OSC_LOG"
echo "          $C3_LOG"

EXITED_PID=""
set +e
wait -n -p EXITED_PID "$SIM_PID" "$OSC_PID" "$C3_PID"
RC=$?
set -e

case "$EXITED_PID" in
  "$SIM_PID") WHO="simulator" ;;
  "$OSC_PID") WHO="osc" ;;
  "$C3_PID")  WHO="c3" ;;
  *)          WHO="unknown" ;;
esac

printf 'first_exited=%s\nexit_code=%s\n' "$WHO" "$RC" >> "$STATUS_LOG"
echo
echo "[STOP] First process to exit: $WHO (code=$RC)"
echo "[STOP] Remaining experiment processes will be terminated."

case "$WHO" in
  simulator) tail -n 80 "$SIM_LOG" ;;
  osc)       tail -n 80 "$OSC_LOG" ;;
  c3)        tail -n 80 "$C3_LOG" ;;
esac

exit "$RC"
INNER
RC=$?

git -C "$REPO" status --short > "$RESULT_DIR/git_status_after.txt" || true
printf '%s\n' "$RC" > "$RESULT_DIR/runner_exit_code.txt"

echo
echo "Run record: $RESULT_DIR"
exit "$RC"
