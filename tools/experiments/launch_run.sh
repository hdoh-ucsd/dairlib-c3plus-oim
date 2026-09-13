#!/usr/bin/env bash
# One isolated experiment: sim + OSC + C3+ trio plus per-step recorder.
# Usage: launch_run.sh DEMO OBJNAME GX GY GYAW CAP PORT OUTDIR [--controller-params YAML] [--goal-yaw-degrees DEGREES]
# run_experiment.py supplies the planner environment derived from the scene YAML.
set -uo pipefail
if [[ $# -lt 8 ]]; then
  echo "Usage: launch_run.sh DEMO OBJNAME GX GY GYAW CAP PORT OUTDIR [--controller-params YAML] [--goal-yaw-degrees DEGREES]" >&2
  exit 2
fi
DEMO="${1:?}"; OBJ="${2:?}"; GX="$3"; GY="$4"; GYAW="$5"
CAP="${6:-120}"; PORT="${7:?}"; OUT="${8:?}"
shift 8
CONTROLLER_PARAMS=""; GOAL_YAW=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --controller-params|--goal-yaw-degrees)
      [[ $# -ge 2 && -n "$2" ]] || { echo "Missing value for $1" >&2; exit 2; }
      if [[ "$1" == --controller-params ]]; then CONTROLLER_PARAMS="$2"; else GOAL_YAW="$2"; fi
      shift 2 ;;
    *) echo "Unknown launch option: $1" >&2; exit 2 ;;
  esac
done
WT="$(cd "$(dirname "$0")/../.." && pwd)"
# Build the three targets from this checkout before launching.
BIN="$WT/bazel-bin/examples/sampling_c3"
PY="${PYTHON:-python3}"
URL="udpm://239.255.76.67:${PORT}?ttl=0"
planner_goal_args=()
controller_args=()
if [[ -n "$CONTROLLER_PARAMS" ]]; then
  [[ -f "$CONTROLLER_PARAMS" ]] || { echo "Missing controller YAML: $CONTROLLER_PARAMS" >&2; exit 2; }
  CONTROLLER_PARAMS="$(cd "$(dirname "$CONTROLLER_PARAMS")" && pwd)/$(basename "$CONTROLLER_PARAMS")"
  for name in franka_sim franka_osc_controller franka_sampling_c3_controller; do
    native_help="$("$BIN/$name" --helpshort 2>&1)"
    if ! grep -Eq -- '(^|[[:space:]])-controller_params([[:space:]]|=)' <<< "$native_help"; then
      echo "$name does not support --controller_params. Rebuild with python3 -m tools.experiments build before launching." >&2
      exit 2
    fi
  done
  controller_args=("--controller_params=$CONTROLLER_PARAMS")
fi
if [[ -n "$GOAL_YAW" ]]; then
  if [[ ! "$GOAL_YAW" =~ ^([+-]?90|[+-]?0)([.]0+)?$ ]]; then
    echo "GOAL_YAW_DEGREES must be -90, 0, or 90 (absolute world yaw)." >&2
    exit 2
  fi
  # Gflags prints help and exits before constructing any controller systems.
  # Some versions exit nonzero for help, so inspect the text independently.
  controller_help="$("$BIN/franka_sampling_c3_controller" --helpshort 2>&1)"
  if ! grep -Eq -- '(^|[[:space:]])-goal_yaw_degrees([[:space:]]|=)' <<< "$controller_help"; then
    echo "Controller does not support --goal_yaw_degrees. Rebuild with python3 -m tools.experiments build before launching this campaign." >&2
    exit 2
  fi
  planner_goal_args=("--goal_yaw_degrees=$GOAL_YAW")
fi
mkdir -p "$OUT" || exit 1
OUT="$(cd "$OUT" && pwd)"
cd "$WT"   # parameter yamls resolve relative to cwd -> worktree copies
export TMPDIR="$OUT/tmp"; mkdir -p "$TMPDIR"
export SAMPLING_C3_OBSTACLE_MODE="${SAMPLING_C3_OBSTACLE_MODE:-lcs_contact}"

SIM=""; OSC=""; PLAN=""; REC=""
cleanup() {
  for pg in "$SIM" "$OSC" "$PLAN" "$REC"; do
    [ -z "$pg" ] || kill -- -"$pg" 2>/dev/null || true
  done
  sleep 1
  for pg in "$SIM" "$OSC" "$PLAN" "$REC"; do
    [ -z "$pg" ] || kill -9 -- -"$pg" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid "$BIN/franka_osc_controller" --is_simulation=true --demo_name="$DEMO" \
  --robot_model=xarm6 --lcm_url="$URL" "${controller_args[@]}" > "$OUT/osc.log" 2>&1 & OSC=$!
setsid "$BIN/franka_sampling_c3_controller" --is_simulation=true --demo_name="$DEMO" \
  --robot_model=xarm6 --lcm_url="$URL" "${controller_args[@]}" "${planner_goal_args[@]}" > "$OUT/planner.log" 2>&1 & PLAN=$!
setsid "$PY" "$WT/tools/experiments/record_metrics.py" \
  --goal "$GX" "$GY" "$GYAW" --object-name "$OBJ" \
  --out-steps "$OUT/steps_raw.jsonl" --out-trace "$OUT/state_trace.jsonl" \
  --url "$URL" --duration "$CAP" --exit-on-success \
  > "$OUT/recorder.log" 2>&1 & REC=$!
sleep 3
setsid "$BIN/franka_sim" --demo_name="$DEMO" --robot_model=xarm6 --matched_mu \
  --lcm_url="$URL" "${controller_args[@]}" > "$OUT/sim.log" 2>&1 & SIM=$!
wait "$REC"
recorder_rc=$?
grep -h "SUCCESS\|FINAL" "$OUT/recorder.log" | tail -2
echo "RUN DONE $DEMO -> $OUT"
exit "$recorder_rc"
