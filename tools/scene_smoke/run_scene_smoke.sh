#!/usr/bin/env bash
# One isolated scene-smoke trial: sim + OSC + C3+ trio plus per-step recorder.
# Usage: run_scene_smoke.sh DEMO OBJNAME GX GY GYAW CAP PORT OUTDIR [EXTRA_ENV_FILE]
# EXTRA_ENV_FILE (optional): sourced before launch (e.g. ICRA obstacle env).
set -uo pipefail
DEMO="${1:?}"; OBJ="${2:?}"; GX="$3"; GY="$4"; GYAW="$5"
CAP="${6:-120}"; PORT="${7:?}"; OUT="${8:?}"; ENVF="${9:-}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN=/root/push_anything_ADMM/external/oim_c++_anything/bazel-bin/examples/sampling_c3
PY=/root/miniconda3/envs/push_anything_ADMM/bin/python3
URL="udpm://239.255.76.67:${PORT}?ttl=0"
mkdir -p "$OUT"
cd "$WT"   # parameter yamls resolve relative to cwd -> worktree copies
export TMPDIR="$OUT/tmp"; mkdir -p "$TMPDIR"
export SAMPLING_C3_OBSTACLE_MODE="${SAMPLING_C3_OBSTACLE_MODE:-lcs_contact}"
if [ -n "$ENVF" ]; then set -a; . "$ENVF"; set +a; fi

setsid "$BIN/franka_osc_controller" --demo_name="$DEMO" --robot_model=xarm6 \
  --lcm_url="$URL" > "$OUT/osc.log" 2>&1 & OSC=$!
setsid "$BIN/franka_sampling_c3_controller" --demo_name="$DEMO" --robot_model=xarm6 \
  --lcm_url="$URL" > "$OUT/planner.log" 2>&1 & PLAN=$!
setsid "$PY" "$WT/tools/scene_smoke/record_metrics.py" \
  --goal "$GX" "$GY" "$GYAW" --object-name "$OBJ" \
  --out-steps "$OUT/steps_raw.jsonl" --out-trace "$OUT/state_trace.jsonl" \
  --url "$URL" --duration "$CAP" --exit-on-success \
  > "$OUT/recorder.log" 2>&1 & REC=$!
sleep 3
setsid "$BIN/franka_sim" --demo_name="$DEMO" --robot_model=xarm6 --matched_mu \
  --lcm_url="$URL" > "$OUT/sim.log" 2>&1 & SIM=$!
wait $REC
for pg in $SIM $OSC $PLAN; do kill -- -"$pg" 2>/dev/null; done
sleep 1
for pg in $SIM $OSC $PLAN; do kill -9 -- -"$pg" 2>/dev/null; done
grep -h "SUCCESS\|FINAL" "$OUT/recorder.log" | tail -2
echo "RUN DONE $DEMO -> $OUT"
