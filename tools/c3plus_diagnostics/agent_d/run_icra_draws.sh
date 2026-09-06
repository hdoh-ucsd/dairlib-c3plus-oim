#!/usr/bin/env bash
# Faithful icra_sign baseline runner. Usage: run_icra_draws.sh DEMO GX GY GYAW CAP OUT PORT
set -uo pipefail
DEMO="${1:?}"; GX="$2"; GY="$3"; GYAW="$4"; CAP="${5:-600}"; OUT="${6:?}"; PORT="${7:-7995}"
WT=/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-ycb-icra
BIN=$WT/bazel-bin/examples/sampling_c3
PY=/root/miniconda3/envs/push_anything_ADMM/bin/python3
URL="udpm://239.255.76.67:${PORT}?ttl=0"
cd "$WT"
export SAMPLING_C3_OBSTACLE_MODE=lcs_contact
export SAMPLING_C3_OBJECT_FOOTPRINT=c_glyph
export SAMPLING_C3_OBS_TOP_Z=0.046
SAMPLING_C3_OBS_POLYS="$(cat $WT/results_icra_port/obstacles/obs_polys_env.txt)"
export SAMPLING_C3_OBS_POLYS
mkdir -p "$OUT"
export SAMPLING_C3_COST_LOG_DIR="$OUT"
"$BIN/franka_osc_controller" --demo_name="$DEMO" --lcm_url="$URL" > "$OUT/osc.log" 2>&1 & OSC=$!
"$BIN/franka_sampling_c3_controller" --demo_name="$DEMO" --lcm_url="$URL" > "$OUT/planner.log" 2>&1 & PLAN=$!
"$PY" "$WT/tools/c3plus_diagnostics/agent_d/record_run.py" \
  --goal "$GX" "$GY" "$GYAW" --out "$OUT/state_trace.jsonl" \
  --url "$URL" --duration "$CAP" --exit-on-success > "$OUT/success.log" 2>&1 & REC=$!
sleep 3
"$BIN/franka_sim" --demo_name="$DEMO" --lcm_url="$URL" > "$OUT/sim.log" 2>&1 & SIM=$!
wait $REC
kill $SIM $OSC $PLAN 2>/dev/null; wait $SIM $OSC $PLAN 2>/dev/null
grep -h "FINAL\|SUCCESS" "$OUT/success.log" | tail -2
