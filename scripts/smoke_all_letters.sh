#!/bin/bash
set -uo pipefail
cd "$(dirname "$0")/.."
PY=/root/miniconda3/envs/push_anything_ADMM/bin/python3
declare -A PORT=( [i]=8051 [c]=8052 [r]=8053 [a]=8054 )
for g in i c r a; do
  bash scripts/smoke_open_table_glyph.sh "$g" "${PORT[$g]}" > /tmp/smoke_run_$g.out 2>&1 &
done
sleep 2
for g in i c r a; do
  $PY scripts/watch_object_state.py "${PORT[$g]}" 19 > /tmp/watch_$g.log 2>&1 &
done
wait
for g in i c r a; do
  echo "===== $g ====="
  head -2 /tmp/watch_$g.log; tail -2 /tmp/watch_$g.log
  grep -E 'MATCHED_MU. body|Object index' /tmp/glyph_smoke_$g/sim.log | tail -3
  grep -v warning /tmp/glyph_smoke_$g/planner.log | tail -3
  tail -1 /tmp/glyph_smoke_$g/osc.log
done
