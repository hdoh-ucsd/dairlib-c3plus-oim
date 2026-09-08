#!/usr/bin/env bash
# Serial (unloaded) rerun of open_task pairs 2-5: planner-starvation probe.
set -u
CAP="${1:-200}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
S="$WT/tools/scene_smoke"
R="$WT/results/xarm6_c3plus_scene_smoke/runs/open_task"
declare -A GOALS
GOALS[2]="0.397 -0.431 3.2400"; GOALS[3]="0.404 -0.393 2.7924"
GOALS[4]="0.386 -0.424 2.7704"; GOALS[5]="0.359 -0.424 2.7338"
for i in 2 3 4 5; do
  out="$R/solo_pair0$i"; rm -rf "$out"; mkdir -p "$out"
  echo "[SOLO] open_task pair$i"
  bash "$S/run_scene_smoke.sh" "matched_open_table_xarm6_t$i" G_shape_video \
    ${GOALS[$i]} "$CAP" $((7900 + i)) "$out" "" 2>&1 | tail -2
done
echo "SOLO PROBE DONE"
