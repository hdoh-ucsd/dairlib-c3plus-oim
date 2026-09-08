#!/usr/bin/env bash
# Pair-2..5 campaign: 24 trials (6 scenes x 4 pairs) through N parallel lanes,
# each fully isolated (unique udpm port per job, per-run dirs/tmp/pgroups).
# Simulations only; packaging is a separate pass. Usage: run_campaign_pairs.sh [CAP] [LANES]
set -u
CAP="${1:-200}"; LANES="${2:-5}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
S="$WT/tools/scene_smoke"
R="$WT/results/xarm6_c3plus_scene_smoke/runs"

# scene demo_prefix gx gy gyaw envfile  (per pair: goal from the tables below)
declare -A GOALS
GOALS[open_task_2]="0.397 -0.431 3.2400";      GOALS[open_task_3]="0.404 -0.393 2.7924"
GOALS[open_task_4]="0.386 -0.424 2.7704";      GOALS[open_task_5]="0.359 -0.424 2.7338"
GOALS[single_obstacle_2]="0.397 -0.431 3.2400";GOALS[single_obstacle_3]="0.404 -0.393 2.7924"
GOALS[single_obstacle_4]="0.386 -0.424 2.7704";GOALS[single_obstacle_5]="0.359 -0.424 2.7338"
GOALS[shelf_gap_2]="0.397 -0.431 3.2400";      GOALS[shelf_gap_3]="0.404 -0.393 2.7924"
GOALS[shelf_gap_4]="0.386 -0.424 2.7704";      GOALS[shelf_gap_5]="0.359 -0.424 2.7338"
GOALS[ycb_clutter_2]="0.397 -0.431 3.2400";    GOALS[ycb_clutter_3]="0.404 -0.393 2.7924"
GOALS[ycb_clutter_4]="0.386 -0.424 2.7704";    GOALS[ycb_clutter_5]="0.359 -0.424 2.7338"
GOALS[slalom_2]="0.404 -0.393 2.7924";         GOALS[slalom_3]="0.363 -0.410 3.5684"
GOALS[slalom_4]="0.367 -0.380 2.7338";         GOALS[slalom_5]="0.404 -0.418 3.3892"
GOALS[icra_sign_2]="0.523 -0.4262 1.6692";     GOALS[icra_sign_3]="0.5247 -0.4041 1.2708"
GOALS[icra_sign_4]="0.5107 -0.4221 1.1996";    GOALS[icra_sign_5]="0.4845 -0.4286 1.1630"

declare -A DEMO ENVF
DEMO[open_task]="matched_open_table_xarm6_t";        ENVF[open_task]=""
DEMO[single_obstacle]="matched_single_obstacle_xarm6_t"; ENVF[single_obstacle]="$S/env_single_obstacle.sh"
DEMO[shelf_gap]="matched_shelf_gap_xarm6_t";         ENVF[shelf_gap]="$S/env_shelf_gap.sh"
DEMO[ycb_clutter]="matched_ycb_clutter_xarm6_t";     ENVF[ycb_clutter]="$S/env_ycb_clutter.sh"
DEMO[slalom]="matched_slalom_xarm6_t";               ENVF[slalom]="$S/env_slalom.sh"
DEMO[icra_sign]="anything_icra_c_matched_xarm6_t";   ENVF[icra_sign]="$S/env_icra_sign.sh"

PORT=7840
JOBS=()
for scene in open_task single_obstacle shelf_gap ycb_clutter slalom icra_sign; do
  for i in 2 3 4 5; do
    JOBS+=("$scene $i")
  done
done

running=0
for job in "${JOBS[@]}"; do
  scene="${job% *}"; i="${job#* }"
  out="$R/$scene/pair0$i"; mkdir -p "$out"
  goal="${GOALS[${scene}_${i}]}"
  PORT=$((PORT + 1))
  echo "[LAUNCH] $scene pair$i port $PORT goal $goal"
  bash "$S/run_scene_smoke.sh" "${DEMO[$scene]}$i" G_shape_video $goal \
    "$CAP" "$PORT" "$out" "${ENVF[$scene]}" > "$out/launcher.log" 2>&1 &
  running=$((running + 1))
  if [ "$running" -ge "$LANES" ]; then
    wait -n
    running=$((running - 1))
  fi
done
wait
echo "CAMPAIGN PAIRS DONE"
