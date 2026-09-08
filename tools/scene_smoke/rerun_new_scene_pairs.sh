#!/usr/bin/env bash
# Rerun the 12 ycb/slalom/icra pair trials (regenerated dirs), 5 lanes.
set -u
CAP="${1:-200}"; LANES="${2:-5}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
S="$WT/tools/scene_smoke"
R="$WT/results/xarm6_c3plus_scene_smoke/runs"
declare -A GOALS
GOALS[ycb_clutter_2]="0.397 -0.431 3.2400";    GOALS[ycb_clutter_3]="0.404 -0.393 2.7924"
GOALS[ycb_clutter_4]="0.386 -0.424 2.7704";    GOALS[ycb_clutter_5]="0.359 -0.424 2.7338"
GOALS[slalom_2]="0.404 -0.393 2.7924";         GOALS[slalom_3]="0.363 -0.410 3.5684"
GOALS[slalom_4]="0.367 -0.380 2.7338";         GOALS[slalom_5]="0.404 -0.418 3.3892"
GOALS[icra_sign_2]="0.523 -0.4262 1.6692";     GOALS[icra_sign_3]="0.5247 -0.4041 1.2708"
GOALS[icra_sign_4]="0.5107 -0.4221 1.1996";    GOALS[icra_sign_5]="0.4845 -0.4286 1.1630"
declare -A DEMO ENVF
DEMO[ycb_clutter]="matched_ycb_clutter_xarm6_t";   ENVF[ycb_clutter]="$S/env_ycb_clutter.sh"
DEMO[slalom]="matched_slalom_xarm6_t";             ENVF[slalom]="$S/env_slalom.sh"
DEMO[icra_sign]="anything_icra_c_matched_xarm6_t"; ENVF[icra_sign]="$S/env_icra_sign.sh"
PORT=7870
running=0
for scene in ycb_clutter slalom icra_sign; do
  for i in 2 3 4 5; do
    out="$R/$scene/pair0$i"; rm -rf "$out"; mkdir -p "$out"
    PORT=$((PORT + 1))
    echo "[LAUNCH] $scene pair$i port $PORT"
    bash "$S/run_scene_smoke.sh" "${DEMO[$scene]}$i" G_shape_video \
      ${GOALS[${scene}_${i}]} "$CAP" "$PORT" "$out" "${ENVF[$scene]}" \
      > "$out/launcher.log" 2>&1 &
    running=$((running + 1))
    if [ "$running" -ge "$LANES" ]; then wait -n; running=$((running - 1)); fi
  done
done
wait
echo "RERUN DONE"
