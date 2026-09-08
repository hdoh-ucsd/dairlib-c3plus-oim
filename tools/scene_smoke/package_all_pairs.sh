#!/usr/bin/env bash
# Package pair-2..5 runs for the scenes named as args (default: all six),
# LANES parallel packagings at a time.
set -u
LANES="${LANES:-6}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
S="$WT/tools/scene_smoke"
U="$WT/examples/sampling_c3/urdf"
declare -A GOALS
GOALS[open_task_02]="0.397 -0.431 3.2400";      GOALS[open_task_03]="0.404 -0.393 2.7924"
GOALS[open_task_04]="0.386 -0.424 2.7704";      GOALS[open_task_05]="0.359 -0.424 2.7338"
GOALS[single_obstacle_02]="0.397 -0.431 3.2400";GOALS[single_obstacle_03]="0.404 -0.393 2.7924"
GOALS[single_obstacle_04]="0.386 -0.424 2.7704";GOALS[single_obstacle_05]="0.359 -0.424 2.7338"
GOALS[shelf_gap_02]="0.397 -0.431 3.2400";      GOALS[shelf_gap_03]="0.404 -0.393 2.7924"
GOALS[shelf_gap_04]="0.386 -0.424 2.7704";      GOALS[shelf_gap_05]="0.359 -0.424 2.7338"
GOALS[ycb_clutter_02]="0.397 -0.431 3.2400";    GOALS[ycb_clutter_03]="0.404 -0.393 2.7924"
GOALS[ycb_clutter_04]="0.386 -0.424 2.7704";    GOALS[ycb_clutter_05]="0.359 -0.424 2.7338"
GOALS[slalom_02]="0.404 -0.393 2.7924";         GOALS[slalom_03]="0.363 -0.410 3.5684"
GOALS[slalom_04]="0.367 -0.380 2.7338";         GOALS[slalom_05]="0.404 -0.418 3.3892"
GOALS[icra_sign_02]="0.523 -0.4262 1.6692";     GOALS[icra_sign_03]="0.5247 -0.4041 1.2708"
GOALS[icra_sign_04]="0.5107 -0.4221 1.1996";    GOALS[icra_sign_05]="0.4845 -0.4286 1.1630"
declare -A DEMO OBS OBJ
DEMO[open_task]="matched_open_table_xarm6_t";        OBS[open_task]="";                              OBJ[open_task]=""
DEMO[single_obstacle]="matched_single_obstacle_xarm6_t"; OBS[single_obstacle]="$U/single_obstacle_box_oimframe.sdf"; OBJ[single_obstacle]=""
DEMO[shelf_gap]="matched_shelf_gap_xarm6_t";         OBS[shelf_gap]="$U/scene_shelf_gap_oimframe.sdf"; OBJ[shelf_gap]=""
DEMO[ycb_clutter]="matched_ycb_clutter_xarm6_t";     OBS[ycb_clutter]="$U/scene_ycb_clutter_oimframe.sdf"; OBJ[ycb_clutter]=""
DEMO[slalom]="matched_slalom_xarm6_t";               OBS[slalom]="$U/scene_slalom_oimframe.sdf";     OBJ[slalom]=""
DEMO[icra_sign]="anything_icra_c_matched_xarm6_t";   OBS[icra_sign]="$U/scene_icra_sign.sdf";        OBJ[icra_sign]="$U/push_c_glyph.sdf"
SCENES=("$@"); [ ${#SCENES[@]} -eq 0 ] && SCENES=(open_task single_obstacle shelf_gap ycb_clutter slalom icra_sign)
running=0
for scene in "${SCENES[@]}"; do
  for p in 02 03 04 05; do
    bash "$S/package_run.sh" "$scene" "${DEMO[$scene]}${p#0}" \
      ${GOALS[${scene}_${p}]} "${OBS[$scene]}" "${OBJ[$scene]}" "$p" \
      > "$WT/results/xarm6_c3plus_scene_smoke/runs/$scene/pair$p/package.log" 2>&1 &
    running=$((running + 1))
    if [ "$running" -ge "$LANES" ]; then wait -n; running=$((running - 1)); fi
  done
done
wait
grep -l "PACKAGE DONE" "$WT"/results/xarm6_c3plus_scene_smoke/runs/*/pair0*/package.log | wc -l
echo "PACKAGING PASS DONE"
