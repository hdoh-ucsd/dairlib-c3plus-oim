#!/usr/bin/env bash
# Package one grid run of the baseline-vs-ReLU comparison: metrics CSV +
# result JSON + mp4 + cost-diagnostics figure, into the run dir itself.
#
# Usage: package_grid_run.sh VARIANT SCENE SXXGYY [DEMO] [GX GY GYAW]
#   VARIANT  baseline|relu
#   SCENE    open_task|single_obstacle|shelf_gap|ycb_clutter|icra_sign|slalom
#   SXXGYY   run dir name, e.g. s01g01
#   DEMO     optional demo name (derived from scene/start/goal if omitted)
#   GX GY GYAW  optional goal pose (read from the goal demo's
#               goal_params.yaml if omitted)
# For --variant relu the cost figure labels the obstacle curve
# 'obstacle_ReLU' (200*max(0,(0.01-d_fp)/0.01)^2, footprint distance).
set -uo pipefail
VARIANT="${1:?variant}"; SCENE="${2:?scene}"; SG="${3:?sMMgNN}"
DEMO="${4:-}"; GX="${5:-}"; GY="${6:-}"; GYAW="${7:-}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
PY=/root/miniconda3/envs/push_anything_ADMM/bin/python3
S="$WT/tools/scene_smoke"
U="$WT/examples/sampling_c3/urdf"
R="$WT/results/c3plus_relu_chomp_comparison/$VARIANT/$SCENE/$SG"
[ -d "$R" ] || { echo "no run dir $R"; exit 1; }
[ -s "$R/state_trace.jsonl" ] || { echo "incomplete run (no state trace), skipping: $R"; exit 0; }
grep -q "^RUN DONE" "$R/launcher.log" 2>/dev/null || { echo "run still in progress, skipping: $R"; exit 0; }

SM="${SG:1:2}"; GN="${SG:4:2}"; M=$((10#$SM)); N=$((10#$GN))

declare -A FAM OBS OBJ
FAM[open_task]="matched_open_table_xarm6_";        OBS[open_task]=""
FAM[single_obstacle]="matched_single_obstacle_xarm6_"; OBS[single_obstacle]="$U/single_obstacle_box_oimframe.sdf"
FAM[shelf_gap]="matched_shelf_gap_xarm6_";         OBS[shelf_gap]="$U/scene_shelf_gap_oimframe.sdf"
FAM[ycb_clutter]="matched_ycb_clutter_xarm6_";     OBS[ycb_clutter]="$U/scene_ycb_clutter_oimframe.sdf"
FAM[slalom]="matched_slalom_xarm6_";               OBS[slalom]="$U/scene_slalom_oimframe.sdf"
FAM[icra_sign]="anything_icra_c_matched_xarm6_";   OBS[icra_sign]="$U/scene_icra_sign.sdf"
OBJ_SDF="$U/push_t_oimscale_m01.sdf"
[ "$SCENE" = "icra_sign" ] && OBJ_SDF="$U/push_c_glyph.sdf"

if [ -z "$DEMO" ]; then
  if [ "$M" -eq "$N" ]; then DEMO="${FAM[$SCENE]}t$M"; else DEMO="${FAM[$SCENE]}s${M}g${N}"; fi
fi
if [ -z "$GX" ]; then
  read -r GX GY GYAW < <("$PY" - "$SCENE" "$N" <<'EOF'
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])) or ".",))
sys.path.insert(0, "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/oim-scene-sync-metrics/tools/relu_chomp")
import common
x, y, yaw = common.goal_of(sys.argv[1], int(sys.argv[2]))
print(f"{x} {y} {yaw:.6f}")
EOF
)
fi

# baseline keeps the legacy RUNID (diagonal symlinked runs are already
# packaged under that name); relu gets a variant-tagged one.
if [ "$VARIANT" = "baseline" ]; then
  RUNID="xarm6_c3plus_${SCENE}_s${SM}_to_g${GN}"
else
  RUNID="xarm6_c3plus_${VARIANT}_${SCENE}_s${SM}_to_g${GN}"
fi
echo "packaging $R as $RUNID (demo=$DEMO goal=$GX $GY $GYAW)"

"$PY" "$S/postprocess_run.py" \
  --run-dir "$R" --scene "$SCENE" --run-id "$RUNID" \
  --scene-config "$S/scene_configs/$SCENE.yaml" \
  --demo "$DEMO" 2>&1 | tail -4

OBS_ARG=()
[ -n "${OBS[$SCENE]}" ] && OBS_ARG=(--obstacle-sdf "${OBS[$SCENE]}")
[ -f "$R/$RUNID.mp4" ] || "$PY" "$S/render_run_3d.py" \
  --trace "$R/state_trace.jsonl" --out "$R/$RUNID.mp4" \
  --object-sdf "$OBJ_SDF" "${OBS_ARG[@]}" \
  --goal "$GX" "$GY" "$GYAW" --title "$RUNID" 2>&1 | tail -2

"$PY" "$WT/tools/relu_chomp/cost_fig.py" \
  --run-dir "$R" --scene "$SCENE" --variant "$VARIANT" 2>&1 | tail -1

ls -la "$R" | grep -E "$RUNID|cost_diagnostics" || true
echo "PACKAGE DONE $VARIANT/$SCENE/$SG"
