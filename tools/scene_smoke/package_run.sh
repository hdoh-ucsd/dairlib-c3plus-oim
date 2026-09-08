#!/usr/bin/env bash
# Post-run packaging: metrics CSV + eval graph + result JSON + manifest + mp4.
# Usage: package_run.sh SCENE DEMO GX GY GYAW [OBSTACLE_SDF] [OBJECT_SDF]
set -uo pipefail
SCENE="${1:?}"; DEMO="${2:?}"; GX="$3"; GY="$4"; GYAW="$5"
OBS_SDF="${6:-}"; OBJ_SDF="${7:-}"; PAIR="${8:-01}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
PY=/root/miniconda3/envs/push_anything_ADMM/bin/python3
R="$WT/results/xarm6_c3plus_scene_smoke/runs/$SCENE"
{ [ "$PAIR" != "01" ] || [ -d "$R/pair01" ]; } && R="$R/pair$PAIR"
RUNID="xarm6_c3plus_${SCENE}_s${PAIR}_to_g${PAIR}"
[ -z "$OBJ_SDF" ] && OBJ_SDF="$WT/examples/sampling_c3/urdf/push_t_oimscale_m01.sdf"
"$PY" "$WT/tools/scene_smoke/postprocess_run.py" \
  --run-dir "$R" --scene "$SCENE" --run-id "$RUNID" \
  --scene-config "$WT/tools/scene_smoke/scene_configs/$SCENE.yaml" \
  --demo "$DEMO" 2>&1 | tail -6
OBS_ARG=()
[ -n "$OBS_SDF" ] && OBS_ARG=(--obstacle-sdf "$OBS_SDF")
"$PY" "$WT/tools/scene_smoke/render_run_3d.py" \
  --trace "$R/state_trace.jsonl" --out "$R/$RUNID.mp4" \
  --object-sdf "$OBJ_SDF" "${OBS_ARG[@]}" \
  --goal "$GX" "$GY" "$GYAW" --title "$RUNID" 2>&1 | tail -2
ls -la "$R" | grep "$RUNID"
echo "PACKAGE DONE $SCENE"
