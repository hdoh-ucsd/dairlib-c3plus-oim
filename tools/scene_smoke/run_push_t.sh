#!/usr/bin/env bash
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${1:-$REPO/results/push_t_smoke}"
GOAL_X="${2:-0.0}"
GOAL_Y="${3:-0.0}"
GOAL_YAW="${4:-0.0}"

cd "$REPO"

# Build the minimal stack required for a single push-t smoke run.
# This avoids the many redundant demo-specific Launchers under examples/.
bazel build \
  //examples/sampling_c3:franka_sim \
  //examples/sampling_c3:franka_osc_controller \
  //examples/sampling_c3:franka_sampling_c3_controller

# Run one deterministic scene-smoke experiment. The runner writes a trace and
# a packaged output directory (including logs + state_trace.jsonl).
python3 tools/scene_smoke/run_experiment.py \
  --scene open_task \
  --variant baseline \
  --start 1 \
  --goal 1 \
  --out "$OUT"

# Render the recorded trace into a short mp4.
python3 tools/scene_smoke/render_run_3d.py \
  --trace "$OUT/state_trace.jsonl" \
  --out "$OUT/push_t_render.mp4" \
  --object-sdf "$REPO/examples/sampling_c3/urdf/push_t_oimscale_m01.sdf" \
  --goal "$GOAL_X" "$GOAL_Y" "$GOAL_YAW" \
  --title "push_t" \
  --fps 10 \
  --max-frames 1200

echo "Done. Experiment dir: $OUT"
echo "Rendered video: $OUT/push_t_render.mp4"
