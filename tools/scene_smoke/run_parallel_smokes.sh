#!/usr/bin/env bash
# Launch the five remaining scene smokes concurrently, each fully isolated
# (unique LCM port, output dir, temp dir, process group via setsid in the
# per-run script). Usage: run_parallel_smokes.sh CAP_SECONDS
set -u
CAP="${1:-200}"
WT="$(cd "$(dirname "$0")/../.." && pwd)"
R="$WT/results/xarm6_c3plus_scene_smoke/runs"
S="$WT/tools/scene_smoke"
run() { # scene demo gx gy gyaw port envfile
  bash "$S/run_scene_smoke.sh" "$2" G_shape_video "$3" "$4" "$5" "$CAP" "$6" \
    "$R/$1" "${7:-}" > "$R/$1/launcher.log" 2>&1 &
  echo "launched $1 pid $! port $6"
}
mkdir -p "$R"/{single_obstacle,shelf_gap,ycb_clutter,icra_sign,slalom}
run single_obstacle matched_single_obstacle_xarm6_t1 0.381 -0.4 3.1416 7811 "$S/env_single_obstacle.sh"
run shelf_gap       matched_shelf_gap_xarm6_t1       0.381 -0.4 3.1416 7812 "$S/env_shelf_gap.sh"
run ycb_clutter     matched_ycb_clutter_xarm6_t1     0.381 -0.4 3.1416 7813 "$S/env_ycb_clutter.sh"
run icra_sign       anything_icra_c_matched_xarm6_t1       0.50  -0.40 1.5708 7814 "$S/env_icra_sign.sh"
run slalom          matched_slalom_xarm6_t1          0.381 -0.4 3.1416 7815 "$S/env_slalom.sh"
wait
echo "ALL PARALLEL SMOKES DONE"
