#!/bin/bash
# Run sim+OSC only (no planner) and watch object pose to verify resting spawn.
set -uo pipefail
g=$1; port=$2; dur=${3:-10}
root="$(cd "$(dirname "$0")/.." && pwd)"; cd "$root"
d=open_table_glyph_${g}_xarm6
url="udpm://239.255.76.67:${port}?ttl=0"
bazel-bin/examples/sampling_c3/franka_sim --demo_name=$d --robot_model=xarm6 --matched_mu=true --lcm_url="$url" > /tmp/probe_sim_$g.log 2>&1 &
SIM=$!
bazel-bin/examples/sampling_c3/franka_osc_controller --demo_name=$d --robot_model=xarm6 --xarm6_five_joint=true --prelift_release=true --lcm_url="$url" > /tmp/probe_osc_$g.log 2>&1 &
OSC=$!
/root/miniconda3/envs/push_anything_ADMM/bin/python3 scripts/watch_object_state.py "$port" "$dur"
kill $SIM $OSC 2>/dev/null
wait 2>/dev/null
