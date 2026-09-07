#!/bin/bash
# Smoke one open-table glyph demo for 20 s on a private LCM URL.
# Usage: smoke_open_table_glyph.sh <g> <port>   (g in i c r a)
set -uo pipefail
g=$1; port=$2
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
d=open_table_glyph_${g}_xarm6
fp=${g}_glyph
url="udpm://239.255.76.67:${port}?ttl=0"
out=/tmp/glyph_smoke_${g}
mkdir -p "$out"
bazel-bin/examples/sampling_c3/franka_sim --demo_name=$d --robot_model=xarm6 --matched_mu=true --lcm_url="$url" > "$out/sim.log" 2>&1 &
SIM=$!
bazel-bin/examples/sampling_c3/franka_osc_controller --demo_name=$d --robot_model=xarm6 --xarm6_five_joint=true --prelift_release=true --lcm_url="$url" > "$out/osc.log" 2>&1 &
OSC=$!
bazel-bin/examples/sampling_c3/franka_sampling_c3_controller --demo_name=$d --robot_model=xarm6 --lcm_url="$url" > "$out/planner.log" 2>&1 &
PLN=$!
sleep 20
kill $SIM $OSC $PLN 2>/dev/null
wait 2>/dev/null
echo "=== $g exit; log sizes:"; wc -l "$out"/*.log
