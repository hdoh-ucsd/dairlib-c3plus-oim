#!/bin/bash
# 15 s smoke of an xarm6 demo trio on a private LCM url. Usage: smoke_xarm6_demo.sh <demo_name> <log_prefix> [obstacle_mode]
set -u
cd "$(dirname "$0")/.."
D="$1"; P="$2"; OM="${3:-}"
LCM="udpm://239.255.76.67:7991?ttl=0"
mkdir -p /tmp/xarm6_smokes
timeout 15 ./bazel-bin/examples/sampling_c3/franka_sim --demo_name="$D" --robot_model=xarm6 --matched_mu=true --lcm_url="$LCM" > /tmp/xarm6_smokes/${P}_sim.log 2>&1 &
timeout 15 ./bazel-bin/examples/sampling_c3/franka_osc_controller --demo_name="$D" --robot_model=xarm6 --xarm6_five_joint=true --prelift_release=true --lcm_url="$LCM" > /tmp/xarm6_smokes/${P}_osc.log 2>&1 &
if [ -n "$OM" ]; then
  SAMPLING_C3_OBSTACLE_MODE="$OM" timeout 15 ./bazel-bin/examples/sampling_c3/franka_sampling_c3_controller --demo_name="$D" --robot_model=xarm6 --lcm_url="$LCM" > /tmp/xarm6_smokes/${P}_c3.log 2>&1 &
else
  timeout 15 ./bazel-bin/examples/sampling_c3/franka_sampling_c3_controller --demo_name="$D" --robot_model=xarm6 --lcm_url="$LCM" > /tmp/xarm6_smokes/${P}_c3.log 2>&1 &
fi
wait
echo "SMOKE DONE $D"
