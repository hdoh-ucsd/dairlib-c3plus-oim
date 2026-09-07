#!/bin/bash
# 15 s smoke of the matched_open_table_xarm6_t1 trio.
cd "$(dirname "$0")/../.."
U="udpm://239.255.76.67:7995?ttl=0"
D=matched_open_table_xarm6_t1
bazel-bin/examples/sampling_c3/franka_sim --demo_name=$D --robot_model=xarm6 --matched_mu --lcm_url=$U >/tmp/smoke_sim.log 2>&1 &
SIM=$!
bazel-bin/examples/sampling_c3/franka_osc_controller --is_simulation=true --demo_name=$D --robot_model=xarm6 --lcm_url=$U >/tmp/smoke_osc.log 2>&1 &
OSC=$!
sleep 1
SAMPLING_C3_OBSTACLE_MODE=lcs_contact bazel-bin/examples/sampling_c3/franka_sampling_c3_controller --is_simulation=true --demo_name=$D --robot_model=xarm6 --lcm_url=$U >/tmp/smoke_c3.log 2>&1 &
C3=$!
sleep 15
kill $SIM $OSC $C3 2>/dev/null
sleep 1
kill -9 $SIM $OSC $C3 2>/dev/null
echo "===SIM==="; tail -6 /tmp/smoke_sim.log
echo "===OSC==="; tail -4 /tmp/smoke_osc.log
echo "===C3==="; tail -10 /tmp/smoke_c3.log
