#!/usr/bin/env bash
cd "$(dirname "$0")/../.."
export SAMPLING_C3_OBSTACLE_MODE=lcs_contact
export SAMPLING_C3_OBJECT_FOOTPRINT=c_glyph
export SAMPLING_C3_OBS_TOP_Z=0.046
SAMPLING_C3_OBS_POLYS="$(cat tools/scene_smoke/icra_obs_polys_env.txt)"
export SAMPLING_C3_OBS_POLYS
timeout 10 bazel-bin/examples/sampling_c3/franka_sampling_c3_controller \
  --is_simulation=true --demo_name=anything_icra_c_matched_xarm6_t1 \
  --robot_model=xarm6 --lcm_url='udpm://239.255.76.67:7899?ttl=0' 2>&1 \
  | grep -v warning | tail -6
