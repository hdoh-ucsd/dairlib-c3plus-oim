#!/bin/bash
# Rewire open_table_glyph_{i,c,r,a}_xarm6 onto the push-anything letter objects
# (I/C/R_shape_texture, A_shape_video), per anything_I demo pattern.
set -euo pipefail
cd "$(dirname "$0")/../examples/sampling_c3"

declare -A LETTER=( [i]=I_shape_texture [c]=C_shape_texture [r]=R_shape_texture [a]=A_shape_video )
declare -A REST=( [i]=-0.007789 [c]=-0.005982 [r]=-0.008902 [a]=-0.004207 )

for g in i c r a; do
  d=open_table_glyph_${g}_xarm6
  L=${LETTER[$g]}
  Z=${REST[$g]}
  p=$d/parameters

  # --- sim_params: letter sim SDF, spawn at (0.30,0.40,rest), EE start moved
  #     off the spawn (base joint -0.7 rad; tip (0.3266,0.4007)->(0.508,0.096))
  sed -i \
    -e "s|object_model: .*|object_model: examples/sampling_c3/urdf/${L}/${L}.sdf|" \
    -e "s|object_models: .*|object_models: [examples/sampling_c3/urdf/${L}/${L}.sdf]|" \
    -e "s|q_init_franka: \[0.888666, |q_init_franka: [0.188666, |" \
    -e "s|q_init_object: .*|q_init_object: [1.0, 0, 0, 0, 0.30, 0.40, ${Z}]|" \
    -e "s|q_init_objects: .*|q_init_objects: [[1.0, 0, 0, 0, 0.30, 0.40, ${Z}]]|" \
    "$p/sim_params.yaml"

  # --- controller params: letter controller SDF, body/base names
  sed -i \
    -e "s|object_model: .*|object_model: examples/sampling_c3/urdf/${L}/${L}_controller.sdf|" \
    -e "s|object_models: .*|object_models: [examples/sampling_c3/urdf/${L}/${L}_controller.sdf]|" \
    -e "s|object_body_name: .*|object_body_name: body|" \
    -e "s|base_names: .*|base_names: [${L}]|" \
    "$p/sampling_c3_controller_params.yaml"

  # --- vis params
  sed -i "s|object_vis_model: .*|object_vis_model: examples/sampling_c3/urdf/${L}/${L}.sdf|" "$p/vis_params.yaml"

  # --- sampling params: anything lineage kMeshNormal (strategy 7), verbatim
  cp ../anything_I/parameters/sampling_params.yaml "$p/sampling_params.yaml" 2>/dev/null \
    || cp anything_I/parameters/sampling_params.yaml "$p/sampling_params.yaml"

  # --- goal params: per-letter resting height + fixed target z
  sed -i \
    -e "s|resting_object_height: .*|resting_object_height: ${Z}      # in world frame (table top -0.029 - mesh min z)|" \
    -e "s|resting_object_heights: .*|resting_object_heights: [${Z}]   # in world frame|" \
    -e "s|fixed_target_position: .*|fixed_target_position: [0.50, -0.40, ${Z}]|" \
    -e "s|fixed_target_positions: .*|fixed_target_positions: [[0.50, -0.40, ${Z}]]|" \
    "$p/goal_params.yaml"

  # --- restore reference contact options (match anything_I == bt010 T values)
  sed -i \
    -e "s|mu_per_pair_type: .*|mu_per_pair_type: [0.4165, 1, 0.4615]   # match URDFs with (2mu1*mu2)/(mu1+mu2)|" \
    -e "s|                            \[0, 1, 3, 0\]\].*|                            [0, 2, 3, 0]]|" \
    "$p/sampling_c3plus_options.yaml"
done
echo "=== object refs now:"
grep -rn 'object_model:\|base_names:\|q_init_object:\|resting_object_height:\|fixed_target_position:\|sampling_strategy' open_table_glyph_*_xarm6/parameters/{sim_params,sampling_c3_controller_params,goal_params,sampling_params}.yaml | grep -v '#.*base_names' | head -40
