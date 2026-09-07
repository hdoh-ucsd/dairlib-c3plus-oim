#!/bin/bash
# Create open_table_glyph_{i,c,r,a}_xarm6 task dirs from push_t_bt010_open_table_xarm6.
set -euo pipefail
cd "$(dirname "$0")/../examples/sampling_c3"
for g in i c r a; do
  d=open_table_glyph_${g}_xarm6
  rm -rf "$d"
  cp -r push_t_bt010_open_table_xarm6 "$d"
  grep -rl 'push_t_bt010_open_table_xarm6' "$d" | xargs sed -i "s|push_t_bt010_open_table_xarm6|$d|g"
  sed -i \
    -e "s|urdf/push_t_oimscale_m01.sdf|urdf/push_${g}_glyph.sdf|g" \
    -e "s|\[1.0, 0, 0, 0, 0.5, 0.3, 0.0008\]|[1.0, 0, 0, 0, 0.30, 0.40, -0.0165]|g" \
    -e "s|\[\[1.0, 0, 0, 0, 0.5, 0.3, 0.0008\]\]|[[1.0, 0, 0, 0, 0.30, 0.40, -0.0165]]|g" \
    "$d/parameters/sim_params.yaml"
  sed -i \
    -e "s|urdf/push_t_oimscale_m01_control.sdf|urdf/push_${g}_glyph_controller.sdf|g" \
    -e "s|object_body_name: vertical_link|object_body_name: c_glyph_base|" \
    -e "s|base_names: \[vertical_link\]|base_names: [c_glyph_base]|" \
    -e "s|# Note:  Use vertical_link body name as representative of the pose of the entire|# Note:  All glyph SDFs use a single link named c_glyph_base (shared naming|" \
    -e "s|# T model.  This works because vertical_link's origin is the same as the T's.|# convention from the C-glyph task); its origin is the glyph frame origin.|" \
    "$d/parameters/sampling_c3_controller_params.yaml"
  sed -i "s|urdf/push_t.sdf|urdf/push_${g}_glyph.sdf|" "$d/parameters/vis_params.yaml"
  sed -i \
    -e "s|fixed_target_position: \[0.5, -0.3, 0.0008\]|fixed_target_position: [0.50, -0.40, -0.0165]|" \
    -e "s|fixed_target_positions: \[\[0.5, -0.3, 0.0008\]\]|fixed_target_positions: [[0.50, -0.40, -0.0165]]|" \
    -e "s|fixed_target_orientation: \[0.0, 0.0, 0.0, 1.0\]|fixed_target_orientation: [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]|" \
    -e "s|fixed_target_orientations: \[\[0.0, 0.0, 0.0, 1.0\]\]|fixed_target_orientations: [[0.7071067811865476, 0.0, 0.0, 0.7071067811865476]]|" \
    "$d/parameters/goal_params.yaml"
done
echo "=== residual T references (should only be comments/harmless):"
grep -rn 'push_t_bt010\|vertical_link\|oimscale\|push_t\b' open_table_glyph_*_xarm6/ || true
