#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/../examples/sampling_c3"
for g in i c r a; do
  d=open_table_glyph_${g}_xarm6
  sed -i "s|_push_t\"|_glyph_${g}\"|g" "$d/BUILD.bazel"
  for p in "$d"/franka_hardware_t.pmd "$d"/franka_sim_t.pmd; do
    sed -i "s|--demo_name=push_t|--demo_name=$d|g; s|start_logging.py sim push_t /mnt/data2/push_t/logs/|start_logging.py sim $d /mnt/data2/$d/logs/|; s|start_logging.py hw push_t /mnt/data2/push_t/logs/|start_logging.py hw $d /mnt/data2/$d/logs/|" "$p"
  done
  mv "$d/franka_hardware_t.pmd" "$d/franka_hardware_glyph_${g}.pmd"
  mv "$d/franka_sim_t.pmd" "$d/franka_sim_glyph_${g}.pmd"
done
grep -rn 'demo_name=push_t\b' open_table_glyph_*_xarm6/ || echo OK
