#!/usr/bin/env bash
B=https://raw.githubusercontent.com/NikolaRaicevic2001/Object-Informed-Manipulation-MJX/main
L=/root/push_anything_ADMM/external/Object-Informed-Manipulation-MJX
for f in oim/utils/scenes.py oim/models/xarm6_pusht_tabletop/icra_sign.xml \
         examples/poses/icra_sign.yaml oim/objects/planar_pushing.py \
         oim/models/xarm6_pusht_tabletop/glyph_hulls.py tests/test_scenes.py; do
  if curl -s "$B/$f" | diff -q - "$L/$f" >/dev/null 2>&1; then
    echo "SAME $f"
  else
    echo "DIFF $f"
  fi
done
