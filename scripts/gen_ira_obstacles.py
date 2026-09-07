#!/usr/bin/env python3
"""Generate static I/R/A letter-obstacle SDF for the C-object scene."""
import glob, os

WT = '/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant'
U = WT + '/examples/sampling_c3/urdf'
# (letter dir, pose x y z yaw) — z = each letter's measured resting height on
# the -0.029 table top; slalom across the (0.30,0.40)->(0.50,-0.40) task line.
LETTERS = [
    ('I_shape_texture', 0.30, 0.15, -0.0078, 1.5708),
    ('R_shape_texture', 0.55, 0.10, -0.0102, 0.0),
    ('A_shape_video',   0.38, -0.18, -0.0056, 0.6),
]
parts = ['<?xml version="1.0"?>',
         '<!-- Static letter obstacles (I, R, A) for the C-object scene.',
         '     Same push-anything VHACD pieces as the dynamic letters, welded',
         '     static at resting height on the OIM table. -->',
         '<sdf version="1.7">',
         '  <model name="ira_letter_obstacles">',
         '    <static>true</static>']
for name, x, y, z, yaw in LETTERS:
    parts.append(f'    <link name="obstacle_{name}">')
    parts.append(f'      <pose>{x} {y} {z} 0 0 {yaw}</pose>')
    for i, piece in enumerate(sorted(glob.glob(f'{U}/{name}/{name}_convex_*.obj'))):
        rel = f'{name}/{os.path.basename(piece)}'
        parts.append(f'''      <visual name="v{i}">
        <geometry><mesh><uri>{rel}</uri></mesh></geometry>
        <material><diffuse>0.85 0.55 0.10 1</diffuse></material>
      </visual>
      <collision name="c{i}">
        <geometry><mesh><uri>{rel}</uri></mesh></geometry>
        <drake:proximity_properties xmlns:drake="uri:drake">
          <drake:compliant_hydroelastic/>
          <drake:hydroelastic_modulus> 3.0e7 </drake:hydroelastic_modulus>
          <drake:mesh_resolution_hint> 0.02 </drake:mesh_resolution_hint>
          <drake:hunt_crossley_dissipation>10</drake:hunt_crossley_dissipation>
          <drake:mu_dynamic>0.3</drake:mu_dynamic>
        </drake:proximity_properties>
      </collision>''')
    parts.append('    </link>')
parts += ['  </model>', '</sdf>']
out = U + '/scene_ira_letter_obstacles.sdf'
open(out, 'w').write('\n'.join(parts) + '\n')
print('wrote', out, 'letters:', [l[0] for l in LETTERS])
