import csv
hulls = list(csv.DictReader(open('results_icra_port/obstacles/planner_glyph_hulls.csv')))
layout = {r['glyph']: r for r in csv.DictReader(open('results_icra_port/scene/icra_sign_layout.csv'))}
with open('results_agent_d/xarm6_crosscheck/scene_fidelity/icra_glyph_obstacles.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['glyph', 'world_x', 'world_y', 'world_z', 'vertex_index',
                'x_local', 'y_local', 'x_world', 'y_world', 'source'])
    for r in hulls:
        L = layout[r['glyph']]
        w.writerow([r['glyph'], L['x'], L['y'], L['z'], r['vertex_index'],
                    r['x_local'], r['y_local'],
                    round(float(L['x']) + float(r['x_local']), 4),
                    round(float(L['y']) + float(r['y_local']), 4),
                    'scenes.py L' + r['source_line']])
print('written')
