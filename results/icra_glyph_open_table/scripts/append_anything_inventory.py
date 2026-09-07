import re, os
import numpy as np

root = '/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant'
U = os.path.join(root, 'examples/sampling_c3/urdf')
OM = os.path.join(root, 'results/icra_glyph_open_table/object_models')
names = {'I': 'I_shape_texture', 'C': 'C_shape_texture',
         'R': 'R_shape_texture', 'A': 'A_shape_video'}


def field(t, s):
    m = re.search(f'<{t}>([^<]+)</{t}>', s)
    return m.group(1).strip() if m else ''


inv, svc = [], []
for g, nm in names.items():
    s = open(f'{U}/{nm}/{nm}.sdf').read()
    c = open(f'{U}/{nm}/{nm}_controller.sdf').read()
    v = np.array([l.split()[1:4] for l in open(f'{U}/{nm}/{nm}.obj')
                  if l.startswith('v ')], float)
    ext = v.max(0) - v.min(0)
    src = f'examples/sampling_c3/urdf/{nm}/{nm}.sdf'
    inv.append(
        f"{g},{field('mass', s)},0,0,0,{field('ixx', s)},{field('iyy', s)},"
        f"{field('izz', s)},{ext[0]:.4f},{ext[1]:.4f},{ext[2]:.4f},"
        f"{s.count('<collision')},0.3,0.3,"
        "n/a (limit-surface model not used; point-contact LCS),"
        f"{src},push-anything lineage (VHACD 10-piece; width/height=footprint "
        "x/y extent; stroke col holds thickness)")
    for prop in ['mass', 'ixx', 'iyy', 'izz']:
        sv, cv = field(prop, s), field(prop, c)
        if sv == cv:
            svc.append(f"{g}_anything,{prop},{sv},{cv},MATCH,")
        else:
            svc.append(f"{g}_anything,{prop},{sv},{cv},MISMATCH,"
                       "controller uses universal 1kg/diag declared tensor "
                       "(known intentional divergence)")
    smu = sorted(set(re.findall(r'mu_dynamic>\s*([^<\s]+)', s)))
    cmu = sorted(set(re.findall(r'mu_dynamic>\s*([^<\s]+)', c)))
    svc.append(f"{g}_anything,mu_values,{'/'.join(smu)},{'/'.join(cmu)},"
               f"{'MATCH' if smu == cmu else 'MISMATCH'},")
    ns, nc = s.count('<collision'), c.count('<collision')
    svc.append(f"{g}_anything,collision_geoms_nonwitness,{ns},{nc - 3},"
               f"{'MATCH' if ns == nc - 3 else 'MISMATCH'},"
               f"controller total {nc} incl 3 witness spheres")
with open(f'{OM}/glyph_model_inventory.csv', 'a') as f:
    f.write('\n'.join(inv) + '\n')
with open(f'{OM}/glyph_sim_vs_controller.csv', 'a') as f:
    f.write('\n'.join(svc) + '\n')
print('\n'.join(inv))
print()
print('\n'.join(svc))
