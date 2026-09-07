import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import glyph_micro_push as g

nm = g.ANYTHING['I']
v = g.load_obj_verts(os.path.join(g.URDF, nm, f'{nm}.obj'))
contacts, c = g.pick_contacts_cloud(v)
print('centroid', c.round(4))
for k, (p, n, note) in contacts.items():
    print(k, p.round(4), n.round(3), note)
r = g.run_push('I', *contacts['A_translation'][:2], object_set='anything')
for k, vv in r.items():
    print(k, np.round(vv, 4) if vv is not None else None)
