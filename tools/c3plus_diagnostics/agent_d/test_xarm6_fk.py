import os, sys
os.environ["AGENTD_FK"] = "xarm6"
sys.argv = ['x', 'r', 'icra_sign', '0.5', '-0.4', '1.57', 'a', 'b', 'c']
src = open('tools/c3plus_diagnostics/agent_d/postprocess_draw.py').read()
exec(src.split('# ---- T footprint')[0])
print('n_positions', plant.num_positions())
import numpy as np
print('ee at zeros:', ee_pos(np.zeros(plant.num_positions())))
