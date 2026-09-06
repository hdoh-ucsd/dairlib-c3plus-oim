import sys
sys.argv = ['x', 'push_t_bt010_ycb_clutter', '0.0', '0.09', '0.0', '-0.06', 'dbg3']
src = open('tools/c3plus_diagnostics/agent_d/forced_contact_sanity.py').read()
exec(src.split('x0, y0, yaw0')[0])
print('num_positions', plant.num_positions(), 'num_velocities', plant.num_velocities())
print('pos idx', jx.position_start(), jy.position_start(), jz.position_start())
print('vel idx', jx.velocity_start(), jy.velocity_start(), jz.velocity_start())
print('q', plant.GetPositions(pctx))
