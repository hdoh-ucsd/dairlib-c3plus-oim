# OIM xArm6 assets

Imported through the audited Python-port asset set at
`sim/models/oim_xarm6_tabletop`, whose upstream is
NikolaRaicevic2001/Object-Informed-Manipulation-MJX.

This folder retains `xarm6/xarm6_policyport.xml`, the fixed-base xArm6 model
with five actuated joints and fixed wrist roll, and its seven referenced OBJ
meshes in `xarm6/assets/`. These model and mesh files are unchanged from that
local audited import. The native simulator and experiment renderer load this
robot model directly; the experiment scenes and object models are defined
elsewhere under `examples/sampling_c3/`.
