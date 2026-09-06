#!/usr/bin/env python3
"""Agent D: exact-contact physical sanity test (section 4).

Standalone pydrake sim: ground + T + scene obstacles + pusher sphere on
actuated x/y/z prismatic joints with a stiff PD servo. Places the pusher at a
chosen contact sector and pushes in a chosen direction; measures object motion
and clearances. No LCM, no sim lock.
Usage: forced_contact_sanity.py TASK_DIR START_OFF_X START_OFF_Y PUSH_DX PUSH_DY TAG
  offsets are relative to the object start; push is a world-frame translation
  executed over 2.0 s.
"""
import sys, os, math, csv
import numpy as np
import yaml
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import PrismaticJoint, SpatialInertia, UnitInertia
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector
from pydrake.systems.analysis import Simulator
from pydrake.geometry import Sphere, ProximityProperties, AddContactMaterial
from pydrake.multibody.plant import CoulombFriction

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
task, offx, offy, dx, dy, tag = sys.argv[1], *map(float, sys.argv[2:6]), sys.argv[6]

def load_yaml(p):
    with open(p) as f:
        return yaml.safe_load(f)

cp = load_yaml(f"{WT}/examples/sampling_c3/{task}/parameters/sampling_c3_controller_params.yaml")
sim = load_yaml(cp["sim_params_file"].replace("examples/", f"{WT}/examples/"))
scen = load_yaml(cp["scenario_params_file"].replace("examples/", f"{WT}/examples/"))

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
parser = Parser(plant, scene_graph)
parser.SetAutoRenaming(True)
parser.AddModels(f"{WT}/examples/sampling_c3/urdf/ground.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"),
                 RigidTransform([0, 0, -0.029]))
obj = parser.AddModels(sim["object_models"][0].replace("examples/", f"{WT}/examples/"))[0]
if scen.get("obstacle_model"):
    parser.AddModels(scen["obstacle_model"].replace("examples/", f"{WT}/examples/"))

# pusher: sphere on x/y/z prismatic carriage
pusher_mi = plant.AddModelInstance("pusher_rig")
pm = 0.3
inert = SpatialInertia(pm, np.zeros(3), UnitInertia.SolidSphere(0.0195))
bx = plant.AddRigidBody("carr_x", pusher_mi, SpatialInertia(0.5, np.zeros(3), UnitInertia(1e-4, 1e-4, 1e-4)))
by = plant.AddRigidBody("carr_y", pusher_mi, SpatialInertia(0.5, np.zeros(3), UnitInertia(1e-4, 1e-4, 1e-4)))
tip = plant.AddRigidBody("pusher_tip", pusher_mi, inert)
jx = plant.AddJoint(PrismaticJoint("px", plant.world_frame(), bx.body_frame(), [1, 0, 0]))
jy = plant.AddJoint(PrismaticJoint("py", bx.body_frame(), by.body_frame(), [0, 1, 0]))
jz = plant.AddJoint(PrismaticJoint("pz", by.body_frame(), tip.body_frame(), [0, 0, 1]))
for j in (jx, jy, jz):
    plant.AddJointActuator(j.name(), j)
props = ProximityProperties()
AddContactMaterial(dissipation=0.1, friction=CoulombFriction(0.4615, 0.4615), properties=props)
plant.RegisterCollisionGeometry(tip, RigidTransform(), Sphere(0.0195), "tip_c", props)
plant.Finalize()

class Servo(LeafSystem):
    def __init__(self):
        super().__init__()
        self.DeclareVectorInputPort("state", plant.num_multibody_states())
        self.DeclareVectorOutputPort("u", 3, self.calc)
        self.kp, self.kd = 300.0, 40.0

    def target(self, t):
        # 0-1 s: hold at start; 1-3 s: linear push
        a = min(max((t - 1.0) / 2.0, 0.0), 1.0)
        return np.array([sx + a * dx, sy + a * dy, float(os.environ.get("AGENTD_PUSH_Z", "0.019"))])

    def calc(self, ctx, out):
        x = self.get_input_port().Eval(ctx)
        q = x[0:3]
        v = x[plant.num_positions():plant.num_positions()+3]
        tgt = self.target(ctx.get_time())
        out.SetFromVector(self.kp * (tgt - q) - self.kd * v)

q0 = np.array(sim["q_init_objects"][0], dtype=float)
ovr = os.environ.get("AGENTD_OBJ_POSE")
if ovr:
    _x, _y, _yw = map(float, ovr.split(","))
    q0[0], q0[1], q0[2], q0[3] = math.cos(_yw/2), 0.0, 0.0, math.sin(_yw/2)
    q0[4], q0[5] = _x, _y
sx, sy = q0[4] + offx, q0[5] + offy
servo = builder.AddSystem(Servo())
builder.Connect(plant.get_state_output_port(), servo.get_input_port())
builder.Connect(servo.get_output_port(), plant.get_actuation_input_port())
diagram = builder.Build()
ctx = diagram.CreateDefaultContext()
pctx = plant.GetMyContextFromRoot(ctx)
plant.SetPositions(pctx, obj, q0)
jx.set_translation(pctx, sx); jy.set_translation(pctx, sy); jz.set_translation(pctx, float(os.environ.get("AGENTD_PUSH_Z", "0.019")))

def obj_pose():
    q = plant.GetPositions(pctx, obj)
    yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
    return q[4], q[5], yaw

def min_dists():
    qo = scene_graph.get_query_output_port().Eval(scene_graph.GetMyContextFromRoot(ctx))
    insp = scene_graph.model_inspector()
    d_po, d_oo, d_pobs = 9e9, 9e9, 9e9
    fn_sum = 0.0
    for p in qo.ComputeSignedDistancePairwiseClosestPoints(max_distance=2.0):
        na = insp.GetName(insp.GetFrameId(p.id_A))
        nb = insp.GetName(insp.GetFrameId(p.id_B))
        pair = na + "|" + nb
        is_p = "pusher_tip" in pair
        is_o = ("push_t" in pair or "vertical_link" in pair or "horizontal_link" in pair or "c_glyph" in pair)
        is_obs = "scene_" in pair or "obstacle" in pair
        if is_p and is_o:
            d_po = min(d_po, p.distance)
        elif is_o and is_obs and not is_p:
            d_oo = min(d_oo, p.distance)
        elif is_p and is_obs:
            d_pobs = min(d_pobs, p.distance)
    return d_po, d_oo, d_pobs

x0, y0, yaw0 = obj_pose()
simr = Simulator(diagram, ctx)
simr.set_publish_every_time_step(False)
min_gap, contact_time = 9e9, 0.0
min_oo, min_pobs = 9e9, 9e9
for tstep in np.arange(0.1, 3.51, 0.1):
    simr.AdvanceTo(tstep)
    g, oo, pobs = min_dists()
    min_gap = min(min_gap, g)
    min_oo = min(min_oo, oo)
    min_pobs = min(min_pobs, pobs)
    if g < 1e-4:
        contact_time += 0.1
    if abs(tstep*10 - round(tstep*10)) < 1e-6 and int(round(tstep*10)) % 5 == 0:
        ox_, oy_, oyaw_ = obj_pose()
        print(f't={tstep:.1f} gap={g:.4f} obj=({ox_:.4f},{oy_:.4f},{oyaw_:.3f}) pusher_y={jy.get_translation(pctx):.4f} pz={jz.get_translation(pctx):.4f}')
x1, y1, yaw1 = obj_pose()
print('OBJ_FINAL', x1, y1, yaw1)
print('pusher final', jx.get_translation(pctx), jy.get_translation(pctx), jz.get_translation(pctx))
row = dict(scene=task, tag=tag, start_off=f"{offx:.3f};{offy:.3f}", push=f"{dx:.3f};{dy:.3f}",
           min_pusher_obj_gap=round(min_gap, 5), contact_dur_s=round(contact_time, 2),
           obj_dxy=round(math.hypot(x1-x0, y1-y0), 5),
           obj_dyaw=round(abs(math.remainder(yaw1-yaw0, 2*math.pi)), 4),
           min_obj_obstacle=round(min_oo, 5) if min_oo < 8e9 else "",
           min_pusher_obstacle=round(min_pobs, 5) if min_pobs < 8e9 else "",
           contact_acquired=min_gap < 1e-4)
print(row)
if os.environ.get("AGENTD_OBJ_POSE"):
    sys.exit(0)
path = "results_agent_d/contacts/scene_exact_contact_sanity.csv"
new = not os.path.exists(path)
with open(path, "a", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(row.keys()))
    if new:
        w.writeheader()
    w.writerow(row)
