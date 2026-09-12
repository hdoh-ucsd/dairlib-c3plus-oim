#!/usr/bin/env python3
"""Agent D: scene inventory + initial geometry checks for OIM-mapped scenes.

Replicates franka_sim.cc's plant assembly (Franka + EE + ground/platform +
object SDF + optional static obstacle SDF), sets the task's q_init, and dumps:
  - per-model inventory (bodies, geometries, mass, inertia, friction, pose)
  - pairwise signed distances (penetration check)
  - goal reachability vs workspace limits
Usage: scene_inventory.py TASK_DIR OUT_PREFIX
"""
import sys, os, csv, math
import numpy as np
import yaml
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.systems.framework import DiagramBuilder

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
task_dir, out_prefix = sys.argv[1], sys.argv[2]

def load_yaml(p):
    with open(p) as f:
        return yaml.safe_load(f)

cp = load_yaml(f"{WT}/examples/sampling_c3/{task_dir}/parameters/sampling_c3_controller_params.yaml")
sim = load_yaml(cp["sim_params_file"].replace("examples/", f"{WT}/examples/"))
scen = load_yaml(cp["scenario_params_file"].replace("examples/", f"{WT}/examples/"))
goal = load_yaml(cp["goal_params_file"].replace("examples/", f"{WT}/examples/"))
samp = load_yaml(cp["sampling_params_file"].replace("examples/", f"{WT}/examples/"))

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant, scene_graph)
parser.SetAutoRenaming(True)
franka = parser.AddModelsFromUrl(
    "package://drake_models/franka_description/urdf/panda_arm.urdf")[0]
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"))
ee = parser.AddModels(f"{WT}/examples/sampling_c3/urdf/end_effector_full.urdf")[0]
plant.WeldFrames(
    plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_flange"),
    RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)), [0, 0, 0.107]))
ground = parser.AddModels(f"{WT}/examples/sampling_c3/urdf/ground.urdf")[0]
platform = parser.AddModels(f"{WT}/examples/sampling_c3/urdf/platform.urdf")[0]
plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("ground"),
                 RigidTransform([0, 0, -0.029]))
plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("platform"),
                 RigidTransform([0, 0, -0.0145]))
objects = []
for m in sim["object_models"]:
    objects.append(parser.AddModels(m.replace("examples/", f"{WT}/examples/"))[0])
obstacle_model = scen.get("obstacle_model")
obs_instances = []
if obstacle_model:
    obs_instances = parser.AddModels(
        obstacle_model.replace("examples/", f"{WT}/examples/"))
plant.Finalize()

diagram = builder.Build()
ctx = diagram.CreateDefaultContext()
pctx = plant.GetMyContextFromRoot(ctx)
q_fr = np.array(sim["q_init_franka"], dtype=float)
plant.SetPositions(pctx, franka, q_fr)
for mi, q0 in zip(objects, sim["q_init_objects"]):
    plant.SetPositions(pctx, mi, np.array(q0, dtype=float))

insp = scene_graph.model_inspector()
rows = []
for mi in range(plant.num_model_instances()):
    from pydrake.multibody.tree import ModelInstanceIndex
    m = ModelInstanceIndex(mi)
    name = plant.GetModelInstanceName(m)
    for bi in plant.GetBodyIndices(m):
        b = plant.get_body(bi)
        X = plant.EvalBodyPoseInWorld(pctx, b)
        si = b.default_spatial_inertia()
        mass = si.get_mass()
        com = si.get_com().tolist() if mass > 0 else [0, 0, 0]
        gids = plant.GetCollisionGeometriesForBody(b)
        vis = plant.GetVisualGeometriesForBody(b)
        fric = ""
        shapes = []
        for g in gids:
            props = insp.GetProximityProperties(g)
            shapes.append(type(insp.GetShape(g)).__name__)
            if props and props.HasProperty("material", "coulomb_friction"):
                cf = props.GetProperty("material", "coulomb_friction")
                fric = f"{cf.static_friction():.3f}/{cf.dynamic_friction():.3f}"
        rot = RollPitchYaw(X.rotation()).vector()
        rows.append(dict(
            model=name, body=b.name(),
            floating=b.is_floating_base_body(),
            n_collision=len(gids), n_visual=len(vis),
            collision_shapes=";".join(shapes),
            mass=round(mass, 5), com=";".join(f"{v:.4f}" for v in com),
            friction=fric,
            x=round(X.translation()[0], 4), y=round(X.translation()[1], 4),
            z=round(X.translation()[2], 4),
            rpy=";".join(f"{v:.3f}" for v in rot),
            static=(mass == 0 or not b.is_floating_base_body())))
with open(f"{out_prefix}_scene_inventory.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)

# Geometry checks: signed distances
qobj = scene_graph.get_query_output_port().Eval(scene_graph.GetMyContextFromRoot(ctx))
pairs = qobj.ComputeSignedDistancePairwiseClosestPoints(max_distance=1.0)
checks = []
def gname(gid):
    return insp.GetName(insp.GetFrameId(gid)) + "/" + insp.GetName(gid).split("::")[-1]
worst = {}
for p in pairs:
    a, b = gname(p.id_A), gname(p.id_B)
    key = (a.split("/")[0], b.split("/")[0])
    if key[0] == key[1]:
        continue
    if key not in worst or p.distance < worst[key][0]:
        worst[key] = (p.distance, a, b)
for key, (d, a, b) in sorted(worst.items(), key=lambda kv: kv[1][0]):
    checks.append(dict(scene=task_dir, frame_a=a, frame_b=b,
                       min_signed_distance=round(d, 5),
                       penetrating=d < -1e-4))
with open(f"{out_prefix}_initial_geometry_checks.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(checks[0].keys()))
    w.writeheader(); w.writerows(checks)

# Goal / workspace summary
tgt = goal["fixed_target_position"]
ws = {k: samp.get(k) for k in samp if "workspace" in k or "limit" in k}
print("TASK", task_dir)
print("object_models", sim["object_models"])
print("q_init_objects", sim["q_init_objects"])
print("goal", tgt, goal["fixed_target_orientation"])
print("planner_obstacles", scen.get("obstacles"), "model:", obstacle_model)
print("obstacle_instances", [plant.GetModelInstanceName(m) for m in obs_instances])
print("workspace_keys", {k: v for k, v in ws.items()})
goal_r = math.hypot(tgt[0], tgt[1])
print("goal_radius_from_base", round(goal_r, 4))
neg = [c for c in checks if c["penetrating"]]
print("PENETRATIONS", len(neg), neg[:5])
