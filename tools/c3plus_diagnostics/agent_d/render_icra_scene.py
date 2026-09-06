#!/usr/bin/env python3
"""Render the faithful icra_sign initial scene (top-down + perspective)."""
import os, json, math
import numpy as np
from PIL import Image
from pydrake.geometry import (ClippingRange, ColorRenderCamera, DepthRange,
    DepthRenderCamera, MakeRenderEngineVtk, RenderCameraCore,
    RenderEngineVtkParams)
from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraInfo, RgbdSensor

WT = os.path.dirname(os.path.abspath(__file__)) + "/../../.."
builder = DiagramBuilder()
plant, sg = AddMultibodyPlantSceneGraph(builder, 0.0)
sg.AddRenderer("r", MakeRenderEngineVtk(RenderEngineVtkParams()))
parser = Parser(plant, sg)
parser.SetAutoRenaming(True)
parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"))
parser.AddModels(f"{WT}/examples/sampling_c3/urdf/end_effector_full.urdf")
plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_flange"),
                 RigidTransform(RollPitchYaw(3.1415, 0, 0).ToRotationMatrix(), [0, 0, 0.107]))
ground = parser.AddModels(f"{WT}/examples/sampling_c3/urdf/ground.urdf")[0]
plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("ground"),
                 RigidTransform([0, 0, -0.029]))
obj = parser.AddModels(f"{WT}/examples/sampling_c3/urdf/push_c_glyph.sdf")[0]
scene = parser.AddModels(f"{WT}/examples/sampling_c3/urdf/scene_icra_sign.sdf")[0]
plant.Finalize()

cams = {}
core_i = CameraInfo(1280, 960, np.pi / 4.0)
for name, eye, target in [
        ("perspective", np.array([1.45, -0.85, 0.75]), np.array([0.45, 0.0, 0.0])),
        ("topdown", np.array([0.42, 0.02, 1.55]), np.array([0.42, 0.0, 0.0]))]:
    z = (target - eye); z /= np.linalg.norm(z)
    up = np.array([0, 0, 1.0]) if name == "perspective" else np.array([1.0, 0, 0])
    x = np.cross(z, up); x /= np.linalg.norm(x); y = np.cross(z, x)
    core = RenderCameraCore("r", core_i, ClippingRange(0.05, 10.0), RigidTransform())
    cam = builder.AddSystem(RgbdSensor(
        sg.world_frame_id(),
        RigidTransform(RotationMatrix(np.column_stack([x, y, z])), eye),
        ColorRenderCamera(core, show_window=False),
        DepthRenderCamera(core, DepthRange(0.05, 10.0))))
    builder.Connect(sg.get_query_output_port(), cam.query_object_input_port())
    cams[name] = cam
diagram = builder.Build()
ctx = diagram.CreateDefaultContext()
pctx = plant.GetMyContextFromRoot(ctx)
import yaml
sim = yaml.safe_load(open(f"{WT}/examples/sampling_c3/anything_icra_c/parameters/sim_params.yaml"))
plant.SetPositions(pctx, plant.GetModelInstanceName(obj) and obj, np.array(sim["q_init_objects"][0]))
q_fr = np.array(sim["q_init_franka"])
from pydrake.multibody.tree import ModelInstanceIndex
for mi in range(plant.num_model_instances()):
    m = ModelInstanceIndex(mi)
    if plant.num_positions(m) == 7 and plant.GetModelInstanceName(m).startswith("panda"):
        plant.SetPositions(pctx, m, q_fr)
diagram.ForcedPublish(ctx)
for name, cam in cams.items():
    rgb = cam.color_image_output_port().Eval(cam.GetMyContextFromRoot(ctx)).data[:, :, :3]
    Image.fromarray(rgb.copy()).save(
        f"{WT}/results_icra_port/figures/icra_sign_initial_{name}.png")
    print("wrote", name)

state = dict(
    object="push_c_glyph", q_init_object=sim["q_init_objects"][0],
    q_init_franka=sim["q_init_franka"],
    glyphs={n: True for n in ["I", "R", "A", "2", "0", "2b", "6"]},
    planner_obstacles=8, static_glyphs=True, dynamic_c=True,
    goal=[0.5, -0.4, "pi/2"], penetrations=0)
json.dump(state, open(f"{WT}/results_icra_port/validation/initial_scene_state.json", "w"), indent=1)
print("state json written")
