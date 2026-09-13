#!/usr/bin/env python3
"""Offline 3D playback renderer for xArm6 C3+/OIM runs (generalized).

Builds a visual-only MultibodyPlant (xarm6 MJCF meshes + tool + a
user-specified object SDF + optional static obstacle/scene SDF +
ground/platform + goal marker), renders each trace frame with
RenderEngineVtk via an RgbdSensor, burns a HUD with PIL, and encodes an
mp4 with ffmpeg.

Trace format: one JSON object per line:
  {t, q (list, first 5 = xArm6 joints), obj ([qw,qx,qy,qz,x,y,z]),
   pos_err, ang_err}
"""
import argparse
import json
import os
import shutil
import subprocess
import tempfile

import numpy as np
from PIL import Image, ImageDraw

from pydrake.geometry import (
    Box, ClippingRange, Cylinder, DepthRange, DepthRenderCamera,
    MakeRenderEngineVtk, RenderCameraCore, RenderEngineVtkParams,
)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraInfo, RgbdSensor
from pydrake.common.eigen_geometry import Quaternion

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
URDF = os.path.join(REPO, "examples/sampling_c3/urdf")
XARM6_SRC_DIR = os.path.join(URDF, "oim_xarm6_tabletop/xarm6")
# OIM-faithful tool + table (matches AddXarm6ToPlant in the sim): flush
# stick EE on link6, white 0.80 x 1.523 m table with its long axis along y.
EE = os.path.join(URDF, "end_effector_xarm6_stick.urdf")
GROUND = os.path.join(URDF, "ground_oim_xarm6.urdf")

W, H = 960, 720
FOV_Y = 0.9
EYE = np.array([1.3, -0.9, 0.9])
TARGET = np.array([0.4, 0.0, 0.1])


def prepare_xarm6_with_normals(assets_tmp=None):
    """VTK requires OBJ vertex normals; the vendor OBJs lack them.
    Copy the MJCF + assets to a run-unique tmp dir and re-export meshes
    with normals (avoids parallel-job clobbering)."""
    import trimesh
    if assets_tmp is None:
        assets_tmp = tempfile.mkdtemp(prefix="xarm6_vtk_assets_")
    dst = assets_tmp
    if not os.path.exists(os.path.join(dst, ".done")):
        if os.path.realpath(dst) == os.path.realpath(XARM6_SRC_DIR):
            raise ValueError("Use a temporary asset directory, not the source meshes")
        shutil.copytree(XARM6_SRC_DIR, dst, dirs_exist_ok=True)
        adir = os.path.join(dst, "assets")
        for fn in os.listdir(adir):
            if fn.endswith(".obj"):
                m = trimesh.load(os.path.join(adir, fn), force="mesh")
                m.export(os.path.join(adir, fn), include_normals=True)
        open(os.path.join(dst, ".done"), "w").close()
    return os.path.join(dst, "xarm6_policyport.xml")


def look_at(eye, target):
    """Camera pose: +Z forward (view direction), +X right, +Y down."""
    fwd = target - eye
    fwd = fwd / np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, up)
    right = right / np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.column_stack([right, down, fwd])
    return RigidTransform(RotationMatrix(R), eye)


def weld_unjointed_bases(plant, model):
    """Weld to world any body of `model` that is not the child of a joint
    (pre-Finalize), so non-static scene SDFs render as fixed geometry."""
    child_bodies = set()
    for ji in plant.GetJointIndices(model):
        child_bodies.add(plant.get_joint(ji).child_body().index())
    for bi in plant.GetBodyIndices(model):
        if bi not in child_bodies:
            b = plant.get_body(bi)
            plant.WeldFrames(plant.world_frame(), b.body_frame(),
                             RigidTransform())


def sdf_is_static(path):
    try:
        with open(path) as f:
            txt = f.read()
        return "<static>" in txt and "true" in txt.split("<static>")[1][:10]
    except OSError:
        return False


def add_goal_marker(plant, gx, gy, gyaw):
    """Translucent green disc + yaw arrow at the goal pose (visual only)."""
    green = np.array([0.1, 0.9, 0.2, 0.55])
    world = plant.world_body()
    z = 0.006  # just above the ground plane
    plant.RegisterVisualGeometry(
        world, RigidTransform([gx, gy, z]),
        Cylinder(0.03, 0.002), "goal_disc", green)
    # yaw arrow: thin box extending from disc center along the goal yaw
    L = 0.07
    Ryaw = RotationMatrix(RollPitchYaw(0.0, 0.0, gyaw))
    off = Ryaw @ np.array([L / 2, 0.0, 0.0])
    plant.RegisterVisualGeometry(
        world, RigidTransform(Ryaw, [gx + off[0], gy + off[1], z]),
        Box(L, 0.008, 0.002), "goal_arrow", green)


def build(object_sdf, obstacle_sdf, goal, assets_tmp):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    # Camera-frame headlight + a downward fill so the white table reads white
    # instead of shaded gray under the single default directional light.
    from pydrake.geometry import LightParameter
    scene_graph.AddRenderer("vtk", MakeRenderEngineVtk(RenderEngineVtkParams(
        lights=[LightParameter(type="directional", frame="camera",
                               direction=[0, 0, 1], intensity=0.85),
                LightParameter(type="directional", frame="world",
                               direction=[0, 0, -1], intensity=0.55)])))
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    # MJCF parser auto-welds jointless base to world
    parser.AddModels(prepare_xarm6_with_normals(assets_tmp))
    parser.AddModels(EE)
    plant.WeldFrames(
        plant.GetFrameByName("xarm6_link6"),
        plant.GetFrameByName("end_effector_flange"),
        RigidTransform())  # flush at link6, like the sim's OIM stick weld
    obj_model = parser.AddModels(object_sdf)[0]
    if obstacle_sdf:
        obs_model = parser.AddModels(obstacle_sdf)[0]
        if not sdf_is_static(obstacle_sdf):
            weld_unjointed_bases(plant, obs_model)
    parser.AddModels(GROUND)
    plant.WeldFrames(plant.GetFrameByName("xarm6_link_base"),
                     plant.GetFrameByName("ground"),
                     RigidTransform([0, 0, -0.029]))
    if goal is not None:
        add_goal_marker(plant, goal[0], goal[1], goal[2])
    plant.Finalize()

    core = RenderCameraCore("vtk", CameraInfo(W, H, FOV_Y),
                            ClippingRange(0.05, 10.0), RigidTransform())
    depth_cam = DepthRenderCamera(core, DepthRange(0.1, 5.0))
    sensor = builder.AddSystem(RgbdSensor(
        scene_graph.world_frame_id(), look_at(EYE, TARGET), depth_cam))
    builder.Connect(scene_graph.get_query_output_port(),
                    sensor.query_object_input_port())
    diagram = builder.Build()
    return diagram, plant, sensor, obj_model


def obj_body(plant, obj_model):
    bodies = plant.GetBodyIndices(obj_model)
    for bi in bodies:
        b = plant.get_body(bi)
        if b.is_floating():
            return b
    return plant.get_body(bodies[0])


def render_trace(args):
    with open(args.trace) as f:
        rows = [json.loads(l) for l in f if l.strip()]
    step = max(1, int(np.ceil(len(rows) / args.max_frames)))
    rows = rows[::step]

    assets_tmp = args.assets_tmp or tempfile.mkdtemp(prefix="xarm6_vtk_assets_")
    frames_dir = args.frames_dir or tempfile.mkdtemp(prefix="render_frames_")
    diagram, plant, sensor, obj_model = build(
        args.object_sdf, args.obstacle_sdf, args.goal, assets_tmp)
    ctx = diagram.CreateDefaultContext()
    pctx = plant.GetMyContextFromRoot(ctx)
    sctx = sensor.GetMyContextFromRoot(ctx)
    body = obj_body(plant, obj_model)

    joints = [plant.GetJointByName(f"xarm6_joint{i}") for i in range(1, 6)]

    os.makedirs(frames_dir, exist_ok=True)
    for i, r in enumerate(rows):
        for j, qv in zip(joints, r["q"]):
            j.set_angle(pctx, qv)
        o = r["obj"]
        q = np.array(o[:4])
        q = q / np.linalg.norm(q)
        plant.SetFreeBodyPose(pctx, body,
                              RigidTransform(Quaternion(q), o[4:7]))
        img = sensor.color_image_output_port().Eval(sctx)
        arr = np.array(img.data, copy=True).reshape(H, W, 4)[:, :, :3]
        im = Image.fromarray(arr)
        d = ImageDraw.Draw(im)
        txt = (f"t = {r['t']:7.1f} s   pos_err = {r['pos_err']*1000:6.1f} mm"
               f"   ang_err = {np.degrees(r['ang_err']):6.1f} deg")
        if args.title:
            txt = f"{args.title}   |   " + txt
        d.rectangle([0, 0, W, 28], fill=(0, 0, 0))
        d.text((10, 7), txt, fill=(255, 255, 255))
        im.save(os.path.join(frames_dir, f"f{i:05d}.png"))
        if i % 100 == 0:
            print(f"  frame {i}/{len(rows)}", flush=True)

    subprocess.run([
        "ffmpeg", "-y", "-framerate", str(args.fps),
        "-i", os.path.join(frames_dir, "f%05d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", args.out],
        check=True, capture_output=True)
    print(f"wrote {args.out} ({len(rows)} frames @ {args.fps} fps)")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--trace", required=True, help="state_trace.jsonl")
    p.add_argument("--out", required=True, help="output mp4 path")
    p.add_argument("--object-sdf", required=True,
                   help="manipulated object model (floating, posed by trace)")
    p.add_argument("--obstacle-sdf", default=None,
                   help="optional static scene/obstacle model")
    p.add_argument("--goal", nargs=3, type=float, default=None,
                   metavar=("GX", "GY", "GYAW"),
                   help="goal pose; draws a green disc + yaw arrow")
    p.add_argument("--title", default=None, help="HUD title text")
    p.add_argument("--fps", type=int, default=10)
    p.add_argument("--max-frames", type=int, default=1200)
    p.add_argument("--frames-dir", default=None,
                   help="frame PNG dir (default: unique tmp dir)")
    p.add_argument("--assets-tmp", default=None,
                   help="dir for the normal-fixed xarm6 asset copy "
                        "(default: unique tmp dir)")
    args = p.parse_args()
    render_trace(args)
