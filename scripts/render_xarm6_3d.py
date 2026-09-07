#!/usr/bin/env python3
"""Offline 3D playback renderer for xArm6 C3+ runs.

Builds a visual-only MultibodyPlant (actual xarm6 MJCF meshes + tool + T
object + ground/platform), renders each trace frame with RenderEngineVtk
via an RgbdSensor, burns a HUD with PIL, and encodes an mp4 with ffmpeg.
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

from pydrake.geometry import (
    ClippingRange, DepthRange, DepthRenderCamera, MakeRenderEngineVtk,
    RenderCameraCore, RenderEngineVtkParams, Rgba,
)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraInfo, RgbdSensor
from pydrake.common.eigen_geometry import Quaternion

REPO = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant"
XARM6_SRC_DIR = os.path.join(REPO, "examples/sampling_c3/urdf/oim_xarm6_tabletop/xarm6")


def prepare_xarm6_with_normals():
    """VTK requires OBJ vertex normals; the vendor OBJs lack them.
    Copy the MJCF + assets to /tmp and re-export meshes with normals."""
    import shutil
    import trimesh
    dst = "/tmp/xarm6_vtk_assets"
    if not os.path.exists(os.path.join(dst, ".done")):
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(XARM6_SRC_DIR, dst)
        adir = os.path.join(dst, "assets")
        for fn in os.listdir(adir):
            if fn.endswith(".obj"):
                m = trimesh.load(os.path.join(adir, fn), force="mesh")
                m.export(os.path.join(adir, fn), include_normals=True)
        open(os.path.join(dst, ".done"), "w").close()
    return os.path.join(dst, "xarm6_policyport.xml")


XARM6 = None  # set in build()
EE = os.path.join(REPO, "examples/sampling_c3/urdf/end_effector_full.urdf")
OBJ = os.path.join(REPO, "examples/sampling_c3/urdf/push_t_oimscale_m01.sdf")
GROUND = os.path.join(REPO, "examples/sampling_c3/urdf/ground.urdf")
PLATFORM = os.path.join(REPO, "examples/sampling_c3/urdf/platform.urdf")

W, H = 960, 720
FOV_Y = 0.9
EYE = np.array([1.3, -0.9, 0.9])
TARGET = np.array([0.4, 0.0, 0.1])


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


def build():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    scene_graph.AddRenderer("vtk", MakeRenderEngineVtk(RenderEngineVtkParams()))
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    # MJCF parser auto-welds jointless base to world
    parser.AddModels(prepare_xarm6_with_normals())
    parser.AddModels(EE)
    plant.WeldFrames(
        plant.GetFrameByName("xarm6_link6"),
        plant.GetFrameByName("end_effector_flange"),
        RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)), [0, 0, 0.107]))
    obj_model = parser.AddModels(OBJ)[0]
    parser.AddModels(GROUND)
    parser.AddModels(PLATFORM)
    plant.WeldFrames(plant.GetFrameByName("xarm6_link_base"),
                     plant.GetFrameByName("ground"),
                     RigidTransform([0, 0, -0.029]))
    plant.WeldFrames(plant.GetFrameByName("xarm6_link_base"),
                     plant.GetFrameByName("platform"),
                     RigidTransform([0, 0, -0.0145]))
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


def render_trace(trace_path, out_mp4, frames_dir, max_frames=1200):
    with open(trace_path) as f:
        rows = [json.loads(l) for l in f if l.strip()]
    step = max(1, int(np.ceil(len(rows) / max_frames)))
    rows = rows[::step]

    diagram, plant, sensor, obj_model = build()
    ctx = diagram.CreateDefaultContext()
    pctx = plant.GetMyContextFromRoot(ctx)
    sctx = sensor.GetMyContextFromRoot(ctx)
    body = obj_body(plant, obj_model)

    q1 = plant.GetJointByName("xarm6_joint1")
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
        d.rectangle([0, 0, W, 28], fill=(0, 0, 0))
        d.text((10, 7), txt, fill=(255, 255, 255))
        im.save(os.path.join(frames_dir, f"f{i:05d}.png"))
        if i % 100 == 0:
            print(f"  frame {i}/{len(rows)}", flush=True)

    subprocess.run([
        "ffmpeg", "-y", "-framerate", "10",
        "-i", os.path.join(frames_dir, "f%05d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out_mp4],
        check=True, capture_output=True)
    print(f"wrote {out_mp4} ({len(rows)} frames)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("trace")
    p.add_argument("out")
    p.add_argument("--frames-dir", default="/tmp/xarm6_frames")
    args = p.parse_args()
    render_trace(args.trace, args.out, args.frames_dir)
