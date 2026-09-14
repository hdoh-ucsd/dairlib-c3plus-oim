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

Alternatively, --result reads recording.state_trace from a consolidated result
JSON, or its recorded dynamic states when a preserved trace is unavailable.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
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


def load_result_trace(result):
    """Read preserved trace frames, or project the recorded diagnostic states.

    This loader only rearranges saved values. It does not interpolate states,
    reconstruct joint angles, or evaluate kinematics.
    """
    recording = result.get("recording") or {}
    if "state_trace" in recording:
        rows = recording["state_trace"]
        if not isinstance(rows, list):
            raise ValueError("recording.state_trace must be a list of trace frames")
        return rows
    dynamic = recording.get("snapshot_dynamic") or result.get("dynamic") or {}
    static = result.get("static") or {}
    times = dynamic.get("time")
    positions = dynamic.get("qpos")
    poses = dynamic.get("object_pose_3d")
    addresses = static.get("block_qpos_adr") or []
    if not addresses or not isinstance(addresses[0], int) or isinstance(addresses[0], bool) or addresses[0] < 5:
        raise ValueError("Result must identify robot joints in static.block_qpos_adr")
    robot_joint_count = addresses[0]
    if (not all(isinstance(value, list) for value in (times, positions, poses))
            or len(times) != len(positions) or len(times) != len(poses)):
        raise ValueError("Result time, qpos and object_pose_3d must be aligned state arrays")
    errors = {}
    for key in ("position_error_m", "orientation_error_rad"):
        errors[key] = dynamic.get(key, [None] * len(times))
        if not isinstance(errors[key], list) or len(errors[key]) != len(times):
            raise ValueError(f"Result {key} must align with recorded states")
    rows = []
    for index, (time, qpos, pose) in enumerate(zip(times, positions, poses)):
        if not isinstance(qpos, list) or len(qpos) < robot_joint_count or not isinstance(pose, list) or len(pose) != 7:
            raise ValueError("Result contains an incomplete recorded robot or object pose")
        rows.append({"t": time, "q": qpos[:robot_joint_count], "obj": pose,
                     "pos_err": errors["position_error_m"][index],
                     "ang_err": errors["orientation_error_rad"][index]})
    return rows


def _verify_recorded_model(path, result):
    """Check available recorded digests for an automatically chosen model.

    Includes its referenced mesh files: matching SDF text alone does not prove
    that the visible geometry matches. Older results without asset hashes can
    still replay, but cannot establish that their repository assets are equal.
    """
    hashes = {}
    provenance = result.get("provenance") or {}
    for status in (provenance.get("recorded_runtime") or {}, result.get("runtime_status") or {}):
        recorded = status.get("asset_sha256") or {}
        if not isinstance(recorded, dict):
            raise ValueError("Recorded asset_sha256 must be a mapping")
        for name, digest in recorded.items():
            if name in hashes and hashes[name] != digest:
                raise ValueError(f"Conflicting recorded asset hashes for {name}")
            hashes[name] = digest
    if not hashes:
        return
    repo = Path(REPO).resolve()
    # Existing run-local snapshots have rewritten SDF URIs and therefore use
    # config_sha256, not the canonical source asset hashes checked here.
    try:
        canonical = path.relative_to(repo).as_posix()
    except ValueError:
        return
    if not canonical.startswith("examples/sampling_c3/urdf/"):
        return

    def check(asset):
        name = asset.relative_to(repo).as_posix()
        expected = hashes.get(name)
        if expected is not None and hashlib.sha256(asset.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Replay asset differs from the recorded run: {name}. "
                             "Restore the recorded repository revision/assets, or supply an explicit "
                             "--object-sdf / --obstacle-sdf override to use different geometry.")

    check(path)
    if path.suffix.lower() == ".sdf":
        if __package__:
            from .catalog import model_assets
        else:
            from catalog import model_assets
        selected = {role: {"object_models": [canonical]} for role in ("simulation", "controller")}
        for asset in model_assets(selected, repo):
            check(asset)


def resolve_result_model(reference, result_path, result=None):
    """Resolve retained assets, including snapshots whose config was compacted.

    Missing snapshot paths can identify the same canonical repository asset;
    geometry is never guessed from only a basename or object label. Recorded
    hashes, when available, must match automatically selected checkout assets.
    """
    if not isinstance(reference, str) or not reference:
        raise ValueError("Result does not identify a model; supply --object-sdf")
    path = Path(reference)
    candidates = [path]
    if not path.is_absolute():
        candidates.extend((Path(result_path).parent / path, Path(REPO) / path))
    parts = path.parts
    for index in range(len(parts) - 2):
        if parts[index:index + 3] == ("examples", "sampling_c3", "urdf"):
            candidates.append(Path(REPO).joinpath(*parts[index:]))
    for candidate in candidates:
        if candidate.is_file():
            resolved = candidate.resolve()
            _verify_recorded_model(resolved, result or {})
            return str(resolved)
    raise FileNotFoundError(f"Saved model is unavailable: {reference}; restore its repository asset "
                            "or supply --object-sdf / --obstacle-sdf explicitly")


def load_render_input(args):
    """Return rows and selected model/goal values without constructing Drake."""
    result_path = getattr(args, "result", None)
    trace_path = getattr(args, "trace", None)
    if bool(result_path) == bool(trace_path):
        raise ValueError("Choose exactly one of --trace or --result")
    object_sdf = getattr(args, "object_sdf", None)
    obstacle_sdf = getattr(args, "obstacle_sdf", None)
    goal = getattr(args, "goal", None)
    if result_path:
        result = json.loads(Path(result_path).read_text())
        rows = load_result_trace(result)
        static = result.get("static") or {}
        if object_sdf is None:
            object_sdf = resolve_result_model(static.get("simulation_model"), result_path, result)
        if goal is None:
            goal = static.get("goal")
        if obstacle_sdf is None:
            config = (result.get("provenance") or {}).get("evaluation_scene_config") or {}
            obstacle = static.get("obstacle_model") or config.get("obstacle_model")
            if not obstacle:
                provenance = result.get("provenance") or {}
                configurations = dict(((result.get("hyperparameters") or {}).get("c3plus") or {}).get("configurations") or {})
                configurations.update((provenance.get("c3plus") or {}).get("configurations") or {})
                configurations.update({name: entry.get("data") for name, entry in
                                       ((provenance.get("configuration") or {}).get("files") or {}).items()})
                obstacles = {value["obstacle_model"] for value in configurations.values()
                             if isinstance(value, dict) and value.get("scenario_name")
                             and isinstance(value.get("obstacle_model"), str) and value["obstacle_model"]}
                if len(obstacles) > 1:
                    raise ValueError("Result contains multiple obstacle models; supply --obstacle-sdf")
                obstacle = next(iter(obstacles), None)
            if obstacle:
                obstacle_sdf = resolve_result_model(obstacle, result_path, result)
    else:
        if object_sdf is None:
            raise ValueError("--object-sdf is required with --trace")
        with open(trace_path) as stream:
            rows = [json.loads(line) for line in stream if line.strip()]
    if goal is not None and (len(goal) != 3 or not np.isfinite(np.asarray(goal, dtype=float)).all()):
        raise ValueError("Goal must contain finite world x, y and yaw values")
    for row in rows:
        if (not isinstance(row, dict) or not isinstance(row.get("q"), list) or len(row["q"]) < 5
                or not isinstance(row.get("obj"), list) or len(row["obj"]) != 7):
            raise ValueError("Trace frames require time, at least five robot joints and a seven-value object pose")
        values = np.asarray([row.get("t"), *row["q"], *row["obj"]], dtype=float)
        if not np.isfinite(values).all() or np.linalg.norm(row["obj"][:4]) == 0:
            raise ValueError("Trace contains missing or invalid recorded state values")
    return rows, object_sdf, obstacle_sdf, goal


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
    # Keep explicitly requested frame/mesh directories, but reclaim our own
    # temporary copies after encoding or failure during a serial campaign.
    args = argparse.Namespace(**vars(args))
    with ExitStack() as stack:
        if args.assets_tmp is None:
            args.assets_tmp = stack.enter_context(tempfile.TemporaryDirectory(prefix="xarm6_vtk_assets_"))
        if args.frames_dir is None:
            args.frames_dir = stack.enter_context(tempfile.TemporaryDirectory(prefix="render_frames_"))
        _render_trace(args)


def _render_trace(args):
    if args.max_frames < 1 or args.fps < 1:
        raise ValueError("--max-frames and --fps must be positive")
    rows, object_sdf, obstacle_sdf, goal = load_render_input(args)
    if not rows:
        raise ValueError("Trace contains no frames")
    step = max(1, int(np.ceil(len(rows) / args.max_frames)))
    rows = rows[::step]

    assets_tmp = args.assets_tmp
    frames_dir = args.frames_dir
    diagram, plant, sensor, obj_model = build(
        object_sdf, obstacle_sdf, goal, assets_tmp)
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
        pos_error, ang_error = r.get("pos_err"), r.get("ang_err")
        pos_label = f"{pos_error * 1000:6.1f} mm" if pos_error is not None and np.isfinite(pos_error) else "n/a"
        ang_label = f"{np.degrees(ang_error):6.1f} deg" if ang_error is not None and np.isfinite(ang_error) else "n/a"
        txt = f"t = {r['t']:7.1f} s   pos_err = {pos_label}   ang_err = {ang_label}"
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


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--trace", help="state_trace.jsonl")
    source.add_argument("--result", help="Consolidated *_result.json; uses preserved trace or recorded dynamic states")
    p.add_argument("--out", required=True, help="output mp4 path")
    p.add_argument("--object-sdf",
                   help="manipulated object model (inferred from --result when available)")
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
    args = p.parse_args(argv)
    if args.trace and args.object_sdf is None:
        p.error("--object-sdf is required with --trace")
    render_trace(args)


if __name__ == "__main__":
    main()
