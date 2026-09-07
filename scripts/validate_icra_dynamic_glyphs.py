#!/usr/bin/env python3
"""Validate dynamic ICRA glyph models (I, R, A) + render stills incl. C.

- Loads each sim SDF into a MultibodyPlant with the AddXarm6ToPlant ground
  (ground.urdf welded at z=-0.029 under xarm6_link_base; table top = -0.029).
- Verifies mass/COM/inertia vs the polygon-derived values.
- Rests the glyph on the table (bottom at -0.029, i.e. link z = -0.0165),
  checks signed distance to table in [-1e-6, 1e-3].
- Renders top-down + 3/4 stills to results/icra_glyph_open_table/initial_scene/.
- Writes glyph_sim_vs_controller.csv comparing sim vs controller SDFs.
"""
import csv
import os
import re
import sys

import numpy as np
from PIL import Image

from pydrake.geometry import (
    ClippingRange, DepthRange, DepthRenderCamera, MakeRenderEngineVtk,
    RenderCameraCore, RenderEngineVtkParams, Role,
)
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraInfo, RgbdSensor

sys.path.insert(0, os.path.dirname(__file__))
from build_icra_dynamic_glyphs import load_hulls, polygon_props, DENSITY, THICKNESS

REPO = "/root/push_anything_ADMM/external/oim_c++_anything/.claude/worktrees/audit-xarm6-plant"
URDF = os.path.join(REPO, "examples/sampling_c3/urdf")
GROUND = os.path.join(URDF, "ground.urdf")
SCENE_DIR = os.path.join(REPO, "results/icra_glyph_open_table/initial_scene")
OBJ_DIR = os.path.join(REPO, "results/icra_glyph_open_table/object_models")

TABLE_TOP_Z = -0.029          # ground.urdf top (z=0 in ground frame) welded at -0.029
REST_Z = TABLE_TOP_Z + THICKNESS / 2   # -0.0165

W, H = 800, 600


def look_at(eye, target):
    fwd = target - eye
    fwd = fwd / np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(fwd, up)) > 0.99:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd, up)
    right = right / np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.column_stack([right, down, fwd])
    return RigidTransform(RotationMatrix(R), eye)


def build_scene(sdf_path, cam_pose):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    scene_graph.AddRenderer("vtk", MakeRenderEngineVtk(RenderEngineVtkParams()))
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    obj_model = parser.AddModels(sdf_path)[0]
    parser.AddModels(GROUND)
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"),
                     RigidTransform([0, 0, TABLE_TOP_Z]))
    plant.Finalize()
    core = RenderCameraCore("vtk", CameraInfo(W, H, 0.9),
                            ClippingRange(0.02, 10.0), RigidTransform())
    depth_cam = DepthRenderCamera(core, DepthRange(0.05, 5.0))
    sensor = builder.AddSystem(RgbdSensor(
        scene_graph.world_frame_id(), cam_pose, depth_cam))
    builder.Connect(scene_graph.get_query_output_port(),
                    sensor.query_object_input_port())
    diagram = builder.Build()
    return diagram, plant, scene_graph, sensor, obj_model


def validate_and_render(g, sdf_path, expect=None):
    print(f"=== {g}: {os.path.basename(sdf_path)}")
    views = {
        "top": look_at(np.array([0.0, 0.0, 0.35]), np.array([0.0, 0.0, REST_Z])),
        "3q": look_at(np.array([0.18, -0.18, 0.14]), np.array([0.0, 0.0, REST_Z])),
    }
    imgs = []
    for vname, cam in views.items():
        diagram, plant, scene_graph, sensor, obj_model = build_scene(sdf_path, cam)
        ctx = diagram.CreateDefaultContext()
        pctx = plant.GetMyContextFromRoot(ctx)
        body = plant.get_body(plant.GetBodyIndices(obj_model)[0])
        plant.SetFreeBodyPose(pctx, body, RigidTransform([0, 0, REST_Z]))

        if vname == "top":
            # --- mass properties check
            M = body.default_spatial_inertia()
            mass = M.get_mass()
            com = M.get_com()
            I = M.CalcRotationalInertia().CopyToFullMatrix3()  # about link origin
            # shift to COM
            Icom = I - mass * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
            if expect is not None:
                em, ecom, eI = expect
                assert abs(mass - em) < 1e-9, (mass, em)
                # SDF pose is written to 6 decimals; quantization <= 5e-7 m
                assert np.allclose(com, ecom, atol=1e-6), (com, ecom)
                assert np.allclose(np.diag(Icom), eI, rtol=1e-6), (np.diag(Icom), eI)
                print(f"  mass/COM/inertia MATCH: m={mass:.6f} com={com} Idiag={np.diag(Icom)}")
            else:
                print(f"  mass={mass:.6f} com={com} Idiag={np.diag(Icom)}")
            # --- signed distance to table
            qo = scene_graph.get_query_output_port().Eval(
                scene_graph.GetMyContextFromRoot(ctx))
            insp = qo.inspector()
            pairs = qo.ComputeSignedDistancePairwiseClosestPoints(0.5)
            dmin = None
            for p in pairs:
                na = insp.GetName(insp.GetFrameId(p.id_A))
                nb = insp.GetName(insp.GetFrameId(p.id_B))
                if "ground" in na + nb:
                    d = p.distance
                    dmin = d if dmin is None else min(dmin, d)
            print(f"  resting z = {REST_Z}, min signed distance to table = {dmin:.3e}")
            assert dmin is not None and -1e-6 <= dmin <= 1e-3, dmin

        sctx = sensor.GetMyContextFromRoot(ctx)
        img = sensor.color_image_output_port().Eval(sctx)
        arr = np.array(img.data, copy=True).reshape(H, W, 4)[:, :, :3]
        imgs.append(arr)

    combo = np.concatenate(imgs, axis=1)
    out = os.path.join(SCENE_DIR, f"{g}.png")
    Image.fromarray(combo).save(out)
    print(f"  wrote {out}")


def parse_sdf_props(path):
    txt = open(path).read()
    props = {}
    props["mass"] = re.search(r"<mass>([-\d.e]+)</mass>", txt).group(1)
    props["inertial_pose"] = re.search(r"<inertial><pose>([^<]+)</pose>", txt).group(1)
    for k in ("ixx", "iyy", "izz"):
        props[k] = re.search(rf"<{k}>([-\d.e]+)</{k}>", txt).group(1)
    props["mu_values"] = ",".join(sorted(set(re.findall(r"<mu>([\d.]+)</mu>", txt))))
    geoms = re.findall(r'<collision name="([^"]+)"', txt)
    props["collision_geoms_nonwitness"] = ",".join(g for g in geoms if "witness" not in g)
    props["witness_spheres"] = str(sum(1 for g in geoms if "witness" in g))
    props["mesh_uri"] = ",".join(set(re.findall(r"<uri>([^<]+)</uri>", txt))) or "boxes"
    return props


def main():
    os.makedirs(SCENE_DIR, exist_ok=True)
    hulls = load_hulls()
    for g, (pts, _) in hulls.items():
        A, cx, cy, Ix_c, Iy_c = polygon_props(pts)
        mass = DENSITY * A * THICKNESS
        Ixx = DENSITY * THICKNESS * Ix_c + mass * THICKNESS**2 / 12
        Iyy = DENSITY * THICKNESS * Iy_c + mass * THICKNESS**2 / 12
        Izz = DENSITY * THICKNESS * (Ix_c + Iy_c)
        sdf = os.path.join(URDF, f"push_{g.lower()}_glyph.sdf")
        validate_and_render(g, sdf, expect=(mass, [cx, cy, 0.0], [Ixx, Iyy, Izz]))
    validate_and_render("C", os.path.join(URDF, "push_c_glyph.sdf"), expect=None)

    # sim vs controller comparison
    rows = []
    keys = ["mass", "inertial_pose", "ixx", "iyy", "izz", "mu_values",
            "collision_geoms_nonwitness", "witness_spheres", "mesh_uri"]
    for g in ("I", "R", "A", "C"):
        s = parse_sdf_props(os.path.join(URDF, f"push_{g.lower()}_glyph.sdf"))
        c = parse_sdf_props(os.path.join(URDF, f"push_{g.lower()}_glyph_controller.sdf"))
        for k in keys:
            match = s[k] == c[k]
            if k == "witness_spheres":
                match = s[k] == "0" and c[k] == "3"
                note = "expected delta: controller-only witness spheres"
            else:
                note = ""
            rows.append([g, k, s[k], c[k], "MATCH" if match else "MISMATCH", note])
    path = os.path.join(OBJ_DIR, "glyph_sim_vs_controller.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["glyph", "property", "sim_value", "controller_value", "match", "note"])
        w.writerows(rows)
    bad = [r for r in rows if r[4] != "MATCH"]
    print(f"wrote {path}; mismatches = {len(bad)}")
    for r in bad:
        print("  MISMATCH:", r)


if __name__ == "__main__":
    main()
