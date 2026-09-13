#!/usr/bin/env python3
"""Regenerate the four benchmark SDFs and collision hulls; never run dynamics.

Requires NumPy, SciPy, trimesh and Pillow in this interpreter, and PyBullet
(V-HACD 2.2) in the interpreter selected by --vhacd-python. Runtime experiments
use the checked-in assets and do not require PyBullet.
"""

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull
import trimesh


ROOT = Path(__file__).resolve().parent
NAMES = ("sugar_box", "power_drill", "hammer", "banana")
MASS = 0.1
TABLE_Z = -0.029
FRICTION = 0.3
VHACD = dict(resolution=500000, concavity=0.0025, planeDownsampling=4,
             convexhullDownsampling=4, alpha=0.05, beta=0.05, gamma=0.00125,
             pca=0, mode=0, maxNumVerticesPerCH=64, minVolumePerCH=0.0001,
             convexhullApproximation=1)


def numbers(values):
    return " ".join(f"{float(x):.12g}" for x in values)


def simplify(points, tolerance):
    """Ramer-Douglas-Peucker simplification of one open polyline."""
    if len(points) <= 2:
        return points
    delta = points[-1] - points[0]
    t = np.clip((points - points[0]) @ delta / max(delta @ delta, 1e-30), 0, 1)
    distance = np.linalg.norm(points - points[0] - t[:, None] * delta, axis=1)
    index = int(distance.argmax())
    if distance[index] <= tolerance:
        return points[[0, -1]]
    return np.vstack((simplify(points[:index + 1], tolerance)[:-1],
                      simplify(points[index:], tolerance)))


def footprint(mesh, pitch=0.00025, tolerance=0.0005):
    """Project every triangle to XY, trace the union, simplify its outer ring."""
    origin = mesh.bounds[0, :2] - 2 * pitch
    size = np.ceil(mesh.extents[:2] / pitch).astype(int) + 5
    image = Image.new("1", tuple(size.tolist()))
    draw = ImageDraw.Draw(image)
    for triangle in mesh.triangles[:, :, :2]:
        pixels = np.rint((triangle - origin) / pitch).astype(int)
        draw.polygon([tuple(p) for p in pixels], fill=1)
    occupied = np.asarray(image, dtype=bool)
    edges = {}

    def edge(start, end):
        edges.setdefault(start, []).append(end)

    for y, x in np.argwhere(occupied):
        if not occupied[y - 1, x]:
            edge((x, y), (x + 1, y))
        if not occupied[y, x + 1]:
            edge((x + 1, y), (x + 1, y + 1))
        if not occupied[y + 1, x]:
            edge((x + 1, y + 1), (x, y + 1))
        if not occupied[y, x - 1]:
            edge((x, y + 1), (x, y))
    rings = []
    while edges:
        start = next(iter(edges))
        ring, point = [], start
        while True:
            ring.append(point)
            next_point = edges[point].pop()
            if not edges[point]:
                del edges[point]
            point = next_point
            if point == start:
                break
        polygon = origin + (np.asarray(ring) - 0.5) * pitch
        area = np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1)
                      - polygon[:, 1] * np.roll(polygon[:, 0], -1)) / 2
        rings.append((area, polygon))
    area, polygon = max(rings, key=lambda entry: entry[0])
    split = int(np.linalg.norm(polygon - polygon[0], axis=1).argmax())
    polygon = np.vstack((simplify(polygon[:split + 1], tolerance)[:-1],
                         simplify(np.vstack((polygon[split:], polygon[:1])),
                                  tolerance)[:-1]))
    polygon = np.clip(polygon, mesh.bounds[0, :2], mesh.bounds[1, :2])
    return polygon, dict(method="XY triangle projection raster union; largest outer ring",
                         pixel_size_m=pitch, simplification_tolerance_m=tolerance,
                         outer_rings=int(sum(a > 0 for a, _ in rings)),
                         hole_rings=int(sum(a < 0 for a, _ in rings)),
                         raster_outer_area_m2=float(area))


def ground_witnesses(mesh):
    """A large support triangle from the underside, enclosing the mesh COM."""
    candidates = mesh.vertices[mesh.vertices[:, 2] <= mesh.bounds[0, 2] + 0.005]
    candidates = candidates[ConvexHull(candidates[:, :2]).vertices]
    centre = mesh.center_mass[:2]
    best = None
    for indices in itertools.combinations(range(len(candidates)), 3):
        triangle = candidates[list(indices)]
        basis = np.column_stack((triangle[1, :2] - triangle[0, :2],
                                 triangle[2, :2] - triangle[0, :2]))
        area = abs(np.linalg.det(basis)) / 2
        if area < 1e-12:
            continue
        barycentric = np.linalg.solve(basis, centre - triangle[0, :2])
        if np.min(barycentric) < 0.02 or barycentric.sum() > 0.98:
            continue
        if best is None or area > best[0]:
            best = (area, triangle)
    if best is None:
        raise ValueError("No underside support triangle encloses the mesh COM")
    original = best[1].copy()
    result = original.copy()
    result[:, 2] = mesh.bounds[0, 2] + 0.001
    return result, original


def sdf(name, mesh, hull_paths, witnesses, controller):
    inertia = mesh.moment_inertia
    inertia_xml = "".join(f"<{tag}>{inertia[i, j]:.12g}</{tag}>" for tag, i, j in
                          (("ixx", 0, 0), ("iyy", 1, 1), ("izz", 2, 2),
                           ("ixy", 0, 1), ("ixz", 0, 2), ("iyz", 1, 2)))
    model_name = name + ("_controller" if controller else "")
    parts = ['<?xml version="1.0"?>',
             '<!-- Generated by generate_models.py. Benchmark mass and collision',
             '     approximations are documented in physics_models.json. -->',
             '<sdf version="1.7" xmlns:drake="http://drake.mit.edu">',
             f'  <model name="{model_name}"><link name="{name}_base">',
             f'    <inertial><pose>{numbers(mesh.center_mass)} 0 0 0</pose>',
             f'      <mass>{MASS}</mass><inertia>{inertia_xml}</inertia></inertial>',
             f'    <visual name="object_visual"><geometry><mesh><uri>{name}_centered.obj</uri>',
             '      </mesh></geometry><material><diffuse>0.62 0.67 0.73 1</diffuse></material></visual>']
    friction = ('<drake:proximity_properties><drake:mu_static>0.3</drake:mu_static>'
                '<drake:mu_dynamic>0.3</drake:mu_dynamic></drake:proximity_properties>')
    for i, path in enumerate(hull_paths):
        parts.extend((f'    <collision name="hull_{i:02d}"><geometry><mesh>',
                      f'      <uri>{path}</uri><drake:declare_convex/></mesh></geometry>',
                      f'      {friction}</collision>'))
    if controller:
        parts.append('    <!-- Native contact wiring requires these three spheres LAST. -->')
        for i, point in enumerate(witnesses):
            parts.extend((f'    <collision name="ground_witness_{i}"><pose>{numbers(point)} 0 0 0</pose>',
                          '      <geometry><sphere><radius>0.001</radius></sphere></geometry>',
                          f'      {friction}</collision>'))
    parts.extend(('  </link></model>', '</sdf>'))
    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vhacd-python", default=sys.executable,
                        help="Python interpreter with PyBullet installed (generation only)")
    args = parser.parse_args()
    pybullet_version = subprocess.check_output(
        [args.vhacd_python, "-c", "from importlib.metadata import version;print(version('pybullet'))"],
        text=True).strip()
    metadata = dict(mass_kg=MASS,
                    mass_source="Explicit common 0.1 kg benchmark assumption, not measured object masses",
                    inertia_source="Uniform density volume integrals of each cleaned visual triangle surface",
                    friction_static=FRICTION, friction_dynamic=FRICTION,
                    decomposition=dict(method="PyBullet V-HACD 2.2", pybullet_version=pybullet_version,
                                       trimesh_version=trimesh.__version__, parameters=VHACD,
                                       postprocess="Clip hull vertices to original XYZ bounds, then recompute each convex hull"),
                    ground_model="Simulation uses convex pieces; controller uses three idealized planar support spheres",
                    objects={})
    collision_dir = ROOT / "collision"
    collision_dir.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="object-vhacd-") as temp:
        for name in NAMES:
            source = ROOT / f"{name}_centered.obj"
            mesh = trimesh.load(source, force="mesh", process=False)
            mesh.density = MASS / mesh.volume
            output = Path(temp) / f"{name}.obj"
            program = ("import json,pybullet,sys;pybullet.vhacd(sys.argv[1],sys.argv[2],"
                       "sys.argv[3],**json.loads(sys.argv[4]))")
            subprocess.run([args.vhacd_python, "-c", program, str(source), str(output),
                            str(Path(temp) / f"{name}.log"), json.dumps(VHACD)],
                           stdout=subprocess.DEVNULL, check=True)
            decomposed = trimesh.load(output, force="mesh", process=False)
            hulls = decomposed.split(only_watertight=False, repair=False)
            hulls.sort(key=lambda h: tuple(h.bounds.mean(axis=0)))
            paths, hull_info = [], []
            for i, hull in enumerate(hulls):
                vertices = np.clip(hull.vertices, mesh.bounds[0], mesh.bounds[1])
                hull = trimesh.convex.convex_hull(vertices)
                relative = f"collision/{name}_{i:02d}.obj"
                target = ROOT / relative
                target.write_text(trimesh.exchange.obj.export_obj(
                    hull, include_normals=True, include_texture=False) + "\n")
                paths.append(relative)
                hull_info.append(dict(file=relative, vertices=len(hull.vertices),
                                      triangles=len(hull.faces), volume_m3=float(hull.volume),
                                      sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
            witnesses, source_support = ground_witnesses(mesh)
            polygon, projection = footprint(mesh)
            height = float(mesh.extents[2])
            metadata["objects"][name] = dict(
                source_mesh=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                simulation_model=f"{name}.sdf", controller_model=f"{name}_controller.sdf",
                body_name=f"{name}_base", height_m=height, object_origin_world_z_m=TABLE_Z,
                centre_of_mass_m=mesh.center_mass.tolist(), inertia_at_com_kg_m2=mesh.moment_inertia.tolist(),
                convex_pieces=hull_info, collision_piece_count=len(hull_info),
                collision_piece_volume_sum_over_mesh_volume=sum(h["volume_m3"] for h in hull_info) / mesh.volume,
                witness_radius_m=0.001, witness_centres_m=witnesses.tolist(),
                witness_source_vertices_m=source_support.tolist(),
                witness_selection="Maximum-area triangle from vertices within 5 mm of the bottom, enclosing COM; centres placed on idealized planar support",
                evaluation=dict(footprint=polygon.tolist(), footprint_method=projection,
                                block_half_height=height / 2, tip_target_z=TABLE_Z + height / 2,
                                tip_floor_z_real=TABLE_Z + 0.00555))
            for controller in (False, True):
                target = ROOT / (name + ("_controller" if controller else "") + ".sdf")
                target.write_text(sdf(name, mesh, paths, witnesses, controller))
                metadata["objects"][name]["controller_sha256" if controller else "simulation_sha256"] = (
                    hashlib.sha256(target.read_bytes()).hexdigest())
            print(f"{name}: {len(paths)} convex pieces; {len(polygon)} footprint vertices")
    (ROOT / "physics_models.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
