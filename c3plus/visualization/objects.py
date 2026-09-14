"""Save a PNG of an object in simulation coordinates using Drake.

Loads the configured robot, tool, table, platform, obstacles, and object at its
start or goal pose. Optional model/OBJ overrides are previews; experiment
configurations and mesh files are never rewritten. No dynamics are advanced
and no browser or server is needed."""
import argparse
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from c3plus.configs.catalog import SCENES, demo_name
from c3plus.configs.paths import EXPERIMENTS_FILE, REPO
from c3plus.configs.resolver import compose_demo_configs, load_demo_configs
from .ee_samples import sample_ee_candidates, sample_raw_mesh_ee_candidates


def preview_settings(args):
    """Resolve the physical model, world pose, and image output without rendering."""
    if args.width < 1 or args.height < 1:
        raise ValueError("--width and --height must be positive")
    if args.ee_samples is not None and not 1 <= args.ee_samples <= 10000:
        raise ValueError("--ee-samples must be between 1 and 10000")
    if args.sample_seed is not None and args.ee_samples is None:
        raise ValueError("--sample-seed requires --ee-samples")
    if args.sample_seed is not None and args.sample_seed < 0:
        raise ValueError("--sample-seed must be nonnegative")
    if args.sample_height is not None:
        if args.mesh is None or args.ee_samples is None:
            raise ValueError("--sample-height requires --mesh and --ee-samples")
        if not math.isfinite(args.sample_height):
            raise ValueError("--sample-height must be finite")
    if args.goal_yaw_degrees is not None and args.pose != "goal":
        raise ValueError("--goal-yaw-degrees requires --pose goal")
    if args.goal_yaw_degrees is not None and args.rpy_degrees is not None:
        raise ValueError("Choose either --goal-yaw-degrees or --rpy-degrees")
    if args.mesh_scale is not None and args.mesh is None:
        raise ValueError("--mesh-scale requires --mesh")
    scale = 1.0 if args.mesh_scale is None else args.mesh_scale
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("--mesh-scale must be a positive finite number")
    for value in (args.position, args.rpy_degrees):
        if value is not None and not all(math.isfinite(v) for v in value):
            raise ValueError("Position and orientation must contain finite numbers")

    demo = demo_name(args.scene, args.start, args.goal)
    configs = compose_demo_configs(demo, goal_yaw_degrees=args.goal_yaw_degrees)
    sources = load_demo_configs(demo)
    controller, simulation, goal = (configs[key] for key in ("controller", "simulation", "goal"))
    options = sources[REPO / controller["sampling_c3_options_file"]]
    if options.get("include_walls", False):
        raise ValueError("This preview supports the catalogued scenes with include_walls=false")
    scenario = sources[REPO / controller["scenario_params_file"]]
    pose = (simulation["q_init_objects"][0] if args.pose == "start"
            else [*goal["fixed_target_orientation"], *goal["fixed_target_position"]])
    model = (args.mesh or args.model or REPO / simulation["object_models"][0]).resolve()
    allowed = (".obj",) if args.mesh else (".sdf", ".urdf", ".xml")
    if model.suffix.lower() not in allowed:
        raise ValueError("Use --mesh for an OBJ, or --model for an SDF, URDF, or MJCF XML")
    if not model.is_file():
        raise FileNotFoundError(model)
    if args.geometry == "collision" and args.mesh:
        raise ValueError("Raw OBJ previews have no collision geometry; use --model")
    profiles = sources[REPO / EXPERIMENTS_FILE]["object_profiles"]
    model_names = {(REPO / profile[key]).resolve(): name
                   for name, profile in profiles.items()
                   for key in ("simulation_model", "controller_model")}
    object_name = model_names.get(model, model.stem)
    ee_sampling = None
    if args.ee_samples is not None:
        configured_models = {(REPO / simulation["object_models"][0]).resolve(),
                             (REPO / controller["object_models"][0]).resolve()}
        if args.mesh is None and model not in configured_models:
            raise ValueError("--ee-samples requires the selected scene's configured object model")
        if args.mesh is not None and args.position is None:
            raise ValueError("Raw mesh EE previews require --position X Y Z to set the object placement")
        params_file = REPO / controller["sampling_params_file"]
        params = sources[params_file]
        strategy = params["sampling_strategy"]
        if strategy not in (4, 7):
            raise ValueError("EE previews support the configured perimeter and mesh-normal samplers")
        ee_model = REPO / "examples/sampling_c3/urdf/end_effector_simple_model_xarm6.urdf"
        # Drake accepts its extension prefix without an XML namespace declaration.
        ee_xml = ee_model.read_text()
        if "xmlns:drake=" not in ee_xml:
            ee_xml = ee_xml.replace("<robot ", '<robot xmlns:drake="http://drake.mit.edu" ', 1)
        ee_radius = float(ET.fromstring(ee_xml).find(".//collision/geometry/sphere").attrib["radius"])
        base = controller["base_names"][0]
        mesh_file = REPO / "examples/sampling_c3/urdf" / base / f"{base}.obj"
        if strategy == 7 and not mesh_file.is_file():
            raise FileNotFoundError(mesh_file)
        ee_sampling = {
            "params_file": str(params_file),
            "options_file": str(REPO / controller["sampling_c3_options_file"]),
            "controller_model": str(REPO / controller["object_models"][0]),
            "mesh_file": str(mesh_file) if strategy == 7 else None,
            "ee_model_file": str(ee_model), "ee_radius": ee_radius,
            "count": args.ee_samples,
            "seed": 42 if args.sample_seed is None else args.sample_seed,
        }
        if args.mesh is not None:
            ee_sampling = {
                "mode": "mesh_section_perimeter", "mesh_file": str(model),
                "options_file": str(REPO / controller["sampling_c3_options_file"]),
                "ee_model_file": str(ee_model), "ee_radius": ee_radius,
                "height": -0.012 if args.sample_height is None else args.sample_height,
                "clearance": 0.027, "offset": 0.035,
                "count": args.ee_samples,
                "seed": 42 if args.sample_seed is None else args.sample_seed,
            }
    yaw = "" if args.goal_yaw_degrees is None else f"_yaw{args.goal_yaw_degrees:+04d}"
    samples = "" if ee_sampling is None else f'_ee{ee_sampling["count"]}_seed{ee_sampling["seed"]}'
    output = args.output or (REPO / "results/previews" /
        f"{args.scene}_{object_name}_{args.pose}_s{args.start:02d}g{args.goal:02d}{yaw}_{args.view}_{args.geometry}{samples}.png")
    if output.suffix.lower() != ".png":
        raise ValueError("--output must end in .png")
    obstacle = scenario.get("obstacle_model")
    return {
        "scene": args.scene, "demo": demo, "pose": args.pose,
        "object_name": object_name, "object_file": str(model), "raw_mesh": args.mesh is not None,
        "mesh_scale": scale,
        "position_m": args.position if args.position is not None else pose[4:7],
        "quaternion_wxyz": None if args.rpy_degrees is not None else pose[:4],
        "rpy_degrees": args.rpy_degrees,
        "robot_joints_rad": simulation["q_init_franka"],
        "obstacle_file": str(REPO / obstacle) if obstacle else None,
        "output": str(output.resolve()), "view": args.view, "geometry": args.geometry,
        "width": args.width, "height": args.height, "frames": args.frames,
        "hide_robot": args.hide_robot,
        "ee_sampling": ee_sampling,
    }

def mesh_urdf(path, scale):
    """Wrap an OBJ for display without deriving inertia from its mesh volume."""
    robot = ET.Element("robot", name="mesh_preview")
    link = ET.SubElement(robot, "link", name="object")
    # Positive dummy inertia is only for the viewer's movable body. This is
    # not a physics model, and it intentionally defines no collision geometry.
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value="1")
    ET.SubElement(inertial, "inertia", ixx="1", iyy="1", izz="1", ixy="0", ixz="0", iyz="0")
    geometry = ET.SubElement(ET.SubElement(link, "visual"), "geometry")
    # In-memory URDF parsing requires a URI. Drake resolves its path literally,
    # so retain spaces and let ElementTree escape XML-special characters.
    uri = "file://" + str(Path(path).resolve())
    ET.SubElement(geometry, "mesh", filename=uri, scale=" ".join([str(scale)] * 3))
    return ET.tostring(robot, encoding="unicode")

def prepare_render_geometry(scene_graph, source, directory, geometry, overlay_ids, hidden_ids=()):
    """Select camera-visible geometry and prepare temporary OBJ normals."""
    from pydrake.geometry import Convex, Mesh, PerceptionProperties, RenderLabel, Rgba, Role

    inspector = scene_graph.model_inspector()
    prepared = {}
    for index, gid in enumerate(inspector.GetAllGeometryIds()):
        illustration = inspector.GetIllustrationProperties(gid)
        proximity = inspector.GetProximityProperties(gid)
        visible = (illustration is not None if geometry == "visual" else proximity is not None)
        visible = (visible or gid in overlay_ids) and gid not in hidden_ids
        if inspector.GetPerceptionProperties(gid) is not None:
            scene_graph.RemoveRole(source, gid, Role.kPerception)
        if not visible:
            continue
        properties = PerceptionProperties(illustration) if illustration is not None else PerceptionProperties()
        if not properties.HasProperty("phong", "diffuse"):
            properties.AddProperty("phong", "diffuse", Rgba(0.9, 0.45, 0.15, 1))
        if not properties.HasProperty("label", "id"):
            properties.AddProperty("label", "id", RenderLabel.kDontCare)
        shape = inspector.GetShape(gid)
        if (geometry == "collision" and proximity is not None and isinstance(shape, Mesh)
                and not proximity.HasGroup("hydroelastic")):
            # Drake's point-contact solver uses a convex hull for mesh collision.
            scene_graph.ChangeShape(source, gid, Convex(shape.source(), scale3=shape.scale3()))
        elif isinstance(shape, Mesh) and shape.extension() == ".obj" and shape.source().is_path():
            path = Path(shape.source().path())
            if path not in prepared:
                has_normals, faces_have_normals = False, True
                with path.open(errors="replace") as mesh_file:
                    for line in mesh_file:
                        fields = line.split("#", 1)[0].split()
                        if fields and fields[0] == "vn":
                            has_normals = True
                        elif fields and fields[0] == "f":
                            faces_have_normals &= all(
                                len(vertex.split("/")) == 3 and vertex.split("/")[2]
                                for vertex in fields[1:])
                prepared[path] = path
                if not (has_normals and faces_have_normals):
                    import trimesh
                    destination = Path(directory) / f"mesh_{index}"
                    destination.mkdir()
                    prepared[path] = destination / path.name
                    mesh = trimesh.load(path, force="mesh", process=False)
                    mesh.export(prepared[path], include_normals=True)
            if prepared[path] != path:
                scene_graph.ChangeShape(source, gid, Mesh(str(prepared[path]), scale3=shape.scale3()))
        scene_graph.AssignRole(source, gid, properties)

def build_preview(settings, directory):
    """Build a scene and camera without advancing dynamics or opening a server."""
    # Imports stay here so --help and --dry-run work without Drake or a display.
    import numpy as np
    from pydrake.common.eigen_geometry import Quaternion
    from pydrake.geometry import (
        ClippingRange, DepthRange, DepthRenderCamera, LightParameter,
        MakeRenderEngineVtk, RenderCameraCore, RenderEngineVtkParams, Sphere,
    )
    from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
    from pydrake.systems.framework import DiagramBuilder
    from pydrake.systems.sensors import CameraInfo, RgbdSensor
    from pydrake.visualization import AddFrameTriadIllustration
    from .camera import EYE, TARGET, look_at

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    urdf = REPO / "examples/sampling_c3/urdf"

    # Geometry and welds match AddXarm6ToPlant in sampling_c3_utils.cc.
    arm, = parser.AddModels(str(urdf / "oim_xarm6_tabletop/xarm6/xarm6_policyport.xml"))
    tool, = parser.AddModels(str(urdf / "end_effector_xarm6_stick.urdf"))
    plant.WeldFrames(plant.GetFrameByName("xarm6_link6", arm),
                     plant.GetFrameByName("end_effector_flange", tool), RigidTransform())
    for filename, frame, z in (("ground_oim_xarm6.urdf", "ground", -0.029),
                                ("platform.urdf", "platform", -0.0145)):
        model, = parser.AddModels(str(urdf / filename))
        plant.WeldFrames(plant.GetFrameByName("xarm6_link_base", arm),
                         plant.GetFrameByName(frame, model), RigidTransform([0, 0, z]))
    for index, angle in enumerate(settings["robot_joints_rad"], 1):
        plant.GetJointByName(f"xarm6_joint{index}", arm).set_default_angle(angle)
    if settings["obstacle_file"]:
        # The scene's static SDF already expresses its geometry in world space.
        parser.AddModels(settings["obstacle_file"])
    if settings["raw_mesh"]:
        objects = parser.AddModelsFromString(
            mesh_urdf(settings["object_file"], settings["mesh_scale"]), "urdf")
    else:
        objects = parser.AddModels(settings["object_file"])
    if len(objects) != 1:
        raise ValueError("Object preview requires one movable model")
    model = objects[0]
    children = {plant.get_joint(j).child_body().index() for j in plant.GetJointIndices(model)}
    roots = [plant.get_body(b) for b in plant.GetBodyIndices(model) if b not in children]
    if len(roots) != 1:
        raise ValueError("Object preview requires one free base; use a rigid object model")
    if settings["rpy_degrees"] is None:
        q = np.asarray(settings["quaternion_wxyz"], dtype=float)
        transform = RigidTransform(Quaternion(q / np.linalg.norm(q)), settings["position_m"])
    else:
        transform = RigidTransform(RollPitchYaw(np.radians(settings["rpy_degrees"])), settings["position_m"])
    plant.SetDefaultFloatingBaseBodyPose(roots[0], transform)
    overlay_ids = set()
    if settings["frames"]:
        for body in (plant.world_body(), roots[0]):
            overlay_ids.update(AddFrameTriadIllustration(
                plant=plant, scene_graph=scene_graph, body=body,
                length=0.06, radius=0.002,
            ))
    for index, point in enumerate(settings.get("ee_sample_report", {}).get("points_world", [])):
        overlay_ids.add(plant.RegisterVisualGeometry(
            plant.world_body(), RigidTransform(point), Sphere(settings["ee_sampling"]["ee_radius"]),
            f"ee_sample_{index:04d}", [0.05, 0.65, 0.95, 1.0],
        ))
    plant.SetUseSampledOutputPorts(False)
    plant.Finalize()
    hidden_ids = set()
    if settings["hide_robot"]:
        for model_instance in (arm, tool):
            for body_index in plant.GetBodyIndices(model_instance):
                body = plant.get_body(body_index)
                hidden_ids.update(plant.GetVisualGeometriesForBody(body))
                hidden_ids.update(plant.GetCollisionGeometriesForBody(body))
    prepare_render_geometry(scene_graph, plant.get_source_id(), directory,
                            settings["geometry"], overlay_ids, hidden_ids)
    scene_graph.AddRenderer("preview", MakeRenderEngineVtk(RenderEngineVtkParams(
        backend="EGL", default_clear_color=[0.94, 0.96, 0.98],
        lights=[LightParameter(type="directional", frame="camera", direction=[0, 0, 1], intensity=0.85),
                LightParameter(type="directional", frame="world", direction=[0, 0, -1], intensity=0.55)],
    )))
    target = np.asarray(settings["position_m"]) + [0, 0, 0.02]
    if settings["view"] == "top":
        camera_pose = RigidTransform(RotationMatrix.MakeXRotation(math.pi), target + [0, 0, 0.8])
    elif settings["view"] == "object":
        camera_pose = look_at(target + [0.3, -0.3, 0.25], target)
    else:
        camera_pose = look_at(EYE, TARGET)
    core = RenderCameraCore("preview", CameraInfo(settings["width"], settings["height"], 0.9),
                            ClippingRange(0.01, 100), RigidTransform())
    sensor = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(), camera_pose,
                                        DepthRenderCamera(core, DepthRange(0.01, 50))))
    builder.Connect(scene_graph.get_query_output_port(), sensor.query_object_input_port())
    return builder.Build(), plant, sensor

def render_image(settings):
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    settings = dict(settings)
    sampling = settings.get("ee_sampling")
    if sampling is not None:
        settings["ee_sample_report"] = sample_ee_candidates(settings, sampling["count"], sampling["seed"])
    with tempfile.TemporaryDirectory(prefix="mesh_preview_") as directory:
        diagram, _, sensor = build_preview(settings, directory)
        context = diagram.CreateDefaultContext()
        camera_context = sensor.GetMyContextFromRoot(context)
        pixels = sensor.color_image_output_port().Eval(camera_context).data
        image = Image.fromarray(np.array(pixels, copy=True)[:, :, :3])
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default(size=16)
        title = f'{settings["object_name"]} | {settings["scene"]} | {settings["pose"]} | {settings["geometry"]}'
        position = ", ".join(f"{value:.4f}" for value in settings["position_m"])
        draw.rectangle((0, 0, image.width, 72 if sampling else 48), fill=(24, 32, 45))
        draw.text((10, 5), title, fill="white", font=font)
        draw.text((10, 26), f"Object origin in world: [{position}] m", fill="white", font=font)
        if sampling:
            draw.text((10, 49), f'Blue: {sampling["count"]} EE candidate previews | seed {sampling["seed"]}',
                      fill=(105, 215, 255), font=font)
        output = Path(settings["output"])
        output.parent.mkdir(parents=True, exist_ok=True)
        image.save(output, format="PNG")
        if sampling:
            report = dict(settings["ee_sample_report"])
            report.update(object_name=settings["object_name"], scene=settings["scene"], pose=settings["pose"],
                          object_position_world=settings["position_m"],
                          quaternion_wxyz=settings["quaternion_wxyz"], rpy_degrees=settings["rpy_degrees"])
            output.with_suffix(".ee_samples.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=SCENES, default="open_task")
    parser.add_argument("--start", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--goal", type=int, choices=range(1, 6), default=1)
    parser.add_argument("--pose", choices=("start", "goal"), default="start")
    parser.add_argument("--goal-yaw-degrees", type=int, choices=(90, 0, -90))
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--model", type=Path, help="Preview an SDF, URDF, or MJCF XML with one free base")
    source.add_argument("--mesh", type=Path, help="Preview a raw OBJ as visual geometry")
    parser.add_argument("--mesh-scale", type=float, help="Uniform OBJ scale; default 1 (metres)")
    parser.add_argument("--position", type=float, nargs=3, metavar=("X", "Y", "Z"), help="World position in metres")
    parser.add_argument("--rpy-degrees", type=float, nargs=3, metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--frames", action="store_true", help="Show world and object axes (X red, Y green, Z blue)")
    parser.add_argument("--hide-robot", action="store_true", help="Hide the arm and tool to expose the object and samples")
    parser.add_argument("--view", choices=("scene", "object", "top"), default="scene")
    parser.add_argument("--geometry", choices=("visual", "collision"), default="visual")
    parser.add_argument("--ee-samples", nargs="?", const=64, type=int, metavar="COUNT",
                        help="Overlay reproducible EE candidate previews; default 64; save coordinates as JSON")
    parser.add_argument("--sample-seed", type=int, help="EE preview random seed; default 42")
    parser.add_argument("--sample-height", type=float, metavar="Z",
                        help="Raw OBJ EE centre height in world metres; default -0.012 (17 mm above table)")
    parser.add_argument("--width", type=int, default=1280, help="PNG width in pixels")
    parser.add_argument("--height", type=int, default=960, help="PNG height in pixels")
    parser.add_argument("--output", "-o", type=Path, help="PNG path; default results/previews/<selection>.png")
    parser.add_argument("--dry-run", action="store_true", help="Print selected files, pose, and image path without rendering")
    args = parser.parse_args(argv)
    try:
        settings = preview_settings(args)
        print(json.dumps(settings, indent=2), flush=True)
        if not args.dry_run:
            output = render_image(settings)
            print(f"Saved image: {output}", flush=True)
            if settings["ee_sampling"]:
                print(f'Saved samples: {output.with_suffix(".ee_samples.json")}', flush=True)
    except KeyboardInterrupt:
        return
    except (RuntimeError, ValueError, OSError, KeyError, TypeError, ImportError, ET.ParseError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
