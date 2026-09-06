#include "sampling_c3_utils.h"
#include <array>
#include <iostream>
#include "common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;


ModelInstanceIndex AddFrankaToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    const bool& include_ee,
                                    const bool& include_ground_and_platform,
                                    const bool& include_walls) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("panda_link0"), X_WI);

  if (include_ee) {
    parser.AddModels(FindResourceOrThrow(kEndEffectorModel));
    RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
        drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
      kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"), T_EE_W);
    }

  if (include_ground_and_platform) {
    parser.AddModels(FindResourceOrThrow(kGroundModel));
    parser.AddModels(FindResourceOrThrow(kPlatformModel));

    RigidTransform<double> X_F_P = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kFrankaToPlatformOffset);
    RigidTransform<double> X_F_G_franka = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kFrankaToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("ground"), X_F_G_franka);
    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("platform"), X_F_P);
  }

  if (include_walls) {
    AddWallsToPlant(plant, scene_graph);
  }

  return franka_index;
}

ModelInstanceIndex AddXarm6ToPlant(MultibodyPlant<double>* plant,
                                   SceneGraph<double>* scene_graph,
                                   const bool& include_ee,
                                   const bool& include_ground_and_platform,
                                   const bool& include_walls) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  // Note: Drake's MJCF parser automatically welds the jointless base body
  // (xarm6_link_base) to the world at identity, so no explicit weld is
  // needed (adding one throws a duplicate-joint error).
  ModelInstanceIndex xarm6_index =
      parser.AddModels(FindResourceOrThrow(kXarm6Model))[0];

  // Faithful MJX plant: 5 actuated joints (joint6 welded out in the MJCF).
  // Drake's MJCF parser does not import MuJoCo velocity actuators, so
  // recreate one torque input per revolute joint with the vendor effort
  // limits; velocity limits match the MuJoCo ctrlrange +-0.5 rad/s. The
  // parser also ignores the joint-4 stiffness/springref attributes, so add
  // the passive spring (k = 175 N*m/rad about 0) as a Drake force element.
  const std::array<const char*, 5> xarm6_joints = {
      "xarm6_joint1", "xarm6_joint2", "xarm6_joint3",
      "xarm6_joint4", "xarm6_joint5"};
  const std::array<double, 5> xarm6_effort_limits = {50, 50, 32, 32, 32};
  const double kXarm6VelocityLimit = 0.5;
  for (int i = 0; i < 5; ++i) {
    auto& joint = plant->GetMutableJointByName<
        drake::multibody::RevoluteJoint>(xarm6_joints[i]);
    joint.set_velocity_limits(
        Eigen::VectorXd::Constant(1, -kXarm6VelocityLimit),
        Eigen::VectorXd::Constant(1, kXarm6VelocityLimit));
    const auto& actuator_const = plant->AddJointActuator(
        std::string(xarm6_joints[i]) + "_actuator", joint,
        xarm6_effort_limits[i]);
    auto& actuator =
        plant->get_mutable_joint_actuator(actuator_const.index());
    // MJCF `armature="1"` (per-DOF rotor inertia) is silently lost when the
    // parser drops the <velocity> actuators; without it the wrist DOFs sit
    // far below the discrete stability limit of the kv velocity servo and
    // chatter at ~300 Hz, consuming all torque authority (measured: q static
    // while |qdot| oscillates 0.3-0.7 rad/s). Restore the MuJoCo inertia.
    actuator.set_default_gear_ratio(1.0);
    actuator.set_default_rotor_inertia(1.0);
  }
  plant->AddForceElement<drake::multibody::RevoluteSpring>(
      plant->GetJointByName<drake::multibody::RevoluteJoint>("xarm6_joint4"),
      0.0, 175.0);

  if (include_ee) {
    parser.AddModels(FindResourceOrThrow(kEndEffectorModel));
    RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
        drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
      kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("xarm6_link6"),
                      plant->GetFrameByName("end_effector_flange"), T_EE_W);
  }

  if (include_ground_and_platform) {
    parser.AddModels(FindResourceOrThrow(kGroundModel));
    parser.AddModels(FindResourceOrThrow(kPlatformModel));

    RigidTransform<double> X_F_P = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kFrankaToPlatformOffset);
    RigidTransform<double> X_F_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kFrankaToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("xarm6_link_base"),
                      plant->GetFrameByName("ground"), X_F_G);
    plant->WeldFrames(plant->GetFrameByName("xarm6_link_base"),
                      plant->GetFrameByName("platform"), X_F_P);
  }

  if (include_walls) {
    AddWallsToPlant(plant, scene_graph);
  }

  return xarm6_index;
}

void AddWallsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const bool& include_back_wall) {
  Eigen::Vector3d side_wall_size(kWallLengthX, kWallWidth, kWallHeight);
  AddBoxToPlant(plant, scene_graph, side_wall_size, "left_wall");
  AddBoxToPlant(plant, scene_graph, side_wall_size, "right_wall");
  Eigen::Vector3d wall_size(kWallWidth, kWallLengthY+2*kWallWidth, kWallHeight);
  AddBoxToPlant(plant, scene_graph, wall_size, "front_wall");

  RigidTransform<double> X_G_LW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToLeftWallOffset);
  RigidTransform<double> X_G_RW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToRightWallOffset);
  RigidTransform<double> X_G_FW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToFrontWallOffset);

  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("left_wall"), X_G_LW);
  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("right_wall"), X_G_RW);
  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("front_wall"), X_G_FW);

  if (include_back_wall) {
    AddBoxToPlant(plant, scene_graph, wall_size, "back_wall");
    RigidTransform<double> X_G_BW = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kGroundToBackWallOffset);
    plant->WeldFrames(plant->GetFrameByName("ground"),
                      plant->GetFrameByName("back_wall"), X_G_BW);
  }
}

void AddBoxToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const Eigen::Vector3d& box_size,
    const std::string& box_name) {
  ModelInstanceIndex model_instance_index = plant->AddModelInstance(box_name);
  const drake::multibody::RigidBody<double>& body = plant->AddRigidBody(
    box_name,
    model_instance_index,
    drake::multibody::SpatialInertia<double>::SolidBoxWithMass(
      1.0, box_size(0)/2, box_size(1)/2, box_size(2)/2));

  plant->RegisterVisualGeometry(
    body, RigidTransform<double>::Identity(),
    drake::geometry::Box(box_size(0), box_size(1), box_size(2)),
    box_name, kWallColor);
  plant->RegisterCollisionGeometry(
    body, RigidTransform<double>::Identity(),
    drake::geometry::Box(box_size(0), box_size(1), box_size(2)),
    box_name, kWallFriction);
}

ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& object_model) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  return parser.AddModels(FindResourceOrThrow(object_model))[0];
}

std::vector<ModelInstanceIndex> AddObjectsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  std::vector<ModelInstanceIndex> models;
  for (const auto& model : object_models) {
      models.push_back(
        parser.AddModels(FindResourceOrThrow(model))[0]
      );
  }
  return models;
}

void AddLCSModelToPlant(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    const std::string& object_model,
    const bool& include_end_effector_orientation,
    const bool& include_walls) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_DEMAND(!include_end_effector_orientation);

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);
  parser_lcs.AddModels(object_model);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("base_link"), X_WI);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("ground"), X_W_G);

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }
}


std::vector<ModelInstanceIndex> AddLCSModelsToPlant(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models,
    const bool& include_end_effector_orientation,
    const bool& include_walls) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_ASSERT(!include_end_effector_orientation);

  std::vector<ModelInstanceIndex> obj_models;

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);

  for (const auto& model : object_models) {
    obj_models.push_back(
      parser_lcs.AddModels(FindResourceOrThrow(model))[0]
    );
  }

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("base_link"), X_WI);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("ground"), X_W_G);

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }

  return obj_models;
}

}   // namespace dairlib
