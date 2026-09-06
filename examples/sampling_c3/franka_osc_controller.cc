
#include <dairlib/lcmt_object_state.hpp>
#include <dairlib/lcmt_radio_out.hpp>
#include <dairlib/lcmt_timestamped_saved_traj.hpp>

#include <algorithm>
#include <array>
#include <limits>

#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/lcm_channels.h"
#include "examples/sampling_c3/parameter_headers/osc_params.h"
#include "systems/controllers/osc/end_effector_force.h"
#include "systems/controllers/osc/end_effector_orientation.h"
#include "systems/controllers/osc/end_effector_position.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/gravity_compensator.h"
#include "systems/controllers/osc/external_force_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/lcm_trajectory_systems.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

using systems::controllers::ExternalForceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo_name, "jacktoy",
              "Demo within sampling_c3; used to find controller params file");
DEFINE_string(robot_model, "franka",
              "Robot arm model: 'franka' (default) or 'xarm6'.");
DEFINE_bool(xarm6_five_joint, true,
            "xArm6 only: use the faithful 5-joint velocity-control execution "
            "adapter (damped 5x5 task Jacobian -> joint velocity commands -> "
            "per-joint velocity servos + gravity compensation) instead of the "
            "torque OSC. Matches the intended MJX execution architecture.");
DEFINE_bool(prelift_release, true,
            "xArm6 only: insert a contact-release phase that retreats the tip "
            "planarly away from the object before a commanded lift, avoiding "
            "scooping the object while wedged against it.");

namespace {

// Execution-layer contact-release filter (xArm6 only). Sits between the LCM
// trajectory subscriber and the OSC trajectory receivers. When the incoming
// end_effector_position_target commands a rising z (lift signature) while the
// tip is still wedged against the object, it substitutes a planar retreat
// target (holding current z) until the tip clears release_distance, then
// passes the original trajectory through unchanged. No planner state or
// policy message is modified upstream.
class PreliftReleaseSystem : public drake::systems::LeafSystem<double> {
 public:
  PreliftReleaseSystem(const drake::multibody::MultibodyPlant<double>& plant,
                       drake::systems::Context<double>* context,
                       std::string end_effector_name)
      : plant_(plant),
        context_(context),
        end_effector_name_(std::move(end_effector_name)) {
    this->set_name("prelift_release");
    traj_port_ =
        this->DeclareAbstractInputPort(
                "lcmt_timestamped_saved_traj",
                drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
            .get_index();
    state_port_ =
        this->DeclareVectorInputPort(
                "x, u, t",
                systems::OutputVector<double>(plant.num_positions(),
                                              plant.num_velocities(),
                                              plant.num_actuators()))
            .get_index();
    object_state_port_ =
        this->DeclareAbstractInputPort(
                "lcmt_object_state", drake::Value<dairlib::lcmt_object_state>{})
            .get_index();
    this->DeclareAbstractOutputPort("filtered_traj",
                                    &PreliftReleaseSystem::CalcOutput);
  }

  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(traj_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }

 private:
  static constexpr double kLiftSignatureDz = 0.02;    // m
  static constexpr double kWedgeRadius = 0.085;       // m
  static constexpr double kReleaseDistance = 0.095;   // m
  static constexpr double kMaxRetreatStep = 0.01;     // m per command

  void CalcOutput(const drake::systems::Context<double>& context,
                  dairlib::lcmt_timestamped_saved_traj* output) const {
    const auto& msg =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(context,
                                                                   traj_port_);
    *output = *msg;
    if (msg->utime <= 1e-3) return;

    // Locate the position target block.
    int block_i = -1;
    for (int i = 0; i < msg->saved_traj.num_trajectories; ++i) {
      if (msg->saved_traj.trajectory_names[i] ==
          "end_effector_position_target") {
        block_i = i;
        break;
      }
    }
    if (block_i < 0) return;
    auto& block = output->saved_traj.trajectories[block_i];
    if (block.num_datatypes < 3 || block.num_points < 1) return;

    // Lift signature: last knot z minus first knot z.
    const int n = block.num_points;
    const bool lift_commanded =
        (block.datapoints[2][n - 1] - block.datapoints[2][0]) >
        kLiftSignatureDz;
    if (!lift_commanded) {
      // New pushing/repositioning segment: re-arm for the next lift.
      released_ = false;
      retreat_active_ = false;
      return;
    }

    const systems::OutputVector<double>* robot_output =
        (systems::OutputVector<double>*)this->EvalVectorInput(context,
                                                              state_port_);
    const auto& object_msg =
        this->EvalInputValue<dairlib::lcmt_object_state>(context,
                                                         object_state_port_);
    if (object_msg->num_positions < 7) return;  // No object state yet.
    const double t = robot_output->get_timestamp();

    // FK of the tip.
    plant_.SetPositions(context_, robot_output->GetPositions());
    const Eigen::Vector3d tip =
        plant_
            .EvalBodyPoseInWorld(*context_,
                                 plant_.GetBodyByName(end_effector_name_))
            .translation();
    // Object state layout: quaternion (4) then xyz (3).
    const Eigen::Vector3d object_center(object_msg->position[4],
                                        object_msg->position[5],
                                        object_msg->position[6]);
    const Eigen::Vector2d planar = tip.head<2>() - object_center.head<2>();
    const double distance = planar.norm();

    if (released_) return;
    if (distance >= kReleaseDistance) {
      if (retreat_active_) {
        std::cout << "PRELIFT_RELEASE=DONE t=" << t
                  << " tip=" << tip.transpose()
                  << " object=" << object_center.transpose()
                  << " dist=" << distance << std::endl;
      }
      released_ = true;
      retreat_active_ = false;
      return;
    }
    if (!retreat_active_) {
      if (distance >= kWedgeRadius) {
        // Not wedged; let the lift proceed as commanded.
        released_ = true;
        return;
      }
      retreat_active_ = true;
      std::cout << "PRELIFT_RELEASE=START t=" << t
                << " tip=" << tip.transpose()
                << " object=" << object_center.transpose()
                << " dist=" << distance << std::endl;
    }

    // Retreat: planar step away from the object center, holding current z.
    Eigen::Vector2d dir =
        (distance > 1e-6) ? Eigen::Vector2d(planar / distance)
                          : Eigen::Vector2d(1.0, 0.0);
    Eigen::Vector3d retreat_target = tip;
    retreat_target.head<2>() +=
        dir * std::min(kMaxRetreatStep, kReleaseDistance - distance + 0.002);
    for (int col = 0; col < n; ++col) {
      for (int row = 0; row < block.num_datatypes; ++row) {
        block.datapoints[row][col] = (row < 3) ? retreat_target[row] : 0.0;
      }
    }
  }

  drake::systems::InputPortIndex traj_port_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex object_state_port_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::string end_effector_name_;
  mutable bool released_ = false;
  mutable bool retreat_active_ = false;
};

// Faithful xArm6 execution adapter (--xarm6_five_joint): replaces the torque
// OSC with the intended MJX architecture. The plant has 5 velocity-servo
// joints (joint6 welded). Each tick:
//   v_task[0:3] = v_des + Kc*(p_des - p_tip)               (Cartesian loop)
//   v_task[3]   = wx-regulation of the tool tilt            (see below)
//   v_task[4]   = wy-regulation of the tool tilt
//   qdot_cmd = (J^T J + lambda^2 I)^{-1} J^T v_task, clamp +-0.5 rad/s
//   tau_i = clamp(kv_i*(qdot_cmd_i - qdot_i), +-effort_i) + tau_gravity_i
// This is the same damped square 5x5 inversion (task rows [vx,vy,vz,wx,wy],
// stick yaw dropped) the MJX harness uses, on top of the same software
// velocity servo + gravcomp substrate the oim_t bridge validated.
class Xarm6FiveJointVelocityExecutor
    : public drake::systems::LeafSystem<double> {
 public:
  Xarm6FiveJointVelocityExecutor(
      const drake::multibody::MultibodyPlant<double>& plant,
      std::string end_effector_name)
      : plant_(plant),
        context_(plant.CreateDefaultContext()),
        end_effector_name_(std::move(end_effector_name)) {
    this->set_name("xarm6_five_joint_velocity_executor");
    DRAKE_DEMAND(plant.num_positions() == 5);
    DRAKE_DEMAND(plant.num_velocities() == 5);
    DRAKE_DEMAND(plant.num_actuators() == 5);
    state_port_ =
        this->DeclareVectorInputPort(
                "x, u, t",
                systems::OutputVector<double>(plant.num_positions(),
                                              plant.num_velocities(),
                                              plant.num_actuators()))
            .get_index();
    drake::trajectories::PiecewisePolynomial<double> pp(
        Eigen::Vector3d::Zero());
    trajectory_port_ =
        this->DeclareAbstractInputPort(
                "end_effector_trajectory",
                drake::Value<drake::trajectories::Trajectory<double>>(pp))
            .get_index();
    this->DeclareVectorOutputPort(
        "xarm_torque",
        systems::TimestampedVector<double>(plant.num_actuators()),
        &Xarm6FiveJointVelocityExecutor::CalcTorque);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_trajectory() const {
    return this->get_input_port(trajectory_port_);
  }

 private:
  // Cartesian feedback gain (1/s), tilt regulation gain (1/s), Jacobian
  // damping, servo gains/limits (from the MJX velocity actuators).
  static constexpr double kKc = 4.0;
  static constexpr double kAlpha = 2.0;
  static constexpr double kLambda = 0.05;
  static constexpr double kQdotLimit = 0.5;  // rad/s (MuJoCo ctrlrange)
  static constexpr std::array<double, 5> kKv = {300, 300, 200, 200, 200};
  static constexpr std::array<double, 5> kEffort = {50, 50, 32, 32, 32};

  void CalcTorque(const drake::systems::Context<double>& context,
                  systems::TimestampedVector<double>* output) const {
    const auto* robot_output =
        dynamic_cast<const systems::OutputVector<double>*>(
            this->EvalVectorInput(context, state_port_));
    const auto& traj =
        this->EvalAbstractInput(context, trajectory_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();
    const double timestamp = robot_output->get_timestamp();

    plant_.SetPositionsAndVelocities(context_.get(),
                                     robot_output->GetState());
    const auto& tip_frame =
        plant_.GetBodyByName(end_effector_name_).body_frame();
    const drake::math::RigidTransform<double> X_W_tip =
        plant_.CalcRelativeTransform(*context_, plant_.world_frame(),
                                     tip_frame);
    const Eigen::Vector3d p_tip = X_W_tip.translation();

    // Desired position/velocity from the (FirstOrderHold) trajectory,
    // clamped into its time range.
    const double t = std::clamp(timestamp, traj.start_time(),
                                traj.end_time());
    Eigen::Vector3d p_des = traj.value(t);
    // Robot-specific radial safety clamp: never chase a Cartesian target
    // beyond the xArm6's usable planar reach (r2 trial2: an unreachable
    // r=0.67 lift-height target parked the arm at saturation and pushed the
    // measured EE past the planner's radius assert). Direction-preserving:
    // scale only the planar components down to the reach circle.
    constexpr double kMaxPlanarReach = 0.68;
    const double r_des = p_des.head<2>().norm();
    if (r_des > kMaxPlanarReach) {
      p_des.head<2>() *= kMaxPlanarReach / r_des;
    }
    // Terminal hold: beyond the trajectory's time range Drake extrapolates
    // the last polynomial segment, and the measured slope exactly cancelled
    // the Kc feedback (v_des == -Kc*(p_des - p_tip) captured live), nulling
    // v_task and freezing the arm -- the true identity of the r4/r5
    // "kinematic traps". Match the Panda path's terminal-hold semantics:
    // outside the range, hold position with zero desired velocity.
    const bool in_range =
        (t >= traj.start_time()) && (t <= traj.end_time());
    const Eigen::Vector3d v_des =
        in_range ? Eigen::Vector3d(traj.EvalDerivative(t, 1))
                 : Eigen::Vector3d::Zero();

    // 6x5 spatial Jacobian of the tip; rows [angular(3); translational(3)].
    Eigen::MatrixXd J_full(6, plant_.num_velocities());
    plant_.CalcJacobianSpatialVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kV, tip_frame,
        Eigen::Vector3d::Zero(), plant_.world_frame(), plant_.world_frame(),
        &J_full);
    // Square 5x5 task Jacobian: rows [vx, vy, vz, wx, wy] (stick yaw wz
    // dropped -- axisymmetric tool, exactly the MJX construction).
    Eigen::Matrix<double, 5, 5> J;
    J.topRows<3>() = J_full.bottomRows<3>();
    J.bottomRows<2>() = J_full.topRows<2>();

    // Tool tilt regulation. a = R_tip * ez is the tool axis in world; at the
    // welded orientation the tool is vertical with a = +ez (verified
    // numerically at q_init). For a generally signed axis s = sign(a_z),
    // driving a -> s*ez requires wx = s*alpha*a_y, wy = -s*alpha*a_x
    // (from da = omega x a with a ~ s*ez).
    const Eigen::Vector3d a =
        X_W_tip.rotation().matrix() * Eigen::Vector3d::UnitZ();
    const double s = (a.z() >= 0.0) ? 1.0 : -1.0;

    Eigen::Matrix<double, 5, 1> v_task;
    v_task.head<3>() = v_des + kKc * (p_des - p_tip);
    v_task(3) = s * kAlpha * a.y();
    v_task(4) = -s * kAlpha * a.x();

    // Weighted damped square solve, then clamp to the MuJoCo ctrlrange.
    // Row weights make translation primary and tilt a soft BIAS — matching
    // the MJX architecture, where tilt regulation is an additive bias on the
    // sampled qdot, never an equal-priority constraint. Equal weighting
    // (round 4) let the tilt rows pin the solve in a kinematic trap: the tip
    // stalled 6 cm from a reachable target with saturated qdot for minutes,
    // the servo pressed into the wedge, and the sim eventually went NaN.
    static const Eigen::Matrix<double, 5, 1> kRowW =
        (Eigen::Matrix<double, 5, 1>() << 1.0, 1.0, 1.0, 0.2, 0.2).finished();
    const Eigen::Matrix<double, 5, 5> Jw = kRowW.asDiagonal() * J;
    const Eigen::Matrix<double, 5, 5> JtJ =
        Jw.transpose() * Jw +
        kLambda * kLambda * Eigen::Matrix<double, 5, 5>::Identity();
    Eigen::Matrix<double, 5, 1> qdot_cmd =
        JtJ.ldlt().solve(Jw.transpose() *
                         (kRowW.asDiagonal() * v_task));
    if (!qdot_cmd.allFinite()) {
      qdot_cmd.setZero();  // NaN guard: hold rather than propagate.
    }
    // Direction-preserving saturation: componentwise clamping distorts the
    // task direction (measured: descent stalls at z=+0.05 while xy tracks —
    // the clamped joint mix cancels the z motion). Scale the whole vector so
    // max |qdot_i| == kQdotLimit, preserving the damped-J task direction.
    const double qdot_max = qdot_cmd.cwiseAbs().maxCoeff();
    if (qdot_max > kQdotLimit) {
      qdot_cmd *= kQdotLimit / qdot_max;
    }

    // Velocity servo + gravity compensation (MuJoCo gravcomp ordering: the
    // gravity term sits outside the actuator force clamp).
    const Eigen::VectorXd gravity_compensation =
        -plant_.MakeActuationMatrix().transpose() *
        plant_.CalcGravityGeneralizedForces(*context_);
    const Eigen::VectorXd qdot = robot_output->GetVelocities();
    Eigen::VectorXd tau(5);
    for (int i = 0; i < 5; ++i) {
      tau(i) = std::clamp(kKv[i] * (qdot_cmd(i) - qdot(i)), -kEffort[i],
                          kEffort[i]) +
               gravity_compensation(i);
    }
    output->SetDataVector(tau);
    output->set_timestamp(timestamp);

    if (timestamp >= last_print_time_ + 1.0) {
      last_print_time_ = timestamp;
      std::cout << "XARM6_5J t=" << timestamp
                << " p_des=" << p_des.transpose()
                << " p_tip=" << p_tip.transpose()
                << " |qdot_cmd|=" << qdot_cmd.norm()
                << " v_des=" << v_des.transpose()
                << " v_task=" << v_task.transpose() << std::endl;
    }
  }

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex trajectory_port_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::string end_effector_name_;
  mutable double last_print_time_{-std::numeric_limits<double>::infinity()};
};

}  // namespace

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  std::string controller_params_path = "examples/sampling_c3/" +
    FLAGS_demo_name + "/parameters/sampling_c3_controller_params.yaml";
  SamplingC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<SamplingC3ControllerParams>(
          controller_params_path);
  SamplingC3OSCParams osc_params =
      drake::yaml::LoadYamlFile<SamplingC3OSCParams>(
          controller_params.osc_params_file);
  std::string lcm_channels_file = FLAGS_is_simulation ?
      controller_params.lcm_channels_simulation_file :
      controller_params.lcm_channels_hardware_file;
  SamplingC3LcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<SamplingC3LcmChannels>(lcm_channels_file);
  drake::solvers::SolverOptions solver_options =
      drake::yaml::LoadYamlFile<solvers::SolverOptionsFromYaml>(
          FindResourceOrThrow(controller_params.osc_qp_settings_file))
          .GetAsSolverOptions(drake::solvers::OsqpSolver::id());

  // Create a Franka-only plant.
  drake::multibody::MultibodyPlant<double> plant(0.0);
  if (FLAGS_robot_model == "xarm6") {
    AddXarm6ToPlant(&plant);
  } else {
    AddFrankaToPlant(&plant);
  }
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Faithful xArm6 5-joint velocity-control execution path: bypass the OSC
  // diagram entirely. The Cartesian trajectory contract (subscriber,
  // prelift-release filter, position receiver, trajectory generator) is
  // preserved; the executor maps it to joint velocity servo torques.
  if (FLAGS_robot_model == "xarm6" && FLAGS_xarm6_five_joint) {
    DiagramBuilder<double> builder;
    auto state_receiver =
        builder.AddSystem<systems::RobotOutputReceiver>(plant);
    auto end_effector_trajectory_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
            lcm_channel_params.tracking_trajectory_actor_channel, &lcm));
    auto end_effector_position_receiver =
        builder.AddSystem<systems::LcmTrajectoryReceiver>(
            "end_effector_position_target");
    auto radio_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
            lcm_channel_params.radio_channel, &lcm));
    auto end_effector_trajectory =
        builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(
            plant, plant_context.get(), osc_params.neutral_position,
            osc_params.teleop_neutral_position, kEndEffectorName);
    end_effector_trajectory->SetRemoteControlParameters(
        osc_params.neutral_position, osc_params.x_scale, osc_params.y_scale,
        osc_params.z_scale);
    auto executor = builder.AddSystem<Xarm6FiveJointVelocityExecutor>(
        plant, kEndEffectorName);
    auto franka_command_sender =
        builder.AddSystem<systems::RobotCommandSender>(plant);
    auto franka_command_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
            lcm_channel_params.franka_input_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));

    // Optional prelift-release filter between subscriber and receiver.
    PreliftReleaseSystem* prelift_release = nullptr;
    if (FLAGS_prelift_release) {
      auto object_state_sub = builder.AddSystem(
          LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
              lcm_channel_params.object_state_channels.at(0), &lcm));
      prelift_release = builder.AddSystem<PreliftReleaseSystem>(
          plant, plant_context.get(), kEndEffectorName);
      std::cout << "PRELIFT_RELEASE system enabled (channel "
                << lcm_channel_params.object_state_channels.at(0) << ")"
                << std::endl;
      builder.Connect(end_effector_trajectory_sub->get_output_port(),
                      prelift_release->get_input_port_trajectory());
      builder.Connect(state_receiver->get_output_port(0),
                      prelift_release->get_input_port_state());
      builder.Connect(object_state_sub->get_output_port(),
                      prelift_release->get_input_port_object_state());
    }
    const auto& actor_traj_port =
        (prelift_release != nullptr)
            ? prelift_release->get_output_port()
            : end_effector_trajectory_sub->get_output_port();
    builder.Connect(
        actor_traj_port,
        end_effector_position_receiver->get_input_port_trajectory());
    builder.Connect(end_effector_position_receiver->get_output_port(0),
                    end_effector_trajectory->get_input_port_trajectory());
    builder.Connect(state_receiver->get_output_port(0),
                    end_effector_trajectory->get_input_port_state());
    builder.Connect(radio_sub->get_output_port(0),
                    end_effector_trajectory->get_input_port_radio());
    builder.Connect(state_receiver->get_output_port(0),
                    executor->get_input_port_state());
    builder.Connect(end_effector_trajectory->get_output_port(0),
                    executor->get_input_port_trajectory());
    builder.Connect(executor->get_output_port(0),
                    franka_command_sender->get_input_port(0));
    builder.Connect(franka_command_sender->get_output_port(),
                    franka_command_pub->get_input_port());

    auto owned_diagram = builder.Build();
    std::shared_ptr<Diagram<double>> shared_diagram =
        std::move(owned_diagram);
    shared_diagram->set_name("sampling_c3_xarm6_five_joint_controller");
    DrawAndSaveDiagramGraph(*shared_diagram);
    std::cout << "XARM6_5J executor active: Kc=4.0 alpha=2.0 lambda=0.05 "
              << "kv=[300,300,200,200,200] efforts=[50,50,32,32,32] "
              << "qdot_limit=0.5" << std::endl;
    systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
        &lcm, shared_diagram, state_receiver,
        lcm_channel_params.franka_state_channel, true);
    loop.Simulate();
    return 0;
  }

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto end_effector_trajectory_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          lcm_channel_params.tracking_trajectory_actor_channel, &lcm));
  auto end_effector_position_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_position_target");
  auto end_effector_force_receiver =
      builder.AddSystem<systems::LcmTrajectoryReceiver>(
          "end_effector_force_target");
  auto end_effector_orientation_receiver =
      builder.AddSystem<systems::LcmOrientationTrajectoryReceiver>(
          "end_effector_orientation_target");
  auto franka_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.franka_input_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto osc_command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.osc_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  auto franka_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto osc_command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);
  auto end_effector_trajectory =
      builder.AddSystem<EndEffectorPositionTrajectoryGenerator>(
          plant, plant_context.get(), osc_params.neutral_position,
          osc_params.teleop_neutral_position, kEndEffectorName);
  end_effector_trajectory->SetRemoteControlParameters(
      osc_params.neutral_position, osc_params.x_scale,
      osc_params.y_scale, osc_params.z_scale);
  auto end_effector_orientation_trajectory =
      builder.AddSystem<EndEffectorOrientationTrajectoryGenerator>();
  end_effector_orientation_trajectory->SetTrackOrientation(
      osc_params.track_end_effector_orientation);
  auto end_effector_force_trajectory =
      builder.AddSystem<EndEffectorForceTrajectoryGenerator>();
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm));
  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant_context.get(), false);
  if (osc_params.publish_debug_info) {
    auto osc_debug_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
            lcm_channel_params.osc_debug_channel, &lcm,
            TriggerTypeSet({TriggerType::kForced})));
    builder.Connect(osc->get_output_port_osc_debug(),
                    osc_debug_pub->get_input_port());
  }

  auto end_effector_position_tracking_data =
      std::make_unique<TransTaskSpaceTrackingData>(
          "end_effector_target", osc_params.K_p_end_effector,
          osc_params.K_d_end_effector, osc_params.W_end_effector,
          plant, plant);
  end_effector_position_tracking_data->AddPointToTrack(kEndEffectorName);
  const VectorXd& end_effector_acceleration_limits =
      osc_params.end_effector_acceleration * Vector3d::Ones();
  end_effector_position_tracking_data->SetCmdAccelerationBounds(
      -end_effector_acceleration_limits, end_effector_acceleration_limits);
  // The elbow (panda_joint2) posture task uses the Franka's 7th-DOF
  // redundancy; the 6-DOF xArm6 has none, so only build it for the Franka.
  std::unique_ptr<JointSpaceTrackingData> mid_link_position_tracking_data_for_rel;
  if (FLAGS_robot_model == "franka") {
    mid_link_position_tracking_data_for_rel =
        std::make_unique<JointSpaceTrackingData>(
            "panda_joint2_target", osc_params.K_p_mid_link,
            osc_params.K_d_mid_link, osc_params.W_mid_link, plant,
            plant);
    mid_link_position_tracking_data_for_rel->AddJointToTrack("panda_joint2",
                                                             "panda_joint2dot");
  }

  auto end_effector_force_tracking_data =
      std::make_unique<ExternalForceTrackingData>(
          "end_effector_force", osc_params.W_ee_lambda, plant, plant,
          kEndEffectorName, Vector3d::Zero());

  auto end_effector_orientation_tracking_data =
      std::make_unique<RotTaskSpaceTrackingData>(
          "end_effector_orientation_target",
          osc_params.K_p_end_effector_rot,
          osc_params.K_d_end_effector_rot,
          osc_params.W_end_effector_rot, plant, plant);
  end_effector_orientation_tracking_data->AddFrameToTrack(kEndEffectorName);
  Eigen::VectorXd orientation_target = Eigen::VectorXd::Zero(4);
  orientation_target(0) = 1;
  osc->AddTrackingData(std::move(end_effector_position_tracking_data));
  // Since the Franka has 7 joints to control a 6 DOF EE command, add an
  // additional tracking objective for joint 2 at a good configuration for the
  // sampling C3 experiments.  1.1 joint target empirically works well.
  if (mid_link_position_tracking_data_for_rel != nullptr) {
    osc->AddConstTrackingData(
        std::move(mid_link_position_tracking_data_for_rel),
        1.1 * VectorXd::Ones(1));
  }
  osc->AddTrackingData(std::move(end_effector_orientation_tracking_data));
  osc->AddForceTrackingData(std::move(end_effector_force_tracking_data));
  osc->SetAccelerationCostWeights(osc_params.W_acceleration);
  osc->SetInputCostWeights(osc_params.W_input_regularization);
  osc->SetInputSmoothingCostWeights(osc_params.W_input_smoothing_regularization);
  if (osc_params.enforce_acceleration_constraints) {
    osc->EnableAccelerationConstraints();
  } else {
    osc->DisableAccelerationConstraints();
  }
  osc->SetContactFriction(osc_params.mu);
  osc->SetOsqpSolverOptions(solver_options);

  osc->Build();

  if (osc_params.cancel_gravity_compensation) {
    if (FLAGS_is_simulation) {
      std::cerr<<"Sim OSC needs cancel_gravity_compensation: false"<<std::endl;
      return -1;
      return -1;
    }
    auto gravity_compensator =
        builder.AddSystem<systems::GravityCompensationRemover>(plant,
                                                               *plant_context);
    builder.Connect(osc->get_output_port_osc_command(),
                    gravity_compensator->get_input_port());
    builder.Connect(gravity_compensator->get_output_port(),
                    franka_command_sender->get_input_port());
  } else {
    if (!FLAGS_is_simulation) {
      std::cerr<<"HW OSC needs cancel_gravity_compensation: true"<<std::endl;
      return -1;
    }
    builder.Connect(osc->get_output_port_osc_command(),
                    franka_command_sender->get_input_port(0));
  }

  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_trajectory->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_orientation_trajectory->get_input_port_radio());
  builder.Connect(radio_sub->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_radio());
  builder.Connect(franka_command_sender->get_output_port(),
                  franka_command_pub->get_input_port());
  builder.Connect(osc_command_sender->get_output_port(),
                  osc_command_pub->get_input_port());
  builder.Connect(osc->get_output_port_osc_command(),
                  osc_command_sender->get_input_port(0));

  builder.Connect(state_receiver->get_output_port(0),
                  osc->get_input_port_robot_output());
  // xArm6 execution-layer contact-release phase: filter the incoming
  // trajectory before it reaches the OSC trajectory receivers.
  PreliftReleaseSystem* prelift_release = nullptr;
  if (FLAGS_robot_model == "xarm6" && FLAGS_prelift_release) {
    auto object_state_sub =
        builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
            lcm_channel_params.object_state_channels.at(0), &lcm));
    prelift_release = builder.AddSystem<PreliftReleaseSystem>(
        plant, plant_context.get(), kEndEffectorName);
    std::cout << "PRELIFT_RELEASE system enabled (channel "
              << lcm_channel_params.object_state_channels.at(0) << ")"
              << std::endl;
    builder.Connect(end_effector_trajectory_sub->get_output_port(),
                    prelift_release->get_input_port_trajectory());
    builder.Connect(state_receiver->get_output_port(0),
                    prelift_release->get_input_port_state());
    builder.Connect(object_state_sub->get_output_port(),
                    prelift_release->get_input_port_object_state());
  }
  const auto& actor_traj_port =
      (prelift_release != nullptr)
          ? prelift_release->get_output_port()
          : end_effector_trajectory_sub->get_output_port();
  builder.Connect(actor_traj_port,
                  end_effector_position_receiver->get_input_port_trajectory());
  builder.Connect(actor_traj_port,
                  end_effector_force_receiver->get_input_port_trajectory());
  builder.Connect(
      actor_traj_port,
      end_effector_orientation_receiver->get_input_port_trajectory());
  builder.Connect(end_effector_position_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_trajectory());
  builder.Connect(state_receiver->get_output_port(0),
                  end_effector_trajectory->get_input_port_state());
  builder.Connect(
      end_effector_orientation_receiver->get_output_port(0),
      end_effector_orientation_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_target"));
  builder.Connect(
      end_effector_orientation_trajectory->get_output_port(0),
      osc->get_input_port_tracking_data("end_effector_orientation_target"));
  builder.Connect(end_effector_force_receiver->get_output_port(0),
                  end_effector_force_trajectory->get_input_port_trajectory());
  builder.Connect(end_effector_force_trajectory->get_output_port(0),
                  osc->get_input_port_tracking_data("end_effector_force"));

  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("sampling_c3_franka_osc_controller"));
  DrawAndSaveDiagramGraph(*shared_diagram);
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, state_receiver,
      lcm_channel_params.franka_state_channel, true);
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
