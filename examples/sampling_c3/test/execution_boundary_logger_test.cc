#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "examples/sampling_c3/execution_boundary_logger.h"
#include "systems/robot_lcm_systems.h"
#include "drake/common/name_value.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace {
constexpr double kDt = 0.01;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::State;

void Require(bool condition, const char* description) {
  if (!condition) throw std::runtime_error(description);
}

struct ObjectRecord {
  std::vector<double> q, v;
  template <typename Archive>
  void Serialize(Archive* archive) {
    archive->Visit(DRAKE_NVP(q));
    archive->Visit(DRAKE_NVP(v));
  }
};

struct BoundaryRecord {
  int64_t boundary_step{}, plan_utime{};
  double sim_time{}, wall_time{};
  std::vector<ObjectRecord> objects;
  std::vector<double> robot_q, robot_v;
  std::optional<std::string> reason;
  template <typename Archive>
  void Serialize(Archive* archive) {
    archive->Visit(DRAKE_NVP(boundary_step));
    archive->Visit(DRAKE_NVP(plan_utime));
    archive->Visit(DRAKE_NVP(sim_time));
    archive->Visit(DRAKE_NVP(wall_time));
    archive->Visit(DRAKE_NVP(objects));
    archive->Visit(DRAKE_NVP(robot_q));
    archive->Visit(DRAKE_NVP(robot_v));
    archive->Visit(DRAKE_NVP(reason));
  }
};

class CaptureOutput {
 public:
  explicit CaptureOutput(std::ostringstream& target)
      : previous_(std::cout.rdbuf(target.rdbuf())) {}
  ~CaptureOutput() { std::cout.rdbuf(previous_); }
 private:
  std::streambuf* previous_;
};

class Commands final : public drake::systems::LeafSystem<double> {
 public:
  mutable int updates = 0;
  Commands() {
    dairlib::lcmt_robot_input initial{};
    initial.num_efforts = 1;
    initial.effort_names = {"act"};
    initial.efforts = {0.0};
    this->DeclareAbstractState(drake::Value<dairlib::lcmt_robot_input>(initial));
    this->DeclareAbstractOutputPort("command", &Commands::Output);
    this->DeclarePeriodicUnrestrictedUpdateEvent(kDt, 0.0, &Commands::Update);
  }
 private:
  void Output(const Context<double>& context,
              dairlib::lcmt_robot_input* output) const {
    *output = context.get_abstract_state<dairlib::lcmt_robot_input>(0);
  }
  EventStatus Update(const Context<double>& context, State<double>* state) const {
    ++updates;
    const int tick = std::lround(context.get_time() / kDt);
    auto& next = state->get_mutable_abstract_state<dairlib::lcmt_robot_input>(0);
    next.utime = (tick + 1) * 10000;
    next.source_plan_utime = tick < 2 ? 101 : (tick < 4 ? 202 : 303);
    next.efforts[0] = tick + 1;
    return EventStatus::Succeeded();
  }
};

class Efforts final : public drake::systems::LeafSystem<double> {
 public:
  Efforts() {
    this->DeclareAbstractInputPort("command", drake::Value<dairlib::lcmt_robot_input>{});
    this->DeclareVectorOutputPort("effort", 1, &Efforts::Output);
  }
 private:
  void Output(const Context<double>& context,
              drake::systems::BasicVector<double>* output) const {
    output->SetAtIndex(0, get_input_port().Eval<dairlib::lcmt_robot_input>(context).efforts[0]);
  }
};

struct Run {
  std::map<int, Eigen::VectorXd> states;
  int64_t steps = 0;
  bool budget_caught = false;
  bool goal_caught = false;
  double final_time = 0.0;
  Eigen::VectorXd final_state;
  std::vector<BoundaryRecord> records;
  int command_updates = 0;
};

Run Simulate(bool enabled, int budget = -1,
             std::optional<Eigen::Vector3d> goal = std::nullopt) {
  std::ostringstream captured;
  CaptureOutput output_capture(captured);
  drake::systems::DiagramBuilder<double> builder;
  auto* plant = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(kDt);
  plant->mutable_gravity_field().set_gravity_vector(Eigen::Vector3d::Zero());
  const auto robot = plant->AddModelInstance("robot");
  const auto object = plant->AddModelInstance("object");
  const drake::multibody::SpatialInertia<double> inertia(
      1.0, Eigen::Vector3d::Zero(),
      drake::multibody::UnitInertia<double>::SolidSphere(0.1));
  const auto& robot_body = plant->AddRigidBody("robot_body", robot, inertia);
  plant->AddRigidBody("object_body", object, inertia);
  const auto& joint = plant->AddJoint(
      std::make_unique<drake::multibody::PrismaticJoint<double>>(
          "slide", plant->world_frame(), robot_body.body_frame(),
          Eigen::Vector3d::UnitX()));
  plant->AddJointActuator("act", joint);
  plant->Finalize();
  auto* commands = builder.AddSystem<Commands>();
  auto* efforts = builder.AddSystem<Efforts>();
  builder.Connect(commands->get_output_port(), efforts->get_input_port());
  builder.Connect(efforts->get_output_port(), plant->get_actuation_input_port());
  dairlib::ExecutionBoundaryLogger* logger = nullptr;
  if (enabled) {
    const char* test_tmpdir = std::getenv("TEST_TMPDIR");
    const std::string stop_file =
        std::string(test_tmpdir ? test_tmpdir : "/tmp") +
        "/execution_boundary_stop.json";
    logger = builder.AddSystem<dairlib::ExecutionBoundaryLogger>(
        *plant, robot, std::vector<drake::multibody::ModelInstanceIndex>{object},
        std::vector<std::string>{"OBJECT_TEST_STATE"}, 0.0, budget,
        budget > 0 || goal ? stop_file : "", goal);
    builder.Connect(plant->get_state_output_port(), logger->state_input());
    builder.Connect(commands->get_output_port(), logger->command_input());
  }
  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  Run run;
  simulator.set_monitor([&](const Context<double>& context) {
    const auto& plant_context = diagram->GetSubsystemContext(*plant, context);
    run.states[std::lround(context.get_time() / kDt)] =
        plant->GetPositionsAndVelocities(plant_context);
    return EventStatus::Succeeded();
  });
  simulator.Initialize();
  try {
    simulator.AdvanceTo(0.08);
  } catch (const dairlib::ExecutionStepBudgetReached&) {
    run.budget_caught = true;
  } catch (const dairlib::ExecutionGoalReached&) {
    run.goal_caught = true;
  }
  run.final_time = simulator.get_context().get_time();
  run.final_state = plant->GetPositionsAndVelocities(
      diagram->GetSubsystemContext(*plant, simulator.get_context()));
  if (logger) {
    run.steps = logger->n_steps_executed();
    logger->LogTerminal(diagram->GetSubsystemContext(*logger, simulator.get_context()),
                        run.goal_caught ? "goal_reached" :
                        run.budget_caught ? "step_budget" : "shutdown");
  }
  run.command_updates = commands->updates;
  std::istringstream lines(captured.str());
  std::string line;
  while (std::getline(lines, line)) {
    if (line.rfind("[C3_EXECUTION_BOUNDARY] ", 0) == 0 ||
        line.rfind("[C3_EXECUTION_TERMINAL] ", 0) == 0) {
      // JSON is a subset of YAML; use Drake's existing parser, without adding
      // a new JSON dependency solely for this native regression test.
      run.records.push_back(drake::yaml::LoadYamlString<BoundaryRecord>(
          line.substr(line.find("] ") + 2)));
    }
  }
  return run;
}

void CheckBoundaryStates(const Run& observed, const Run& baseline) {
  Require(observed.records.size() == observed.steps + 1,
          "execution records must contain N+1 physical states");
  Require(observed.records.front().wall_time == 0.0,
          "wall time must start at the exact first boundary");
  for (size_t i = 0; i < observed.records.size(); ++i) {
    const auto& row = observed.records[i];
    Require(row.boundary_step == static_cast<int64_t>(i),
            "boundary sequence is not zero-based");
    const auto& expected = baseline.states.at(std::lround(row.sim_time / kDt));
    Require(row.robot_q.at(0) == expected[0] &&
                row.robot_v.at(0) == expected[8],
            "robot boundary state is not the exact pre-actuation state");
    for (int j = 0; j < 7; ++j) {
      Require(row.objects.at(0).q.at(j) == expected[1 + j],
              "object positions do not match the exact physical boundary");
    }
    for (int j = 0; j < 6; ++j) {
      Require(row.objects.at(0).v.at(j) == expected[9 + j],
              "object velocities do not match the exact physical boundary");
    }
    if (i) {
      Require(row.sim_time > observed.records[i - 1].sim_time &&
                  row.wall_time > observed.records[i - 1].wall_time,
              "execution boundary times must strictly increase");
    }
  }
}

void CheckCommandSenderCompatibility() {
  drake::multibody::MultibodyPlant<double> plant(kDt);
  const auto model = plant.AddModelInstance("robot");
  const auto& body = plant.AddRigidBody(
      "body", model, drake::multibody::SpatialInertia<double>(
                         1.0, Eigen::Vector3d::Zero(),
                         drake::multibody::UnitInertia<double>::SolidSphere(0.1)));
  const auto& joint = plant.AddJoint(
      std::make_unique<drake::multibody::PrismaticJoint<double>>(
          "slide", plant.world_frame(), body.body_frame(),
          Eigen::Vector3d::UnitX()));
  plant.AddJointActuator("act", joint);
  plant.Finalize();
  dairlib::systems::RobotCommandSender original(plant);
  dairlib::systems::RobotCommandSender tagged(plant, true);
  Require(original.num_input_ports() == 1, "default sender input API changed");
  Require(tagged.num_input_ports() == 3, "enabled provenance inputs missing");
  auto original_context = original.CreateDefaultContext();
  auto tagged_context = tagged.CreateDefaultContext();
  dairlib::systems::TimestampedVector<double> command(1);
  command.SetAtIndex(0, 2.25);
  command.set_timestamp(0.125);
  original.get_input_port().FixValue(original_context.get(), command);
  tagged.get_input_port(0).FixValue(tagged_context.get(), command);
  const auto expected = original.get_output_port()
                            .Eval<dairlib::lcmt_robot_input>(*original_context);
  Require(expected.source_plan_utime == 0, "default provenance must be zero");
  dairlib::lcmt_timestamped_saved_traj source{};
  source.utime = 1234;
  source.saved_traj.num_trajectories = 1;
  source.saved_traj.trajectory_names = {"end_effector_position_target"};
  source.saved_traj.trajectories.resize(1);
  auto& target = source.saved_traj.trajectories[0];
  target.num_datatypes = 3;
  target.num_points = 1;
  target.datapoints = {{0.4}, {0.2}, {0.05}};
  dairlib::lcmt_radio_out radio{};
  tagged.get_input_port_source_trajectory().FixValue(tagged_context.get(), source);
  tagged.get_input_port_source_radio().FixValue(tagged_context.get(), radio);
  const auto check = [&](int64_t wanted_source) {
    const auto& actual = tagged.get_output_port()
                             .Eval<dairlib::lcmt_robot_input>(*tagged_context);
    Require(actual.source_plan_utime == wanted_source, "incorrect source tag");
    Require(actual.utime == expected.utime &&
                actual.num_efforts == expected.num_efforts &&
                actual.efforts == expected.efforts &&
                actual.effort_names == expected.effort_names,
            "provenance changed actuator values or command time");
  };
  check(1234);
  radio.channel[14] = 1;
  tagged.get_input_port_source_radio().FixValue(tagged_context.get(), radio);
  check(0);  // Teleoperation does not execute the planner trajectory.
  radio.channel[14] = 0;
  tagged.get_input_port_source_radio().FixValue(tagged_context.get(), radio);
  target.datapoints = {{0.0}, {0.0}, {0.0}};
  tagged.get_input_port_source_trajectory().FixValue(tagged_context.get(), source);
  check(0);  // Empty/zero targets must not invent a policy adoption.
}
}  // namespace

int main() {
  try {
    const auto baseline = Simulate(false);
    std::cout << "CASE logging_enabled" << std::endl;
    const auto observed = Simulate(true);
    Require(baseline.states.size() == observed.states.size(), "observer altered sample schedule");
    for (const auto& [tick, values] : baseline.states) {
      Require((values.array() == observed.states.at(tick).array()).all(),
              "observer altered physical state");
    }
    Require(observed.steps == 3, "expected three adopted policies");
    Require(observed.command_updates > observed.steps,
            "repeated servo commands were not exercised");
    Require(observed.final_time == baseline.final_time, "observer altered end time");
    CheckBoundaryStates(observed, baseline);
    Require(observed.records[0].plan_utime == 101 &&
                observed.records[1].plan_utime == 202 &&
                observed.records[2].plan_utime == 303 &&
                observed.records[3].plan_utime == 303,
            "source metadata did not follow the actually applied commands");
    std::cout << "PASS identical_state_and_time logger_enabled_vs_disabled samples="
              << baseline.states.size() << std::endl;
    std::cout << "CASE execution_budget" << std::endl;
    const auto budget = Simulate(true, 2);
    Require(budget.budget_caught, "exact budget exception was not caught");
    Require(budget.steps == 2, "budget allowed extra policy");
    Require(std::abs(budget.final_time - 0.05) < 1e-12, "incorrect pre-adoption stop time");
    Require((budget.final_state.array() == baseline.states.at(5).array()).all(),
            "budget applied the pending B+1 plant update");
    CheckBoundaryStates(budget, baseline);
    Require(budget.records.back().plan_utime == 202 &&
                budget.records.back().reason == "step_budget",
            "budget terminal must describe the last applied policy");
    std::cout << "PASS budget_stops_before_third_policy_state_apply steps=2 sim_time="
              << budget.final_time << std::endl;
    std::cout << "CASE immediate_goal_stop" << std::endl;
    const auto reached = Simulate(true, -1, Eigen::Vector3d(0.01, 0, 0));
    Require(reached.goal_caught && reached.steps == 1,
            "goal did not stop during the first held policy");
    Require(std::abs(reached.final_time - 0.02) < 1e-12,
            "goal waited for another policy or settling interval");
    Require((reached.final_state.array() == baseline.states.at(2).array()).all(),
            "goal stop advanced past the reached state");
    CheckBoundaryStates(reached, baseline);
    Require(reached.records.back().reason == "goal_reached",
            "successful terminal was not marked");
    for (const Eigen::Vector3d& goal : {Eigen::Vector3d(0.05, 0, 0),
                                     Eigen::Vector3d(0, 0, 0.1001)}) {
      const auto missed = Simulate(true, -1, goal);
      Require(!missed.goal_caught && missed.final_time == baseline.final_time,
              "success must satisfy both strict goal tolerances");
    }
    Require(Simulate(true, -1, Eigen::Vector3d(0, 0, 2 * std::acos(-1.0))).goal_caught,
            "goal yaw error was not wrapped");
    std::cout << "PASS goal_stops_at_first_physical_step_and_preserves_reached_state" << std::endl;
    CheckCommandSenderCompatibility();
    std::cout << "PASS default_sender_ports_tags_and_efforts_unchanged" << std::endl;
    std::cout << "ALL_PHYSICAL_BOUNDARY_TESTS_PASSED" << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << std::endl;
    return 1;
  }
}
