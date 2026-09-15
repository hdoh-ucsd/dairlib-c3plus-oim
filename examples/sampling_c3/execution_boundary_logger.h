#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dairlib/lcmt_robot_input.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

// Thrown before applying the pending plant update. Only an explicitly requested
// execution budget enables this termination condition.
class ExecutionStepBudgetReached final : public std::exception {
 public:
  const char* what() const noexcept override {
    return "execution step budget reached";
  }
};

class ExecutionGoalReached final : public std::exception {
 public:
  const char* what() const noexcept override { return "execution goal reached"; }
};

// Observes the same pre-update Context as the discrete plant. In particular,
// this does not use asynchronous object-state messages or change plant inputs.
// Register this system after the plant and other diagram systems so a failed
// plant calculation cannot produce a spurious boundary record.
class ExecutionBoundaryLogger final
    : public drake::systems::LeafSystem<double> {
 public:
  ExecutionBoundaryLogger(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::ModelInstanceIndex robot,
      std::vector<drake::multibody::ModelInstanceIndex> objects,
      std::vector<std::string> object_channels, double actuator_delay,
      int64_t step_budget, std::string stop_file,
      std::optional<Eigen::Vector3d> goal = std::nullopt)
      : plant_(plant), robot_(robot), objects_(std::move(objects)),
        object_channels_(std::move(object_channels)),
        actuator_delay_(actuator_delay), step_budget_(step_budget),
        stop_file_(std::move(stop_file)), goal_(std::move(goal)) {
    if (goal_ && (!goal_->allFinite() || objects_.size() != 1)) {
      throw std::runtime_error("execution_goal requires one object and finite x,y,yaw");
    }
    this->set_name("execution_boundary_logger");
    state_port_ = this->DeclareVectorInputPort(
                         "physical_state", plant.num_multibody_states())
                      .get_index();
    command_port_ = this->DeclareAbstractInputPort(
                           "applied_command",
                           drake::Value<dairlib::lcmt_robot_input>{})
                        .get_index();
    available_ = plant.is_discrete() && plant.has_sampled_output_ports() &&
                 actuator_delay == 0.0;
    WriteHeader(available_ ? "" : "requires_discrete_sampled_plant_and_zero_actuator_delay");
    if (available_) {
      // The plant's default sampled-output mode uses this exact event phase,
      // period and offset. All unrestricted calculations read the OLD Context;
      // Drake applies their pending state changes only after they all succeed.
      this->DeclarePeriodicUnrestrictedUpdateEvent(
          plant.time_step(), 0.0, &ExecutionBoundaryLogger::Observe);
    } else if (step_budget_ > 0 || goal_) {
      throw std::runtime_error(
          "execution stopping requires exact physical execution alignment");
    }
  }

  const drake::systems::InputPort<double>& state_input() const {
    return get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& command_input() const {
    return get_input_port(command_port_);
  }
  int64_t n_steps_executed() const { return n_steps_; }

  // Called from Simulator's monitor after an existing completed step, or from
  // a goal/budget catch with the unchanged pre-update Context. No extra physics
  // step, terminal hold, or settling interval is introduced.
  void LogTerminal(const drake::systems::Context<double>& context,
                   const std::string& reason, int signal_number = 0) const {
    const auto now = Clock::now();
    if (!available_ || terminal_written_) return;
    if (n_steps_ == 0) first_boundary_ = now;
    const double wall_time = Seconds(now);
    if (n_steps_ > 0 &&
        (context.get_time() <= last_sim_time_ || wall_time <= last_wall_time_)) {
      Invalidate("terminal_not_after_last_execution_boundary");
      return;
    }
    if (!WriteState("C3_EXECUTION_TERMINAL", context, n_steps_, last_plan_,
                    wall_time, reason, signal_number)) return;
    terminal_written_ = true;
    if ((reason == "step_budget" || reason == "goal_reached") && !stop_file_.empty()) {
      const std::string temporary = stop_file_ + ".tmp";
      std::ofstream marker(temporary);
      if (!marker) throw std::runtime_error("cannot write execution stop file");
      marker << std::setprecision(17) << "{\"termination_reason\":" << std::quoted(reason)
             << ",\"sim_time\":" << context.get_time()
             << ",\"n_steps_executed\":" << n_steps_;
      if (reason == "goal_reached") {
        const auto& state = state_input().Eval(context);
        const Eigen::VectorXd q = state.head(plant_.num_positions());
        const Eigen::VectorXd v = state.tail(plant_.num_velocities());
        marker << ",\"snapshot\":{\"source\":\"native_goal_terminal\",\"sim_time\":"
               << context.get_time() << ",\"robot_q\":";
        WriteVector(marker, plant_.GetPositionsFromArray(robot_, q));
        marker << ",\"robot_v\":";
        WriteVector(marker, plant_.GetVelocitiesFromArray(robot_, v));
        marker << ",\"objects\":{";
        for (size_t i = 0; i < objects_.size(); ++i) {
          if (i) marker << ',';
          marker << std::quoted(object_channels_[i]) << ':';
          WriteVector(marker, plant_.GetPositionsFromArray(objects_[i], q));
        }
        marker << "}}";
      }
      marker << "}\n";
      marker.flush();
      if (!marker) throw std::runtime_error("cannot flush execution stop file");
      marker.close();
      if (std::rename(temporary.c_str(), stop_file_.c_str()) != 0) {
        throw std::runtime_error("cannot publish execution stop file");
      }
    }
  }

 private:
  using Clock = std::chrono::steady_clock;

  double Seconds(Clock::time_point now) const {
    return std::chrono::duration<double>(now - first_boundary_).count();
  }

  void WriteHeader(const std::string& reason) const {
    std::ostringstream record;
    record.imbue(std::locale::classic());
    record << "[C3_EXECUTION_LOGGING] {\"alignment\":"
           << (available_ ? "\"physical_policy_boundaries_v1\"" : "null")
           << ",\"step_budget\":";
    if (step_budget_ > 0) record << step_budget_;
    else record << "null";
    record << ",\"actuator_delay\":" << std::setprecision(17)
           << actuator_delay_ << ",\"object_channels\":[";
    for (size_t i = 0; i < object_channels_.size(); ++i) {
      if (i) record << ',';
      record << std::quoted(object_channels_[i]);
    }
    record << ']';
    if (goal_) {
      record << ",\"goal\":[" << (*goal_)[0] << ',' << (*goal_)[1] << ',' << (*goal_)[2]
             << "],\"goal_pos_tol\":0.05,\"goal_ang_tol\":0.1";
    }
    if (!reason.empty()) record << ",\"reason\":" << std::quoted(reason);
    record << '}';
    std::cout << record.str() << std::endl;
  }

  void Invalidate(const std::string& reason) const {
    available_ = false;
    WriteHeader(reason);
    if (step_budget_ > 0 || goal_) {
      throw std::runtime_error("cannot enforce execution stopping: " + reason);
    }
  }

  static void WriteVector(std::ostream& stream, const Eigen::VectorXd& values) {
    stream << '[';
    for (int i = 0; i < values.size(); ++i) {
      if (i) stream << ',';
      stream << values[i];
    }
    stream << ']';
  }

  bool WriteState(const char* prefix,
                  const drake::systems::Context<double>& context,
                  int64_t boundary_step, int64_t plan_utime, double wall_time,
                  const std::string& reason = "", int signal_number = 0) const {
    const auto& state = state_input().Eval(context);
    if (!state.allFinite()) {
      Invalidate("nonfinite_physical_state");
      return false;
    }
    const Eigen::VectorXd q = state.head(plant_.num_positions());
    const Eigen::VectorXd v = state.tail(plant_.num_velocities());
    std::ostringstream record;
    record.imbue(std::locale::classic());
    record << std::setprecision(17) << '[' << prefix << "] {\"boundary_step\":"
           << boundary_step << ",\"plan_utime\":" << plan_utime
           << ",\"sim_time\":" << context.get_time()
           << ",\"wall_time\":" << wall_time << ",\"objects\":[";
    for (size_t i = 0; i < objects_.size(); ++i) {
      if (i) record << ',';
      record << "{\"q\":";
      WriteVector(record, plant_.GetPositionsFromArray(objects_[i], q));
      record << ",\"v\":";
      WriteVector(record, plant_.GetVelocitiesFromArray(objects_[i], v));
      record << '}';
    }
    record << "],\"robot_q\":";
    WriteVector(record, plant_.GetPositionsFromArray(robot_, q));
    record << ",\"robot_v\":";
    WriteVector(record, plant_.GetVelocitiesFromArray(robot_, v));
    if (!reason.empty()) record << ",\"reason\":" << std::quoted(reason);
    if (signal_number) record << ",\"signal\":" << signal_number;
    record << '}';
    std::cout << record.str() << std::endl;
    return true;
  }

  drake::systems::EventStatus Observe(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>*) const {
    const auto now = Clock::now();
    if (!available_) return drake::systems::EventStatus::DidNothing();
    // Check every physical simulation step, including while the current policy
    // is held. Stop before applying another update and retain this exact state.
    // The first applied interval must exist for an N+1 execution trajectory.
    if (goal_ && n_steps_ > 0 && context.get_time() > last_sim_time_) {
      const auto& state = state_input().Eval(context);
      const Eigen::VectorXd q = plant_.GetPositionsFromArray(
          objects_[0], state.head(plant_.num_positions()));
      if (q.size() == 7 && q.allFinite()) {
        Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
        if (rotation.norm() > 0) {
          rotation.normalize();
          const auto R = rotation.toRotationMatrix();
          const double yaw = std::atan2(R(1, 0), R(0, 0));
          const double angle_error = std::abs(std::atan2(
              std::sin(yaw - (*goal_)[2]), std::cos(yaw - (*goal_)[2])));
          if (std::hypot(q[4] - (*goal_)[0], q[5] - (*goal_)[1]) < 0.05 &&
              angle_error < 0.1) {
            throw ExecutionGoalReached();
          }
        }
      }
    }
    const auto& command = command_input().Eval<dairlib::lcmt_robot_input>(context);
    const int64_t source = command.source_plan_utime;
    if (source == last_plan_) return drake::systems::EventStatus::DidNothing();
    if (source <= 0 || (n_steps_ > 0 && source < last_plan_)) {
      if (n_steps_ > 0) Invalidate("untracked_or_nonmonotonic_applied_policy");
      return drake::systems::EventStatus::DidNothing();
    }
    if (step_budget_ > 0 && n_steps_ >= step_budget_) {
      throw ExecutionStepBudgetReached();
    }
    if (n_steps_ == 0) first_boundary_ = now;
    const double wall_time = Seconds(now);
    if (n_steps_ > 0 &&
        (context.get_time() <= last_sim_time_ || wall_time <= last_wall_time_)) {
      Invalidate("nonincreasing_execution_boundary_time");
      return drake::systems::EventStatus::DidNothing();
    }
    if (WriteState("C3_EXECUTION_BOUNDARY", context, n_steps_, source,
                   wall_time)) {
      ++n_steps_;
      last_plan_ = source;
      last_sim_time_ = context.get_time();
      last_wall_time_ = wall_time;
    }
    return drake::systems::EventStatus::Succeeded();
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::multibody::ModelInstanceIndex robot_;
  std::vector<drake::multibody::ModelInstanceIndex> objects_;
  std::vector<std::string> object_channels_;
  double actuator_delay_;
  int64_t step_budget_;
  std::string stop_file_;
  std::optional<Eigen::Vector3d> goal_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex command_port_;
  mutable bool available_ = false;
  mutable bool terminal_written_ = false;
  mutable int64_t n_steps_ = 0;
  mutable int64_t last_plan_ = 0;
  mutable double last_sim_time_ = 0;
  mutable double last_wall_time_ = 0;
  mutable Clock::time_point first_boundary_;
};
}  // namespace dairlib
