#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "c3/core/c3.h"
#include "systems/controllers/inner_obstacle_cost.h"

namespace dairlib::systems {

// The factory must construct a fresh solver from the original candidate snapshot,
// including its bounds and solver options. Never copy a solved C3 instance.
using InnerObstacleCandidateFactory = std::function<std::shared_ptr<c3::C3>()>;

struct InnerObstacleTrialResult {
  std::shared_ptr<c3::C3> selected;
  bool trial_attempted{false};
  bool trial_selected{false};
  std::string fallback_reason;
  c3::C3::SolveReport trial_report;
  double construction_seconds{0};
  double solve_seconds{0};
  std::vector<Eigen::VectorXd> nominal_qp_states;
  std::vector<Eigen::VectorXd> trial_qp_states;
  std::vector<InnerObstacleQuadratic> quadratics;
};

// Production gate: none, beta=0, or no obstacles must not construct a trial.
// The explicit Run function also permits beta=0 for the budget-matched test
// harness. Production callers must use this gate; there is no test-mode env var.
bool ShouldRunInnerObstacleTrial(bool enabled, double beta,
                                 std::size_t obstacle_count);

// Retains nominal untouched on all trial failure paths. Selected owns its solver
// for the original ranking/buffer/execution consumers. Costs on a successful
// trial are restored before returning; the optimized result remains stored.
InnerObstacleTrialResult RunInnerObstacleTrial(
    const std::shared_ptr<c3::C3>& nominal,
    const InnerObstacleCandidateFactory& make_candidate,
    const Eigen::VectorXd& initial_state,
    const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta);

}  // namespace dairlib::systems
