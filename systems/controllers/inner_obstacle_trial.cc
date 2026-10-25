#include "systems/controllers/inner_obstacle_trial.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace dairlib::systems {
namespace {
using Clock = std::chrono::steady_clock;

struct Coefficients {
  Eigen::MatrixXd Q;
  Eigen::VectorXd b;
  double c;
  bool convex;
};

// Own numeric originals, never Eigen aliases. All updates are computed from
// these originals. The guard covers partial application, exceptions and return.
class ScopedStateCosts {
 public:
  explicit ScopedStateCosts(const std::vector<drake::solvers::QuadraticCost*>& costs)
      : costs_(costs) {
    std::unordered_set<const void*> seen;
    for (const auto* cost : costs_) {
      if (!cost || !seen.insert(cost).second) {
        throw std::invalid_argument("Aliased or null state-cost evaluator");
      }
      originals_.push_back({cost->Q(), cost->b(), cost->c(), cost->is_convex()});
    }
  }
  ~ScopedStateCosts() {
    if (!modified_) return;
    for (std::size_t k = 1; k < costs_.size(); ++k) {
      const auto& original = originals_[k];
      costs_[k]->UpdateCoefficients(original.Q, original.b, original.c,
                                   original.convex);
    }
  }
  void Apply(const std::vector<InnerObstacleQuadratic>& additions) {
    if (additions.size() != costs_.size()) {
      throw std::invalid_argument("State-cost knot count does not match QP trajectory");
    }
    std::vector<Coefficients> updated;
    for (std::size_t k = 0; k < costs_.size(); ++k) {
      const auto& base = originals_[k];
      const auto& add = additions[k];
      if (!add.valid || add.delta_Q.rows() != base.Q.rows() ||
          add.delta_Q.cols() != base.Q.cols() || add.delta_b.size() != base.b.size()) {
        throw std::invalid_argument("Invalid state-cost addition dimensions");
      }
      Coefficients next{base.Q + add.delta_Q, base.b + add.delta_b,
                        base.c + add.delta_c, true};
      if (!IsFiniteSymmetricPsd(next.Q) || !next.b.allFinite() ||
          !std::isfinite(next.c)) {
        throw std::invalid_argument("Updated state cost is nonfinite or not PSD");
      }
      updated.push_back(std::move(next));
    }
    modified_ = true;
    // Knot zero is fixed and never touched, even by an identical rewrite.
    for (std::size_t k = 1; k < costs_.size(); ++k) {
      const auto& next = updated[k];
      costs_[k]->UpdateCoefficients(next.Q, next.b, next.c, true);
    }
  }
 private:
  std::vector<drake::solvers::QuadraticCost*> costs_;
  std::vector<Coefficients> originals_;
  bool modified_{false};
};
}  // namespace

bool ShouldRunInnerObstacleTrial(bool enabled, double beta,
                                 std::size_t obstacle_count) {
  if (!std::isfinite(beta) || beta < 0) {
    throw std::invalid_argument("Inner multiplier must be finite and nonnegative");
  }
  return enabled && beta > 0 && obstacle_count != 0;
}

InnerObstacleTrialResult RunInnerObstacleTrial(
    const std::shared_ptr<c3::C3>& nominal,
    const InnerObstacleCandidateFactory& make_candidate,
    const Eigen::VectorXd& initial_state,
    const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta) {
  InnerObstacleTrialResult result;
  result.selected = nominal;
  if (!nominal || !nominal->GetLastSolveReport().completed) {
    result.fallback_reason = "Nominal invocation did not complete all finite QP stages";
    return result;  // Keep original nominal failure policy downstream.
  }
  try {
    result.nominal_qp_states = nominal->GetQPStateSolution();
    auto approximation = MakeInnerObstacleTrajectoryQuadratic(
        result.nominal_qp_states, xy_indices, obstacles, beta);
    if (!approximation.valid) {
      result.fallback_reason = "Invalid obstacle approximation: " + approximation.reason;
      return result;
    }
    result.quadratics = std::move(approximation.knots);
    const auto begin = Clock::now();
    auto trial = make_candidate();
    result.construction_seconds = std::chrono::duration<double>(Clock::now() - begin).count();
    if (!trial || trial.get() == nominal.get()) {
      throw std::invalid_argument("Trial factory did not return an independent solver");
    }
    const auto& original_costs = nominal->GetTargetCost();
    const auto& trial_costs = trial->GetTargetCost();
    if (trial_costs.size() != original_costs.size()) {
      throw std::invalid_argument("Trial state-cost count differs from nominal");
    }
    for (std::size_t k = 0; k < trial_costs.size(); ++k) {
      for (const auto* original : original_costs) {
        if (trial_costs[k] == original) {
          throw std::invalid_argument("Trial shares a nominal cost evaluator");
        }
      }
      if (trial_costs[k]->Q().rows() != original_costs[k]->Q().rows() ||
          trial_costs[k]->b().size() != original_costs[k]->b().size() ||
          !trial_costs[k]->Q().isApprox(original_costs[k]->Q(), 0.0) ||
          !trial_costs[k]->b().isApprox(original_costs[k]->b(), 0.0) ||
          trial_costs[k]->c() != original_costs[k]->c()) {
        throw std::invalid_argument("Trial original state costs differ from nominal");
      }
    }
    {
      ScopedStateCosts guard(trial_costs);
      guard.Apply(result.quadratics);
      result.trial_attempted = true;
      const auto solve_begin = Clock::now();
      result.trial_report = trial->SolveChecked(initial_state);
      result.solve_seconds = std::chrono::duration<double>(Clock::now() - solve_begin).count();
      if (!result.trial_report.completed) {
        result.fallback_reason = "Trial QP execution failed: " + result.trial_report.failure_reason;
        return result;
      }
      result.trial_qp_states = trial->GetQPStateSolution();
    }
    result.selected = std::move(trial);
    result.trial_selected = true;
  } catch (const std::exception& error) {
    result.fallback_reason = std::string("Trial exception: ") + error.what();
  } catch (...) {
    result.fallback_reason = "Trial exception: non-standard exception";
  }
  return result;
}
}  // namespace dairlib::systems
