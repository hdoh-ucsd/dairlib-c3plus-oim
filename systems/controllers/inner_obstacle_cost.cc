#include "systems/controllers/inner_obstacle_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Eigenvalues>

namespace dairlib {
namespace systems {
namespace {

std::string ValidateInputs(const Eigen::VectorXd& x,
                           const std::array<int, 2>& xy_indices,
                           const std::vector<DiscObstacle>& obstacles,
                           double beta) {
  if (!std::isfinite(beta) || beta < 0.0) return "invalid inner multiplier";
  if (x.size() < 2 || xy_indices[0] < 0 || xy_indices[1] < 0 ||
      xy_indices[0] >= x.size() || xy_indices[1] >= x.size() ||
      xy_indices[0] == xy_indices[1]) {
    return "invalid state dimension or XY selector";
  }
  if (!x.allFinite()) return "nonfinite nominal state";
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obstacle = obstacles[i];
    if (!obstacle.center.allFinite() || !std::isfinite(obstacle.radius) ||
        obstacle.radius < 0.0 || !std::isfinite(obstacle.weight) ||
        obstacle.weight < 0.0 || !std::isfinite(obstacle.sigma) ||
        obstacle.sigma <= 0.0) {
      return "invalid disc parameters at obstacle " + std::to_string(i);
    }
  }
  return {};
}

InnerObstacleQuadratic ZeroQuadratic(Eigen::Index nx) {
  InnerObstacleQuadratic result;
  result.delta_Q = Eigen::MatrixXd::Zero(nx, nx);
  result.delta_b = Eigen::VectorXd::Zero(nx);
  return result;
}

InnerObstacleQuadratic InvalidQuadratic(Eigen::Index nx,
                                        const std::string& reason) {
  auto result = ZeroQuadratic(nx);
  result.reason = reason;
  return result;
}

// Wider intermediates avoid spurious overflow in beta*weight or sigma*sigma.
// The public coefficients must still be representable as finite doubles.
long double Potential(double distance, const DiscObstacle& obstacle,
                      double beta) {
  return static_cast<long double>(beta) * obstacle.weight *
         std::exp(-(static_cast<long double>(distance) - obstacle.radius) /
                  obstacle.sigma);
}

bool FiniteDouble(long double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<double>::max();
}

}  // namespace

bool IsFiniteSymmetricPsd(const Eigen::MatrixXd& Q,
                          double relative_tolerance) {
  if (Q.rows() != Q.cols() || Q.rows() == 0 || !Q.allFinite() ||
      !std::isfinite(relative_tolerance) || relative_tolerance < 0.0) {
    return false;
  }
  const double scale = Q.cwiseAbs().maxCoeff();
  if (scale == 0.0) return true;
  // Normalizing also avoids overflow in eigenvalue calculations and tolerances.
  const Eigen::MatrixXd normalized = Q / scale;
  if ((normalized - normalized.transpose()).cwiseAbs().maxCoeff() >
      relative_tolerance) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(
      normalized, Eigen::EigenvaluesOnly);
  return eigensolver.info() == Eigen::Success &&
         eigensolver.eigenvalues().allFinite() &&
         eigensolver.eigenvalues().minCoeff() >= -relative_tolerance;
}

InnerObstacleCostValue EvaluateInnerObstacleCost(
    const Eigen::VectorXd& x, const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta) {
  InnerObstacleCostValue result;
  result.reason = ValidateInputs(x, xy_indices, obstacles, beta);
  if (!result.reason.empty()) return result;
  const Eigen::Vector2d p(x[xy_indices[0]], x[xy_indices[1]]);
  long double total = 0.0L;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obstacle = obstacles[i];
    if (beta == 0.0 || obstacle.weight == 0.0) continue;
    const Eigen::Vector2d r = p - obstacle.center;
    const double distance = std::hypot(r.x(), r.y());
    const long double alpha = Potential(distance, obstacle, beta);
    if (!r.allFinite() || !std::isfinite(distance) || !FiniteDouble(alpha)) {
      result.reason = "nonfinite potential at obstacle " + std::to_string(i);
      return result;
    }
    total += alpha;
    if (!FiniteDouble(total)) {
      result.reason = "overflow in summed obstacle value";
      return result;
    }
  }
  result.value = static_cast<double>(total);
  result.valid = true;
  return result;
}

InnerObstacleQuadratic MakeInnerObstacleQuadratic(
    const Eigen::VectorXd& x_bar, const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta) {
  const auto reason = ValidateInputs(x_bar, xy_indices, obstacles, beta);
  if (!reason.empty()) return InvalidQuadratic(x_bar.size(), reason);
  auto result = ZeroQuadratic(x_bar.size());
  const Eigen::Vector2d p(x_bar[xy_indices[0]], x_bar[xy_indices[1]]);
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obstacle = obstacles[i];
    if (beta == 0.0 || obstacle.weight == 0.0) continue;
    const auto invalid = [&](const std::string& detail) {
      return InvalidQuadratic(x_bar.size(), detail + " at obstacle " +
                                                 std::to_string(i));
    };
    const Eigen::Vector2d r = p - obstacle.center;
    const double distance = std::hypot(r.x(), r.y());
    if (!r.allFinite() || !std::isfinite(distance)) {
      return invalid("nonfinite center distance");
    }
    if (distance == 0.0) return invalid("singular center distance");
    const Eigen::Vector2d n = r / distance;
    const long double alpha = Potential(distance, obstacle, beta);
    const long double gradient_scale = alpha / obstacle.sigma;
    const long double curvature_scale = gradient_scale / obstacle.sigma;
    if (!FiniteDouble(alpha) || !FiniteDouble(gradient_scale) ||
        !FiniteDouble(curvature_scale)) {
      return invalid("nonfinite potential or derivative");
    }
    result.value += static_cast<double>(alpha);
    result.g_p -= static_cast<double>(gradient_scale) * n;
    result.K_p += static_cast<double>(curvature_scale) * (n * n.transpose());
    if (!std::isfinite(result.value) || !result.g_p.allFinite() ||
        !result.K_p.allFinite()) {
      return invalid("overflow in summed obstacle coefficients");
    }
  }
  if (!IsFiniteSymmetricPsd(result.K_p)) {
    return InvalidQuadratic(x_bar.size(), "invalid surrogate curvature");
  }
  const Eigen::Vector2d b_p = result.g_p - result.K_p * p;
  for (int i = 0; i < 2; ++i) {
    result.delta_b[xy_indices[i]] = b_p[i];
    for (int j = 0; j < 2; ++j) {
      result.delta_Q(xy_indices[i], xy_indices[j]) = result.K_p(i, j);
    }
  }
  result.delta_c = result.value - result.g_p.dot(p) +
                   0.5 * p.dot(result.K_p * p);
  if (!result.delta_b.allFinite() || !std::isfinite(result.delta_c)) {
    return InvalidQuadratic(x_bar.size(), "nonfinite expanded coefficients");
  }
  result.valid = true;
  return result;
}

InnerObstacleTrajectoryQuadratic MakeInnerObstacleTrajectoryQuadratic(
    const std::vector<Eigen::VectorXd>& x_bar,
    const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta) {
  InnerObstacleTrajectoryQuadratic result;
  if (x_bar.empty()) {
    result.reason = "empty state trajectory";
    return result;
  }
  const auto reason = ValidateInputs(x_bar.front(), xy_indices, obstacles, beta);
  if (!reason.empty()) {
    result.reason = "knot 0: " + reason;
    return result;
  }
  auto initial = ZeroQuadratic(x_bar.front().size());
  initial.valid = true;
  result.knots.push_back(std::move(initial));
  for (size_t k = 1; k < x_bar.size(); ++k) {
    auto quadratic = MakeInnerObstacleQuadratic(x_bar[k], xy_indices,
                                                obstacles, beta);
    if (x_bar[k].size() != x_bar.front().size() || !quadratic.valid ||
        !std::isfinite(result.value + quadratic.value)) {
      InnerObstacleTrajectoryQuadratic invalid;
      invalid.reason = "knot " + std::to_string(k) + ": " +
          (x_bar[k].size() != x_bar.front().size()
               ? "inconsistent state dimension"
               : (!quadratic.valid ? quadratic.reason : "overflow in total"));
      return invalid;
    }
    result.value += quadratic.value;
    result.knots.push_back(std::move(quadratic));
  }
  result.valid = true;
  return result;
}

}  // namespace systems
}  // namespace dairlib
