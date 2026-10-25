#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace dairlib {
namespace systems {

// The existing center/disc distance convention; no footprint or contact terms.
struct DiscObstacle {
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.0};
  double weight{0.0};
  double sigma{1.0};
};

struct InnerObstacleCostValue {
  bool valid{false};
  std::string reason;
  double value{0.0};
};

struct InnerObstacleQuadratic {
  bool valid{false};
  std::string reason;
  double value{0.0};
  Eigen::Vector2d g_p{Eigen::Vector2d::Zero()};
  // PSD distance-linearization surrogate, NOT the exact nonlinear Hessian.
  Eigen::Matrix2d K_p{Eigen::Matrix2d::Zero()};
  // Add these to a binding using 0.5*x.transpose()*Q*x + b.dot(x) + c.
  Eigen::MatrixXd delta_Q;
  Eigen::VectorXd delta_b;
  double delta_c{0.0};
};

struct InnerObstacleTrajectoryQuadratic {
  bool valid{false};
  std::string reason;
  // Indexed like the input trajectory, 0..N. Knot 0 is explicitly all zero.
  std::vector<InnerObstacleQuadratic> knots;
  double value{0.0};
};

// Exact beta-weighted nonlinear scalar value. A disc center is valid for value
// evaluation even though its radial direction cannot define a local quadratic.
InnerObstacleCostValue EvaluateInnerObstacleCost(
    const Eigen::VectorXd& x, const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta);

// All inputs are checked, including inactive obstacle parameters. Active
// singular normals, nonfinite arithmetic, and overflow invalidate the complete
// addition; callers must use their explicit nominal fallback. Zero beta/weight
// avoids computing a normal. No clipping or invented escape direction is used.
InnerObstacleQuadratic MakeInnerObstacleQuadratic(
    const Eigen::VectorXd& x_bar, const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta);

// Builds additions at knots 1..N, including the terminal state. An invalid knot
// invalidates the entire trajectory and returns no partially usable additions.
InnerObstacleTrajectoryQuadratic MakeInnerObstacleTrajectoryQuadratic(
    const std::vector<Eigen::VectorXd>& x_bar,
    const std::array<int, 2>& xy_indices,
    const std::vector<DiscObstacle>& obstacles, double beta);

// Checks finite coefficients, symmetry, and eigenvalues before a caller marks
// an updated quadratic binding convex. Tolerance is relative to max |Q_ij|.
bool IsFiniteSymmetricPsd(const Eigen::MatrixXd& Q,
                          double relative_tolerance = 1e-12);

}  // namespace systems
}  // namespace dairlib
