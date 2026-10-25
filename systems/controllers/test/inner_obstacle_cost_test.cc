#include "systems/controllers/inner_obstacle_cost.h"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace dairlib {
namespace systems {
namespace {

constexpr double kTolerance = 1e-11;
const std::array<int, 2> kXy{{3, 1}};

Eigen::VectorXd State(const Eigen::Vector2d& p) {
  Eigen::VectorXd x(5);
  x << 7.0, p.y(), -4.0, p.x(), 9.0;
  return x;
}

DiscObstacle Obstacle(const Eigen::Vector2d& center = {0.2, -0.1},
                      double radius = 0.3, double weight = 2.5,
                      double sigma = 0.7) {
  DiscObstacle obstacle;
  obstacle.center = center;
  obstacle.radius = radius;
  obstacle.weight = weight;
  obstacle.sigma = sigma;
  return obstacle;
}

double ExpandedValue(const InnerObstacleQuadratic& cost,
                     const Eigen::VectorXd& x) {
  return 0.5 * x.dot(cost.delta_Q * x) + cost.delta_b.dot(x) + cost.delta_c;
}

TEST(InnerObstacleCostTest, ExactScalarAndSpecifiedRadialDerivatives) {
  const Eigen::Vector2d p(1.4, 0.8);
  const auto obstacle = Obstacle();
  const double beta = 1.7;
  const double distance = (p - obstacle.center).norm() - obstacle.radius;
  const double alpha = beta * obstacle.weight *
                       std::exp(-distance / obstacle.sigma);
  const Eigen::Vector2d n = (p - obstacle.center).normalized();
  const auto scalar = EvaluateInnerObstacleCost(State(p), kXy, {obstacle}, beta);
  const auto cost = MakeInnerObstacleQuadratic(State(p), kXy, {obstacle}, beta);
  ASSERT_TRUE(scalar.valid) << scalar.reason;
  ASSERT_TRUE(cost.valid) << cost.reason;
  EXPECT_NEAR(scalar.value, alpha, kTolerance);
  EXPECT_NEAR(cost.value, alpha, kTolerance);
  EXPECT_TRUE(cost.g_p.isApprox(-alpha / obstacle.sigma * n, kTolerance));
  EXPECT_TRUE(cost.K_p.isApprox(alpha / std::pow(obstacle.sigma, 2) *
                                n * n.transpose(), kTolerance));
}

TEST(InnerObstacleCostTest, GradientMatchesNonlinearFiniteDifferences) {
  const auto x = State({1.4, 0.8});
  const std::vector<DiscObstacle> obstacles{
      Obstacle(), Obstacle({-0.4, 1.0}, 0.2, 0.8, 0.9)};
  const auto cost = MakeInnerObstacleQuadratic(x, kXy, obstacles, 1.7);
  ASSERT_TRUE(cost.valid) << cost.reason;
  const double step = 1e-6;
  for (int axis = 0; axis < 2; ++axis) {
    auto plus = x;
    auto minus = x;
    plus[kXy[axis]] += step;
    minus[kXy[axis]] -= step;
    const auto f_plus = EvaluateInnerObstacleCost(plus, kXy, obstacles, 1.7);
    const auto f_minus = EvaluateInnerObstacleCost(minus, kXy, obstacles, 1.7);
    ASSERT_TRUE(f_plus.valid);
    ASSERT_TRUE(f_minus.valid);
    EXPECT_NEAR(cost.g_p[axis], (f_plus.value - f_minus.value) / (2 * step),
                1e-8);
  }
}

TEST(InnerObstacleCostTest, SurrogateIsSymmetricPositiveSemidefinite) {
  const auto cost = MakeInnerObstacleQuadratic(
      State({1.4, 0.8}), kXy,
      {Obstacle(), Obstacle({-0.4, 1.0}, 0.2, 0.8, 0.9)}, 1.7);
  ASSERT_TRUE(cost.valid) << cost.reason;
  EXPECT_TRUE(cost.K_p.isApprox(cost.K_p.transpose(), kTolerance));
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(cost.K_p)
                .eigenvalues().minCoeff(), -kTolerance);
  EXPECT_TRUE(IsFiniteSymmetricPsd(cost.delta_Q));
  // Unselected physical-state entries are untouched, with a noncontiguous XY
  // selector ensuring that the helper makes no assumptions about model layout.
  for (int i : {0, 2, 4}) {
    EXPECT_TRUE(cost.delta_Q.row(i).isZero(0.0));
    EXPECT_TRUE(cost.delta_Q.col(i).isZero(0.0));
    EXPECT_EQ(cost.delta_b[i], 0.0);
  }
}

TEST(InnerObstacleCostTest, NominalValueGradientAndCoefficientExpansion) {
  const Eigen::Vector2d p_bar(1.4, 0.8);
  const auto x_bar = State(p_bar);
  const auto cost = MakeInnerObstacleQuadratic(x_bar, kXy, {Obstacle()}, 1.7);
  ASSERT_TRUE(cost.valid) << cost.reason;
  EXPECT_NEAR(ExpandedValue(cost, x_bar), cost.value, kTolerance);
  const Eigen::VectorXd full_gradient = cost.delta_Q * x_bar + cost.delta_b;
  EXPECT_NEAR(full_gradient[kXy[0]], cost.g_p.x(), kTolerance);
  EXPECT_NEAR(full_gradient[kXy[1]], cost.g_p.y(), kTolerance);
  for (const Eigen::Vector2d& offset :
       std::vector<Eigen::Vector2d>{{0.0, 0.0}, {0.2, -0.4}, {-0.7, 0.3}}) {
    auto x = State(p_bar + offset);
    x[0] = -213.0;
    x[2] = 80.0;
    x[4] = 0.0;
    const double local = cost.value + cost.g_p.dot(offset) +
                         0.5 * offset.dot(cost.K_p * offset);
    EXPECT_NEAR(ExpandedValue(cost, x), local, kTolerance);
  }
}

TEST(InnerObstacleCostTest, ZeroMultiplierAndWeightsNeedNoDirection) {
  const auto x = State({0.2, -0.1});  // Exactly at the disc center.
  for (const auto& cost : {
           MakeInnerObstacleQuadratic(x, kXy, {Obstacle()}, 0.0),
           MakeInnerObstacleQuadratic(x, kXy, {Obstacle({0.2, -0.1}, 0.3, 0.0)},
                                      1.0),
           MakeInnerObstacleQuadratic(x, kXy, {}, 1.0)}) {
    ASSERT_TRUE(cost.valid) << cost.reason;
    EXPECT_EQ(cost.value, 0.0);
    EXPECT_TRUE(cost.g_p.isZero(0.0));
    EXPECT_TRUE(cost.K_p.isZero(0.0));
    EXPECT_TRUE(cost.delta_Q.isZero(0.0));
    EXPECT_TRUE(cost.delta_b.isZero(0.0));
    EXPECT_EQ(cost.delta_c, 0.0);
  }
}

TEST(InnerObstacleCostTest, ObstaclesAreAdditive) {
  const auto x = State({1.4, 0.8});
  const auto a = Obstacle();
  const auto b = Obstacle({-0.4, 1.0}, 0.2, 0.8, 0.9);
  const auto ca = MakeInnerObstacleQuadratic(x, kXy, {a}, 1.7);
  const auto cb = MakeInnerObstacleQuadratic(x, kXy, {b}, 1.7);
  const auto both = MakeInnerObstacleQuadratic(x, kXy, {a, b}, 1.7);
  ASSERT_TRUE(ca.valid);
  ASSERT_TRUE(cb.valid);
  ASSERT_TRUE(both.valid);
  EXPECT_NEAR(both.value, ca.value + cb.value, kTolerance);
  EXPECT_TRUE(both.g_p.isApprox(ca.g_p + cb.g_p, kTolerance));
  EXPECT_TRUE(both.K_p.isApprox(ca.K_p + cb.K_p, kTolerance));
  EXPECT_TRUE(both.delta_Q.isApprox(ca.delta_Q + cb.delta_Q, kTolerance));
  EXPECT_TRUE(both.delta_b.isApprox(ca.delta_b + cb.delta_b, kTolerance));
  EXPECT_NEAR(both.delta_c, ca.delta_c + cb.delta_c, kTolerance);
}

TEST(InnerObstacleCostTest, ExcludesFixedInitialKnotAndIncludesTerminalKnot) {
  const std::vector<Eigen::VectorXd> trajectory{
      State({0.2, -0.1}),  // Singular normal would invalidate a steering knot.
      State({1.4, 0.8}), State({0.8, 0.4})};
  const auto cost = MakeInnerObstacleTrajectoryQuadratic(
      trajectory, kXy, {Obstacle()}, 1.7);
  ASSERT_TRUE(cost.valid) << cost.reason;
  ASSERT_EQ(cost.knots.size(), trajectory.size());
  EXPECT_EQ(cost.knots[0].value, 0.0);
  EXPECT_TRUE(cost.knots[0].delta_Q.isZero(0.0));
  EXPECT_TRUE(cost.knots[0].delta_b.isZero(0.0));
  EXPECT_EQ(cost.knots[0].delta_c, 0.0);
  const auto stage = EvaluateInnerObstacleCost(trajectory[1], kXy,
                                              {Obstacle()}, 1.7);
  const auto terminal = EvaluateInnerObstacleCost(trajectory[2], kXy,
                                                 {Obstacle()}, 1.7);
  EXPECT_GT(cost.knots[2].value, 0.0);
  EXPECT_NEAR(cost.knots[2].value, terminal.value, kTolerance);
  EXPECT_NEAR(cost.value, stage.value + terminal.value, kTolerance);
}

TEST(InnerObstacleCostTest, PenetrationIsFiniteAndHasOutwardDescentDirection) {
  const Eigen::Vector2d p(0.4, 0.0);
  const auto obstacle = Obstacle({0.0, 0.0}, 0.8, 2.0, 0.3);
  const auto cost = MakeInnerObstacleQuadratic(State(p), kXy, {obstacle}, 1.0);
  ASSERT_TRUE(cost.valid) << cost.reason;
  EXPECT_GT(cost.value, obstacle.weight);
  EXPECT_LT(cost.g_p.dot(p), 0.0);
  EXPECT_TRUE(IsFiniteSymmetricPsd(cost.K_p));
  EXPECT_NEAR(cost.value, 2.0 * std::exp(0.4 / 0.3), kTolerance);
}

TEST(InnerObstacleCostTest, SingularNormalInvalidatesCompleteAddition) {
  const auto x = State({0.2, -0.1});
  const std::vector<DiscObstacle> obstacles{
      Obstacle({2.0, 2.0}), Obstacle()};
  const auto a = MakeInnerObstacleQuadratic(x, kXy, obstacles, 1.0);
  const auto b = MakeInnerObstacleQuadratic(x, kXy, obstacles, 1.0);
  EXPECT_FALSE(a.valid);
  EXPECT_EQ(a.reason, b.reason);
  EXPECT_NE(a.reason.find("singular"), std::string::npos);
  EXPECT_TRUE(a.delta_Q.isZero(0.0));
  EXPECT_TRUE(a.delta_b.isZero(0.0));
  EXPECT_EQ(a.value, 0.0);
  // The exact scalar is still well defined, and remains useful for diagnostics.
  const auto scalar = EvaluateInnerObstacleCost(x, kXy, {Obstacle()}, 1.0);
  ASSERT_TRUE(scalar.valid);
  EXPECT_NEAR(scalar.value, 2.5 * std::exp(0.3 / 0.7), kTolerance);
}

TEST(InnerObstacleCostTest, RejectsMalformedAndNonfiniteInputs) {
  const auto x = State({1.4, 0.8});
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double beta : {-1.0, infinity, nan}) {
    EXPECT_FALSE(MakeInnerObstacleQuadratic(x, kXy, {Obstacle()}, beta).valid);
    EXPECT_FALSE(EvaluateInnerObstacleCost(x, kXy, {Obstacle()}, beta).valid);
  }
  for (const std::array<int, 2>& selector :
       std::vector<std::array<int, 2>>{{{-1, 1}}, {{1, 1}}, {{1, 5}}}) {
    EXPECT_FALSE(MakeInnerObstacleQuadratic(x, selector, {Obstacle()}, 1.0).valid);
  }
  EXPECT_FALSE(MakeInnerObstacleQuadratic(Eigen::VectorXd(), kXy,
                                          {Obstacle()}, 1.0).valid);
  auto nonfinite_x = x;
  nonfinite_x[0] = nan;
  EXPECT_FALSE(MakeInnerObstacleQuadratic(nonfinite_x, kXy,
                                          {Obstacle()}, 1.0).valid);
  std::vector<DiscObstacle> invalid_obstacles;
  for (double value : {-1.0, infinity, nan}) {
    invalid_obstacles.push_back(Obstacle({0.0, 0.0}, value));
    invalid_obstacles.push_back(Obstacle({0.0, 0.0}, 0.0, value));
    invalid_obstacles.push_back(Obstacle({0.0, 0.0}, 0.0, 1.0, value));
  }
  invalid_obstacles.push_back(Obstacle({0.0, 0.0}, 0.0, 1.0, 0.0));
  invalid_obstacles.push_back(Obstacle({infinity, 0.0}));
  invalid_obstacles.push_back(Obstacle({0.0, nan}));
  for (const auto& obstacle : invalid_obstacles) {
    EXPECT_FALSE(MakeInnerObstacleQuadratic(x, kXy, {obstacle}, 1.0).valid);
    EXPECT_FALSE(MakeInnerObstacleQuadratic(x, kXy, {obstacle}, 0.0).valid);
    EXPECT_FALSE(EvaluateInnerObstacleCost(x, kXy, {obstacle}, 1.0).valid);
  }
}

TEST(InnerObstacleCostTest, RejectsPotentialDerivativeAndSummationOverflow) {
  const auto explosive = Obstacle({0.0, 0.0}, 1000.0, 1.0, 1.0);
  EXPECT_FALSE(EvaluateInnerObstacleCost(State({1.0, 0.0}), kXy,
                                         {explosive}, 1.0).valid);
  EXPECT_FALSE(MakeInnerObstacleQuadratic(State({1.0, 0.0}), kXy,
                                          {explosive}, 1.0).valid);
  const auto narrow = Obstacle({0.0, 0.0}, 0.0, 1.0, 1e-308);
  EXPECT_TRUE(EvaluateInnerObstacleCost(State({5e-309, 0.0}), kXy,
                                        {narrow}, 1.0).valid);
  EXPECT_FALSE(MakeInnerObstacleQuadratic(State({5e-309, 0.0}), kXy,
                                          {narrow}, 1.0).valid);
  const auto large = Obstacle({0.0, 0.0}, 0.0, 1e308, 1.0);
  EXPECT_FALSE(EvaluateInnerObstacleCost(State({0.1, 0.0}), kXy,
                                         {large, large, large}, 1.0).valid);
  EXPECT_FALSE(MakeInnerObstacleQuadratic(State({0.1, 0.0}), kXy,
                                          {large, large, large}, 1.0).valid);
  const auto expansion = Obstacle({1e154 - 1e140, 0.0}, 0.0, 1e308, 1e140);
  EXPECT_FALSE(MakeInnerObstacleQuadratic(State({1e154, 0.0}), kXy,
                                          {expansion}, 1.0).valid);
}

TEST(InnerObstacleCostTest, FiniteExtremeScalesAvoidSpuriousIntermediateOverflow) {
  const auto distant = Obstacle({0.0, 0.0}, 0.0, 1.0, 1e200);
  const auto far_cost = MakeInnerObstacleQuadratic(State({1e200, -1e200}),
                                                  kXy, {distant}, 1.0);
  ASSERT_TRUE(far_cost.valid) << far_cost.reason;
  EXPECT_NEAR(far_cost.value, std::exp(-std::sqrt(2.0)), kTolerance);
  // beta*weight would overflow in double before the small exponential factor.
  const auto weighted = Obstacle({0.0, 0.0}, 0.0, 1e250, 1.0);
  const auto cost = MakeInnerObstacleQuadratic(State({1000.0, 0.0}),
                                              kXy, {weighted}, 1e250);
  ASSERT_TRUE(cost.valid) << cost.reason;
  const double expected = std::exp(500.0 * std::log(10.0) - 1000.0);
  EXPECT_NEAR(cost.value / expected, 1.0, 1e-12);
}

TEST(InnerObstacleCostTest, OverflowingCenterDifferenceIsExplicitlyInvalid) {
  const double max = std::numeric_limits<double>::max();
  const auto obstacle = Obstacle({-max, 0.0});
  const auto cost = MakeInnerObstacleQuadratic(State({max, 0.0}),
                                              kXy, {obstacle}, 1.0);
  EXPECT_FALSE(cost.valid);
  EXPECT_NE(cost.reason.find("nonfinite"), std::string::npos);
  EXPECT_FALSE(EvaluateInnerObstacleCost(State({max, 0.0}),
                                         kXy, {obstacle}, 1.0).valid);
}

TEST(InnerObstacleCostTest, InvalidTrajectoryReturnsNoPartialCosts) {
  EXPECT_FALSE(MakeInnerObstacleTrajectoryQuadratic({}, kXy,
                                                    {Obstacle()}, 1.0).valid);
  for (const auto& terminal :
       std::vector<Eigen::VectorXd>{State({0.2, -0.1}),
                                   Eigen::VectorXd::Zero(6)}) {
    const auto cost = MakeInnerObstacleTrajectoryQuadratic(
        {State({1.4, 0.8}), State({1.0, 0.7}), terminal},
        kXy, {Obstacle()}, 1.0);
    EXPECT_FALSE(cost.valid);
    EXPECT_TRUE(cost.knots.empty());
    EXPECT_NE(cost.reason.find("knot 2"), std::string::npos);
  }
}

TEST(InnerObstacleCostTest, ConvexityCheckRejectsInvalidMatrices) {
  Eigen::Matrix2d Q;
  Q << 1.0, 2.0, 0.0, 1.0;
  EXPECT_FALSE(IsFiniteSymmetricPsd(Q));
  Q << 1.0, 0.0, 0.0, -1.0;
  EXPECT_FALSE(IsFiniteSymmetricPsd(Q));
  Q << 1e308, 1e308, 1e308, 1e308;
  EXPECT_TRUE(IsFiniteSymmetricPsd(Q));
  Q(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsFiniteSymmetricPsd(Q));
  EXPECT_FALSE(IsFiniteSymmetricPsd(Eigen::MatrixXd::Zero(2, 3)));
  EXPECT_FALSE(IsFiniteSymmetricPsd(Eigen::MatrixXd()));
  EXPECT_FALSE(IsFiniteSymmetricPsd(Eigen::Matrix2d::Identity(), -1.0));
}

}  // namespace
}  // namespace systems
}  // namespace dairlib
