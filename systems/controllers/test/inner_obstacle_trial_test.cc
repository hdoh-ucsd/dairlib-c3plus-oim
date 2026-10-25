#include "systems/controllers/inner_obstacle_trial.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "c3/core/c3_plus.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solver_options.h"

namespace dairlib::systems {
namespace {
using c3::C3;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::solvers::MathematicalProgram;
constexpr int kHorizon = 3;
constexpr int kStages = 3;
const std::array<int, 2> kXy{{0, 1}};

VectorXd InitialState() {
  VectorXd x(2);
  x << 0.2, 0.0;
  return x;
}

c3::LCS MakeLCS() {
  MatrixXd D(2, 1);
  D << 1e-4, 0.0;
  return c3::LCS(MatrixXd::Identity(2, 2), MatrixXd::Identity(2, 2), D,
                 VectorXd::Zero(2), MatrixXd::Zero(1, 2),
                 MatrixXd::Identity(1, 1), MatrixXd::Zero(1, 2),
                 VectorXd::Ones(1), kHorizon, 0.1);
}

C3::CostMatrices MakeCosts() {
  return C3::CostMatrices(
      std::vector<MatrixXd>(kHorizon + 1, MatrixXd::Identity(2, 2)),
      std::vector<MatrixXd>(kHorizon, 0.1 * MatrixXd::Identity(2, 2)),
      std::vector<MatrixXd>(kHorizon, 0.01 * MatrixXd::Identity(6, 6)),
      std::vector<MatrixXd>(kHorizon, MatrixXd::Zero(6, 6)));
}

c3::C3Options MakeOptions() {
  c3::C3Options options;
  options.admm_iter = kStages - 1;
  options.rho_scale = 2;
  options.warm_start = false;
  options.end_on_qp_step = false;
  options.scale_lcs = false;
  options.penalize_input_change = false;  // Preserve the pinned legacy semantics.
  options.num_threads = 1;
  return options;
}

std::vector<DiscObstacle> Obstacles() {
  DiscObstacle obstacle;
  obstacle.center.setZero();
  obstacle.radius = 0.1;
  obstacle.weight = 1.0;
  obstacle.sigma = 0.15;
  return {obstacle};
}

std::shared_ptr<c3::C3Plus> MakeCandidate() {
  auto solver = std::make_shared<c3::C3Plus>(
      MakeLCS(), MakeCosts(),
      std::vector<VectorXd>(kHorizon + 1, InitialState()), MakeOptions());
  solver->AddLinearConstraint(MatrixXd::Identity(2, 2),
                              VectorXd::Constant(2, -2.0),
                              VectorXd::Constant(2, 2.0), c3::STATE);
  solver->AddLinearConstraint(MatrixXd::Identity(2, 2),
                              VectorXd::Constant(2, -1.0),
                              VectorXd::Constant(2, 1.0), c3::INPUT);
  drake::solvers::SolverOptions options;
  options.SetOption(drake::solvers::OsqpSolver::id(), "eps_abs", 1e-9);
  options.SetOption(drake::solvers::OsqpSolver::id(), "eps_rel", 1e-9);
  options.SetOption(drake::solvers::OsqpSolver::id(), "max_iter", 20000);
  solver->SetSolverOptions(options);
  return solver;
}

struct Coefficients {
  MatrixXd Q;
  VectorXd b;
  double c;
};

Coefficients CopyCost(const drake::solvers::QuadraticCost& cost) {
  return {cost.Q(), cost.b(), cost.c()};
}

std::vector<Coefficients> CopyStateCosts(C3* solver) {
  std::vector<Coefficients> result;
  for (const auto* cost : solver->GetTargetCost()) {
    result.push_back(CopyCost(*cost));
  }
  return result;
}

template <typename A, typename B>
void ExpectExact(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b) {
  ASSERT_EQ(a.rows(), b.rows());
  ASSERT_EQ(a.cols(), b.cols());
  EXPECT_TRUE((a.array() == b.array()).all());
}

void ExpectCoefficients(const Coefficients& a, const Coefficients& b) {
  ExpectExact(a.Q, b.Q);
  ExpectExact(a.b, b.b);
  EXPECT_DOUBLE_EQ(a.c, b.c);
}

void ExpectRestored(C3* solver, const std::vector<Coefficients>& original) {
  const auto costs = CopyStateCosts(solver);
  ASSERT_EQ(costs.size(), original.size());
  for (size_t k = 0; k < costs.size(); ++k) {
    SCOPED_TRACE(k);
    ExpectCoefficients(costs[k], original[k]);
  }
}

template <typename T>
void ExpectSequence(const std::vector<T>& a, const std::vector<T>& b,
                    double tolerance = 0.0) {
  ASSERT_EQ(a.size(), b.size());
  for (size_t k = 0; k < a.size(); ++k) {
    SCOPED_TRACE(k);
    ASSERT_EQ(a[k].rows(), b[k].rows());
    ASSERT_EQ(a[k].cols(), b[k].cols());
    EXPECT_LE((a[k] - b[k]).cwiseAbs().maxCoeff(), tolerance);
  }
}

void ExpectLCS(const c3::LCS& a, const c3::LCS& b) {
  ASSERT_TRUE(a.HasSameDimensionsAs(b));
  EXPECT_EQ(a.dt(), b.dt());
  ExpectSequence(a.A(), b.A());
  ExpectSequence(a.B(), b.B());
  ExpectSequence(a.D(), b.D());
  ExpectSequence(a.d(), b.d());
  ExpectSequence(a.E(), b.E());
  ExpectSequence(a.F(), b.F());
  ExpectSequence(a.H(), b.H());
  ExpectSequence(a.c(), b.c());
}

void ExpectCostMatrices(const C3::CostMatrices& a, const C3::CostMatrices& b) {
  ExpectSequence(a.Q, b.Q);
  ExpectSequence(a.R, b.R);
  ExpectSequence(a.G, b.G);
  ExpectSequence(a.U, b.U);
}

struct ConstraintSnapshot {
  MatrixXd A;
  VectorXd lower;
  VectorXd upper;
  std::vector<std::string> variables;
};
struct StageSnapshot {
  std::vector<Coefficients> target;
  std::vector<Coefficients> input;
  std::vector<Coefficients> augmented;
  std::vector<ConstraintSnapshot> constraints;
};
using Trace = std::vector<StageSnapshot>;

void Observe(C3* solver, Trace* trace) {
  // Borrow solver while its own callback runs; do not capture its shared_ptr.
  solver->SetQPObserver([solver, trace](int stage, bool final,
                                       const MathematicalProgram& program) {
    EXPECT_EQ(stage, static_cast<int>(trace->size()));
    EXPECT_EQ(final, stage == kStages - 1);
    StageSnapshot snapshot;
    for (const auto* target : solver->GetTargetCost()) {
      int found = 0;
      for (const auto& binding : program.quadratic_costs()) {
        if (binding.evaluator().get() == target) ++found;
      }
      EXPECT_EQ(found, 1);  // Confirm the actual QP contains this evaluator.
      snapshot.target.push_back(CopyCost(*target));
    }
    for (const auto& binding : program.quadratic_costs()) {
      const auto& targets = solver->GetTargetCost();
      if (std::find(targets.begin(), targets.end(), binding.evaluator().get()) !=
          targets.end()) continue;
      const bool input = binding.variables().size() == 2 &&
                         binding.variables()[0].get_name().front() == 'k';
      (input ? snapshot.input : snapshot.augmented)
          .push_back(CopyCost(*binding.evaluator()));
    }
    EXPECT_EQ(snapshot.input.size(), kHorizon);
    for (const auto& binding : program.GetAllConstraints()) {
      const auto* linear = dynamic_cast<const drake::solvers::LinearConstraint*>(
          binding.evaluator().get());
      EXPECT_NE(linear, nullptr);
      if (!linear) continue;
      ConstraintSnapshot constraint{linear->GetDenseA(), linear->lower_bound(),
                                    linear->upper_bound(), {}};
      for (int i = 0; i < binding.variables().size(); ++i) {
        constraint.variables.push_back(binding.variables()[i].get_name());
      }
      snapshot.constraints.push_back(std::move(constraint));
    }
    trace->push_back(std::move(snapshot));
  });
}

void ExpectConstraints(const StageSnapshot& a, const StageSnapshot& b) {
  ASSERT_EQ(a.constraints.size(), b.constraints.size());
  for (size_t i = 0; i < a.constraints.size(); ++i) {
    SCOPED_TRACE(i);
    ExpectExact(a.constraints[i].A, b.constraints[i].A);
    ExpectExact(a.constraints[i].lower, b.constraints[i].lower);
    ExpectExact(a.constraints[i].upper, b.constraints[i].upper);
    EXPECT_EQ(a.constraints[i].variables, b.constraints[i].variables);
  }
}

struct SavedResult {
  std::vector<VectorXd> qp, x, u, lambda, z, delta, w;
};

SavedResult Save(C3* solver) {
  return {solver->GetQPStateSolution(), solver->GetStateSolution(),
          solver->GetInputSolution(), solver->GetForceSolution(),
          solver->GetFullSolution(), solver->GetDualDeltaSolution(),
          solver->GetDualWSolution()};
}

void ExpectSaved(C3* solver, const SavedResult& saved, double tolerance = 0.0) {
  EXPECT_TRUE(solver->GetLastSolveReport().completed);
  ExpectSequence(solver->GetQPStateSolution(), saved.qp, tolerance);
  ExpectSequence(solver->GetStateSolution(), saved.x, tolerance);
  ExpectSequence(solver->GetInputSolution(), saved.u, tolerance);
  ExpectSequence(solver->GetForceSolution(), saved.lambda, tolerance);
  ExpectSequence(solver->GetFullSolution(), saved.z, tolerance);
  ExpectSequence(solver->GetDualDeltaSolution(), saved.delta, tolerance);
  ExpectSequence(solver->GetDualWSolution(), saved.w, tolerance);
}

double ExactCost(const std::vector<VectorXd>& trajectory) {
  double total = 0.0;
  for (size_t k = 1; k < trajectory.size(); ++k) {
    const auto value = EvaluateInnerObstacleCost(trajectory[k], kXy, Obstacles(), 1);
    EXPECT_TRUE(value.valid) << value.reason;
    total += value.value;
  }
  return total;
}

double SurrogateCost(const std::vector<InnerObstacleQuadratic>& approximation,
                     const std::vector<VectorXd>& trajectory) {
  double total = 0.0;
  for (size_t k = 1; k < trajectory.size(); ++k) {
    const auto& x = trajectory[k];
    const auto& q = approximation[k];
    total += 0.5 * x.dot(q.delta_Q * x) + q.delta_b.dot(x) + q.delta_c;
  }
  return total;
}

double OriginalTaskCost(C3* solver) {
  const auto& x = solver->GetQPStateSolution();
  const auto u = solver->GetInputSolution();
  const auto& costs = solver->GetCostMatrices();
  double total = 0;
  for (size_t k = 0; k < x.size(); ++k) {
    const VectorXd error = x[k] - InitialState();
    total += error.dot(costs.Q[k] * error);
    if (k < u.size()) total += u[k].dot(costs.R[k] * u[k]);
  }
  return total;
}

void PrintTrajectory(const std::vector<VectorXd>& trajectory) {
  std::cout << '[';
  for (size_t k = 0; k < trajectory.size(); ++k) {
    if (k) std::cout << ',';
    std::cout << '[';
    for (int i = 0; i < trajectory[k].size(); ++i) {
      if (i) std::cout << ',';
      std::cout << trajectory[k][i];
    }
    std::cout << ']';
  }
  std::cout << ']';
}

TEST(InnerObstacleTrialTest, PositiveCostChangesInnerMotionBeforeRanking) {
  Trace nominal_trace, zero_trace, positive_trace;
  auto nominal = MakeCandidate();
  Observe(nominal.get(), &nominal_trace);
  nominal->Solve(InitialState());  // Original production solve, with support patch.
  ASSERT_TRUE(nominal->GetLastSolveReport().completed);
  const auto saved = Save(nominal.get());
  const auto base_costs = CopyStateCosts(nominal.get());
  const C3::CostMatrices outer_matrices = nominal->GetCostMatrices();
  auto zero = RunInnerObstacleTrial(nominal, [&] {
    auto solver = MakeCandidate();
    Observe(solver.get(), &zero_trace);
    return solver;
  }, InitialState(), kXy, Obstacles(), 0.0);  // Test-only budget control.
  ASSERT_TRUE(zero.trial_selected) << zero.fallback_reason;
  auto positive = RunInnerObstacleTrial(nominal, [&] {
    auto solver = MakeCandidate();
    Observe(solver.get(), &positive_trace);
    return solver;
  }, InitialState(), kXy, Obstacles(), 1.0);
  ASSERT_TRUE(positive.trial_selected) << positive.fallback_reason;
  ASSERT_EQ(nominal_trace.size(), kStages);
  ASSERT_EQ(zero_trace.size(), kStages);
  ASSERT_EQ(positive_trace.size(), kStages);
  ExpectSaved(nominal.get(), saved);
  ExpectSaved(zero.selected.get(), saved, 1e-8);
  ExpectRestored(nominal.get(), base_costs);
  ExpectRestored(zero.selected.get(), base_costs);
  ExpectRestored(positive.selected.get(), base_costs);
  ExpectCostMatrices(nominal->GetCostMatrices(), outer_matrices);
  ExpectCostMatrices(zero.selected->GetCostMatrices(), outer_matrices);
  ExpectCostMatrices(positive.selected->GetCostMatrices(), outer_matrices);
  ExpectLCS(nominal->GetLCS(), zero.selected->GetLCS());
  ExpectLCS(nominal->GetLCS(), positive.selected->GetLCS());
  ExpectSequence(nominal->GetDesiredState(), positive.selected->GetDesiredState());
  std::set<const void*> evaluators;
  for (C3* solver : std::vector<C3*>{nominal.get(), zero.selected.get(),
                                    positive.selected.get()}) {
    for (const auto* cost : solver->GetTargetCost()) {
      EXPECT_TRUE(evaluators.insert(cost).second);
    }
  }
  for (int stage = 0; stage < kStages; ++stage) {
    SCOPED_TRACE(stage);
    const auto& one = nominal_trace[stage];
    const auto& control = zero_trace[stage];
    const auto& enabled = positive_trace[stage];
    ExpectConstraints(one, control);
    ExpectConstraints(control, enabled);
    ASSERT_EQ(control.target.size(), kHorizon + 1);
    for (int k = 0; k <= kHorizon; ++k) {
      SCOPED_TRACE(k);
      ExpectCoefficients(one.target[k], control.target[k]);
      const auto& add = positive.quadratics[k];
      const Coefficients expected{control.target[k].Q + add.delta_Q,
                                  control.target[k].b + add.delta_b,
                                  control.target[k].c + add.delta_c};
      ExpectCoefficients(enabled.target[k], expected);
    }
    for (int k = 0; k < kHorizon; ++k) {
      ExpectCoefficients(one.input[k], control.input[k]);
      ExpectCoefficients(control.input[k], enabled.input[k]);
      EXPECT_TRUE(control.input[k].b.isZero(0.0));
    }
    ASSERT_EQ(one.augmented.size(), control.augmented.size());
    for (size_t k = 0; k < one.augmented.size(); ++k) {
      ExpectCoefficients(one.augmented[k], control.augmented[k]);
      // Later augmented linear terms legitimately follow the changed iterates.
      if (stage == 0) ExpectCoefficients(control.augmented[k], enabled.augmented[k]);
      ExpectExact(control.augmented[k].Q, enabled.augmented[k].Q);
    }
  }
  EXPECT_TRUE(positive.quadratics.front().delta_Q.isZero(0));
  EXPECT_GT(positive.quadratics.back().delta_Q.norm(), 0);
  const double zero_exact = ExactCost(zero.trial_qp_states);
  const double positive_exact = ExactCost(positive.trial_qp_states);
  EXPECT_LT(positive_exact, zero_exact - 0.1);
  EXPECT_GT(positive.trial_qp_states.back()[0], zero.trial_qp_states.back()[0] + 0.05);
  double displacement = 0;
  for (int k = 0; k <= kHorizon; ++k) {
    displacement = std::max(displacement,
        (positive.trial_qp_states[k] - zero.trial_qp_states[k]).norm());
  }
  const double nominal_surrogate = SurrogateCost(positive.quadratics, saved.qp);
  EXPECT_NEAR(nominal_surrogate, ExactCost(saved.qp), 1e-12);
  std::cout << std::setprecision(17) << "INNER_OBSTACLE_FIXTURE {\"seed\":42,"
      << "\"sampling\":\"deterministic; no RNG\",\"beta_inner\":1,"
      << "\"zero_exact\":" << zero_exact
      << ",\"positive_exact\":" << positive_exact
      << ",\"nominal_surrogate\":" << nominal_surrogate
      << ",\"positive_surrogate\":" << SurrogateCost(positive.quadratics, positive.trial_qp_states)
      << ",\"zero_original_task\":" << OriginalTaskCost(zero.selected.get())
      << ",\"positive_original_task\":" << OriginalTaskCost(positive.selected.get())
      << ",\"maximum_position_displacement\":" << displacement
      << ",\"zero_construction_seconds\":" << zero.construction_seconds
      << ",\"positive_construction_seconds\":" << positive.construction_seconds
      << ",\"zero_solve_seconds\":" << zero.solve_seconds
      << ",\"positive_solve_seconds\":" << positive.solve_seconds
      << ",\"nominal_qp_x\":";
  PrintTrajectory(saved.qp);
  std::cout << ",\"positive_qp_x\":";
  PrintTrajectory(positive.trial_qp_states);
  std::cout << ",\"nominal_u\":";
  PrintTrajectory(saved.u);
  std::cout << ",\"positive_u\":";
  PrintTrajectory(positive.selected->GetInputSolution());
  std::cout << ",\"nominal_physical_lambda\":";
  PrintTrajectory(saved.lambda);
  std::cout << ",\"positive_physical_lambda\":";
  PrintTrajectory(positive.selected->GetForceSolution());
  std::cout << "}" << std::endl;
}

TEST(InnerObstacleTrialTest, RealInfeasibleTrialRestoresCostsAndRetainsEveryNominalField) {
  auto nominal = MakeCandidate();
  nominal->Solve(InitialState());
  ASSERT_TRUE(nominal->GetLastSolveReport().completed);
  const auto saved = Save(nominal.get());
  const auto costs = CopyStateCosts(nominal.get());
  std::shared_ptr<c3::C3Plus> failed;
  auto result = RunInnerObstacleTrial(nominal, [&] {
    failed = MakeCandidate();
    // Contradicts the factory's existing [-1,1] bounds at every input knot.
    // Pinned STATE bounds exclude x[0], so do not rely on an initial-state clash.
    failed->AddLinearConstraint(MatrixXd::Identity(2, 2),
                                2 * VectorXd::Ones(2),
                                3 * VectorXd::Ones(2), c3::INPUT);
    return failed;
  }, InitialState(), kXy, Obstacles(), 1.0);
  EXPECT_TRUE(result.trial_attempted);
  EXPECT_FALSE(result.trial_selected);
  EXPECT_EQ(result.selected, nominal);
  EXPECT_FALSE(result.trial_report.completed);
  ASSERT_EQ(result.trial_report.qp_stages.size(), 1u);
  EXPECT_EQ(result.trial_report.qp_stages[0].outcome, C3::QPOutcome::kSolverFailure);
  EXPECT_TRUE(result.trial_qp_states.empty());
  EXPECT_FALSE(result.fallback_reason.empty());
  ExpectRestored(failed.get(), costs);
  ExpectRestored(nominal.get(), costs);
  ExpectSaved(nominal.get(), saved);
  EXPECT_THROW(failed->GetQPStateSolution(), std::logic_error);
  failed.reset();
  nominal.reset();
  // Ownership survives the factory, failed trial destruction, and caller reset.
  ExpectSaved(result.selected.get(), saved);
}

TEST(InnerObstacleTrialTest, ExceptionsAndInvalidApproximationUseOwnedNominal) {
  auto nominal = MakeCandidate();
  nominal->Solve(InitialState());
  ASSERT_TRUE(nominal->GetLastSolveReport().completed);
  const auto saved = Save(nominal.get());
  const auto costs = CopyStateCosts(nominal.get());
  std::shared_ptr<c3::C3Plus> failed;
  auto interrupted = RunInnerObstacleTrial(nominal, [&] {
    failed = MakeCandidate();
    failed->SetQPObserver([](int, bool, const MathematicalProgram&) {
      throw std::runtime_error("test observer exception after coefficient application");
    });
    return failed;
  }, InitialState(), kXy, Obstacles(), 1.0);
  EXPECT_FALSE(interrupted.trial_selected);
  EXPECT_TRUE(interrupted.trial_report.had_exception);
  EXPECT_EQ(interrupted.selected, nominal);
  ExpectRestored(failed.get(), costs);
  ExpectSaved(nominal.get(), saved);
  int constructions = 0;
  auto invalid_obstacles = Obstacles();
  invalid_obstacles[0].center = saved.qp[1];  // Active singular radial normal.
  auto invalid = RunInnerObstacleTrial(nominal, [&] {
    ++constructions;
    return MakeCandidate();
  }, InitialState(), kXy, invalid_obstacles, 1.0);
  EXPECT_EQ(constructions, 0);
  EXPECT_FALSE(invalid.trial_attempted);
  EXPECT_FALSE(invalid.trial_selected);
  EXPECT_EQ(invalid.selected, nominal);
  EXPECT_NE(invalid.fallback_reason.find("singular"), std::string::npos);
  auto factory_exception = RunInnerObstacleTrial(nominal, []() -> std::shared_ptr<C3> {
    throw std::runtime_error("test factory exception");
  }, InitialState(), kXy, Obstacles(), 1.0);
  EXPECT_EQ(factory_exception.selected, nominal);
  EXPECT_FALSE(factory_exception.trial_attempted);
  EXPECT_NE(factory_exception.fallback_reason.find("factory exception"), std::string::npos);
  auto nonstandard_exception = RunInnerObstacleTrial(nominal, []() -> std::shared_ptr<C3> {
    throw 42;
  }, InitialState(), kXy, Obstacles(), 1.0);
  EXPECT_EQ(nonstandard_exception.selected, nominal);
  EXPECT_FALSE(nonstandard_exception.trial_attempted);
  EXPECT_NE(nonstandard_exception.fallback_reason.find("non-standard"), std::string::npos);
  ExpectSaved(nominal.get(), saved);
  ExpectRestored(nominal.get(), costs);
}

TEST(InnerObstacleTrialTest, RepeatedFreshTrialsDoNotAccumulateAndRemainOwned) {
  auto nominal = MakeCandidate();
  nominal->Solve(InitialState());
  ASSERT_TRUE(nominal->GetLastSolveReport().completed);
  const auto costs = CopyStateCosts(nominal.get());
  const auto saved = Save(nominal.get());
  std::weak_ptr<C3> weak_trial;
  auto first = RunInnerObstacleTrial(nominal, [&] {
    auto candidate = MakeCandidate();
    weak_trial = candidate;
    return candidate;
  }, InitialState(), kXy, Obstacles(), 1.0);
  ASSERT_TRUE(first.trial_selected) << first.fallback_reason;
  EXPECT_FALSE(weak_trial.expired());
  const auto selected_saved = Save(first.selected.get());
  auto second = RunInnerObstacleTrial(nominal, MakeCandidate, InitialState(),
                                      kXy, Obstacles(), 1.0);
  ASSERT_TRUE(second.trial_selected) << second.fallback_reason;
  EXPECT_NE(first.selected.get(), second.selected.get());
  ExpectSaved(second.selected.get(), selected_saved, 1e-8);
  ExpectSaved(nominal.get(), saved);
  ExpectRestored(first.selected.get(), costs);
  ExpectRestored(second.selected.get(), costs);
  nominal.reset();
  second.selected.reset();
  // Exercise every getter consumed by ranking, buffering and execution while
  // the result object is the sole owner of the independently solved trial.
  ExpectSaved(first.selected.get(), selected_saved);
  EXPECT_TRUE(std::isfinite(OriginalTaskCost(first.selected.get())));
  first.selected.reset();
  EXPECT_TRUE(weak_trial.expired());
}

TEST(InnerObstacleTrialTest, RejectsAliasedFactoryAndUnsuccessfulNominal) {
  auto nominal = MakeCandidate();
  nominal->Solve(InitialState());
  ASSERT_TRUE(nominal->GetLastSolveReport().completed);
  const auto saved = Save(nominal.get());
  const auto aliased = RunInnerObstacleTrial(nominal, [&] { return nominal; },
      InitialState(), kXy, Obstacles(), 1.0);
  EXPECT_FALSE(aliased.trial_attempted);
  EXPECT_FALSE(aliased.trial_selected);
  EXPECT_NE(aliased.fallback_reason.find("independent"), std::string::npos);
  ExpectSaved(nominal.get(), saved);
  auto unsuccessful = MakeCandidate();
  int constructions = 0;
  auto result = RunInnerObstacleTrial(unsuccessful, [&] {
    ++constructions;
    return MakeCandidate();
  }, InitialState(), kXy, Obstacles(), 1.0);
  EXPECT_EQ(constructions, 0);
  EXPECT_FALSE(result.trial_attempted);
  EXPECT_FALSE(result.trial_selected);
  EXPECT_EQ(result.selected, unsuccessful);
}

TEST(InnerObstacleTrialTest, ProductionGatePreservesSingleSolveBypass) {
  auto nominal = MakeCandidate();
  Trace trace;
  Observe(nominal.get(), &trace);
  nominal->Solve(InitialState());
  ASSERT_TRUE(nominal->GetLastSolveReport().completed);
  const auto saved = Save(nominal.get());
  int factory_calls = 0;
  const auto factory = [&] {
    ++factory_calls;
    return MakeCandidate();
  };
  for (const auto settings :
       std::vector<std::tuple<bool, double, size_t>>{{false, 1.0, 1},
                                                   {true, 0.0, 1},
                                                   {true, 1.0, 0}}) {
    if (ShouldRunInnerObstacleTrial(std::get<0>(settings), std::get<1>(settings),
                                   std::get<2>(settings))) {
      RunInnerObstacleTrial(nominal, factory, InitialState(), kXy, Obstacles(),
                            std::get<1>(settings));
    }
    ExpectSaved(nominal.get(), saved);
  }
  EXPECT_EQ(factory_calls, 0);
  EXPECT_EQ(trace.size(), kStages);
  EXPECT_TRUE(ShouldRunInnerObstacleTrial(true, 1.0, 1));
}
}  // namespace
}  // namespace dairlib::systems
