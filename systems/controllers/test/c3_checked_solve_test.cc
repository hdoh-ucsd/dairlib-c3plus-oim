#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "c3/core/c3.h"
#include "c3/core/c3_plus.h"
#include "drake/solvers/mathematical_program_result.h"

namespace {
using c3::C3;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
constexpr int kHorizon = 2;

c3::LCS MakeLCS() {
  return c3::LCS(MatrixXd::Identity(1, 1), MatrixXd::Identity(1, 1),
                 MatrixXd::Constant(1, 1, 0.1), VectorXd::Zero(1),
                 MatrixXd::Zero(1, 1), MatrixXd::Identity(1, 1),
                 MatrixXd::Zero(1, 1), VectorXd::Ones(1), kHorizon, 0.1);
}

c3::C3Options MakeOptions(std::optional<bool> input_change = false) {
  c3::C3Options options;
  options.admm_iter = 2;
  options.rho_scale = 2;
  options.warm_start = false;
  options.end_on_qp_step = false;
  options.scale_lcs = false;
  options.penalize_input_change = input_change;
  options.num_threads = 1;
  return options;
}

C3::CostMatrices MakeCosts(bool plus = false) {
  const int nz = plus ? 4 : 3;
  return C3::CostMatrices(
      std::vector<MatrixXd>(kHorizon + 1, MatrixXd::Identity(1, 1)),
      std::vector<MatrixXd>(kHorizon, MatrixXd::Constant(1, 1, 0.5)),
      std::vector<MatrixXd>(kHorizon, 0.01 * MatrixXd::Identity(nz, nz)),
      std::vector<MatrixXd>(kHorizon, MatrixXd::Zero(nz, nz)));
}

std::vector<VectorXd> Target() {
  return std::vector<VectorXd>(kHorizon + 1, VectorXd::Ones(1));
}

// Exercise the actual common ADMM loop and pinned OSQP result object. Faults
// exist only in this test subclass, not in production flags or C3Plus.
class FaultFixture final : public C3 {
 public:
  enum class Fault { kNone, kFailure, kNaN, kException, kPreparationException,
                     kNonfinitePublishedOutput, kNonStdExtractionException };
  FaultFixture() : C3(MakeLCS(), MakeCosts(), Target(), MakeOptions()) {
    use_parallelization_in_projection_ = false;
  }
  void Arm(int stage, Fault fault) {
    fault_stage_ = stage;
    fault_ = fault;
    stage_ = -1;
    stored_stages.clear();
  }
  std::vector<int> stored_stages;

 protected:
  VectorXd SolveSingleProjection(const MatrixXd&, const VectorXd& value,
                                const MatrixXd&, const MatrixXd&,
                                const MatrixXd&, const VectorXd&, int,
                                const int&) override {
    return value;
  }
  void AddAugmentedCost(const std::vector<MatrixXd>& G,
                        const std::vector<VectorXd>& wd,
                        const std::vector<VectorXd>& delta,
                        bool final) override {
    ++stage_;
    if (stage_ == fault_stage_ && fault_ == Fault::kPreparationException) {
      throw std::runtime_error("injected QP preparation exception");
    }
    C3::AddAugmentedCost(G, wd, delta, final);
  }
  MathematicalProgramResult SolveQPProgram() override {
    auto result = C3::SolveQPProgram();
    if (stage_ != fault_stage_) return result;
    if (fault_ == Fault::kFailure) {
      result.set_solution_result(SolutionResult::kInfeasibleConstraints);
    } else if (fault_ == Fault::kNaN) {
      VectorXd x = result.get_x_val();
      x[0] = std::numeric_limits<double>::quiet_NaN();
      result.set_x_val(x);
    } else if (fault_ == Fault::kException) {
      throw std::runtime_error("injected QP exception");
    }
    return result;
  }
  void StoreQPResults(const MathematicalProgramResult& result, int iteration,
                      bool final) override {
    stored_stages.push_back(iteration);
    C3::StoreQPResults(result, iteration, final);
    if (stage_ == fault_stage_ && fault_ == Fault::kNonStdExtractionException) {
      throw 7;
    }
    if (stage_ == fault_stage_ && fault_ == Fault::kNonfinitePublishedOutput) {
      lambda_sol_->at(0)[0] = std::numeric_limits<double>::quiet_NaN();
    }
  }
 private:
  int stage_ = -1;
  int fault_stage_ = -1;
  Fault fault_ = Fault::kNone;
};

void ExpectComplete(const C3::SolveReport& report) {
  EXPECT_TRUE(report.attempted);
  EXPECT_TRUE(report.completed) << report.failure_reason;
  EXPECT_FALSE(report.had_exception);
  EXPECT_FALSE(report.invalid_output);
  ASSERT_EQ(report.qp_stages.size(), 3u);
  EXPECT_EQ(report.required_qp_count, 3);
  for (int i = 0; i < 3; ++i) {
    const auto& qp = report.qp_stages[i];
    EXPECT_EQ(qp.admm_iteration, i);
    EXPECT_EQ(qp.is_final, i == 2);
    EXPECT_TRUE(qp.solver_called);
    EXPECT_TRUE(qp.underlying_solver_success);
    EXPECT_EQ(qp.outcome, C3::QPOutcome::kSucceeded);
    EXPECT_EQ(qp.solver_status, SolutionResult::kSolutionFound);
  }
}

TEST(C3CheckedSolve, StatusResetsAfterSuccessAndAfterEachFailedStage) {
  FaultFixture solver;
  EXPECT_FALSE(solver.GetLastSolveReport().attempted);
  EXPECT_FALSE(solver.GetLastSolveReport().completed);
  EXPECT_THROW(solver.GetQPStateSolution(), std::logic_error);
  for (int failed_stage : {0, 1, 2}) {
    solver.Arm(-1, FaultFixture::Fault::kNone);
    ExpectComplete(solver.SolveChecked(VectorXd::Zero(1)));
    solver.Arm(failed_stage, FaultFixture::Fault::kFailure);
    const auto failed = solver.SolveChecked(VectorXd::Zero(1));
    EXPECT_FALSE(failed.completed);
    ASSERT_EQ(failed.qp_stages.size(), failed_stage + 1u);
    const auto& stage = failed.qp_stages.back();
    EXPECT_EQ(stage.admm_iteration, failed_stage);
    EXPECT_EQ(stage.is_final, failed_stage == 2);
    EXPECT_FALSE(stage.underlying_solver_success);
    EXPECT_EQ(stage.outcome, C3::QPOutcome::kSolverFailure);
    EXPECT_EQ(stage.solver_status, SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(solver.stored_stages.size(), static_cast<size_t>(failed_stage));
    EXPECT_THROW(solver.GetQPStateSolution(), std::logic_error);
    solver.Arm(-1, FaultFixture::Fault::kNone);
    ExpectComplete(solver.SolveChecked(VectorXd::Zero(1)));
  }
}

TEST(C3CheckedSolve, InvalidOutputAndExceptionsCannotPublishCheckedTrajectory) {
  for (const auto fault : {FaultFixture::Fault::kNaN,
                          FaultFixture::Fault::kException,
                          FaultFixture::Fault::kPreparationException}) {
    for (const int failed_stage : {0, 2}) {
      FaultFixture solver;
      solver.Arm(failed_stage, fault);
      const auto report = solver.SolveChecked(VectorXd::Zero(1));
      EXPECT_FALSE(report.completed);
      ASSERT_EQ(report.qp_stages.size(), failed_stage + 1u);
      EXPECT_EQ(solver.stored_stages.size(), static_cast<size_t>(failed_stage));
      if (fault == FaultFixture::Fault::kNaN) {
        EXPECT_TRUE(report.invalid_output);
        EXPECT_TRUE(report.qp_stages.back().underlying_solver_success);
        EXPECT_EQ(report.qp_stages.back().outcome, C3::QPOutcome::kInvalidOutput);
      } else {
        EXPECT_TRUE(report.had_exception);
        EXPECT_EQ(report.qp_stages.back().outcome, C3::QPOutcome::kException);
      }
      EXPECT_EQ(report.qp_stages.back().solver_called,
                fault != FaultFixture::Fault::kPreparationException);
      EXPECT_THROW(solver.GetQPStateSolution(), std::logic_error);
    }
  }
  FaultFixture solver;
  solver.Arm(2, FaultFixture::Fault::kNonfinitePublishedOutput);
  const auto report = solver.SolveChecked(VectorXd::Zero(1));
  EXPECT_FALSE(report.completed);
  EXPECT_TRUE(report.invalid_output);
  EXPECT_THROW(solver.GetQPStateSolution(), std::logic_error);
}

TEST(C3CheckedSolve, NonStdExtractionExceptionMarksStageAndPreservesLegacyThrow) {
  FaultFixture solver;
  solver.Arm(2, FaultFixture::Fault::kNonStdExtractionException);
  const auto report = solver.SolveChecked(VectorXd::Zero(1));
  EXPECT_FALSE(report.completed);
  EXPECT_TRUE(report.had_exception);
  ASSERT_EQ(report.qp_stages.size(), 3u);
  EXPECT_EQ(report.qp_stages.back().outcome, C3::QPOutcome::kException);
  EXPECT_TRUE(report.qp_stages.back().underlying_solver_success);
  EXPECT_THROW(solver.GetQPStateSolution(), std::logic_error);
  solver.Arm(2, FaultFixture::Fault::kNonStdExtractionException);
  EXPECT_THROW(solver.Solve(VectorXd::Zero(1)), int);
  EXPECT_FALSE(solver.GetLastSolveReport().completed);
  EXPECT_EQ(solver.GetLastSolveReport().qp_stages.back().outcome,
            C3::QPOutcome::kException);
}

TEST(C3CheckedSolve, InvalidInitialStateDoesNotReusePreviousStatus) {
  FaultFixture solver;
  ExpectComplete(solver.SolveChecked(VectorXd::Zero(1)));
  for (const VectorXd input : {VectorXd::Zero(2).eval(),
       VectorXd::Constant(1, std::numeric_limits<double>::quiet_NaN()).eval()}) {
    const auto report = solver.SolveChecked(input);
    EXPECT_TRUE(report.attempted);
    EXPECT_FALSE(report.completed);
    EXPECT_TRUE(report.qp_stages.empty());
    EXPECT_FALSE(report.failure_reason.empty());
    EXPECT_THROW(solver.GetQPStateSolution(), std::logic_error);
  }
}

TEST(C3CheckedSolve, LegacyFinalSuccessDoesNotEraseIntermediateFailure) {
  FaultFixture solver;
  solver.Arm(0, FaultFixture::Fault::kFailure);
  solver.Solve(VectorXd::Zero(1));
  const auto& report = solver.GetLastSolveReport();
  EXPECT_FALSE(report.completed);
  ASSERT_EQ(report.qp_stages.size(), 3u);
  EXPECT_EQ(report.qp_stages.front().outcome, C3::QPOutcome::kSolverFailure);
  EXPECT_EQ(report.qp_stages.back().outcome, C3::QPOutcome::kSucceeded);
  EXPECT_TRUE(report.qp_stages.back().is_final);
  EXPECT_EQ(solver.GetStateSolution().size(), kHorizon);
  EXPECT_THROW(solver.GetQPStateSolution(), std::logic_error);
  solver.Arm(-1, FaultFixture::Fault::kNone);
  solver.Solve(VectorXd::Zero(1));
  ExpectComplete(solver.GetLastSolveReport());
}

TEST(C3CheckedSolve, RealInfeasibleQPAndIndependentNominalOwnership) {
  auto nominal = std::make_shared<c3::C3Plus>(
      MakeLCS(), MakeCosts(true), Target(), MakeOptions());
  nominal->Solve(VectorXd::Zero(1));
  ExpectComplete(nominal->GetLastSolveReport());
  const auto saved = nominal->GetFullSolution();
  const auto saved_qp = nominal->GetQPStateSolution();
  auto trial = std::make_shared<c3::C3Plus>(
      MakeLCS(), MakeCosts(true), Target(), MakeOptions());
  // Two valid bound sets on the same input are jointly infeasible. The pinned
  // public STATE-bound helper excludes x[0], so initial-state conflict is not
  // a reliable way to create this failure fixture.
  trial->AddLinearConstraint(MatrixXd::Identity(1, 1), -VectorXd::Ones(1),
                            VectorXd::Ones(1), c3::INPUT);
  trial->AddLinearConstraint(MatrixXd::Identity(1, 1), 2 * VectorXd::Ones(1),
                            3 * VectorXd::Ones(1), c3::INPUT);
  const auto failed = trial->SolveChecked(VectorXd::Zero(1));
  EXPECT_FALSE(failed.completed);
  ASSERT_EQ(failed.qp_stages.size(), 1u);
  EXPECT_EQ(failed.qp_stages.front().outcome, C3::QPOutcome::kSolverFailure);
  trial.reset();
  ExpectComplete(nominal->GetLastSolveReport());
  for (int i = 0; i < kHorizon; ++i) {
    EXPECT_TRUE(saved[i].isApprox(nominal->GetFullSolution()[i], 0));
  }
  ASSERT_EQ(saved_qp.size(), kHorizon + 1u);
  for (int i = 0; i <= kHorizon; ++i) {
    EXPECT_TRUE(saved_qp[i].isApprox(nominal->GetQPStateSolution()[i], 0));
    EXPECT_TRUE(saved_qp[i].allFinite());
  }
}

struct Coefficients { MatrixXd Q; VectorXd b; double c; };
using StageInputs = std::vector<Coefficients>;
using InputTrace = std::vector<StageInputs>;

void ObserveInputs(C3* solver, InputTrace* trace) {
  solver->SetQPObserver([trace](int iteration, bool final,
                               const MathematicalProgram& program) {
    EXPECT_EQ(iteration, static_cast<int>(trace->size()));
    EXPECT_EQ(final, iteration == 2);
    StageInputs input;
    for (const auto& binding : program.quadratic_costs()) {
      if (binding.variables().size() == 1 &&
          binding.variables()[0].get_name().front() == 'k') {
        input.push_back({binding.evaluator()->Q(), binding.evaluator()->b(),
                         binding.evaluator()->c()});
      }
    }
    EXPECT_EQ(input.size(), kHorizon);
    trace->push_back(std::move(input));
  });
}

TEST(C3CheckedSolve, ConsumedInputCostsPreserveLegacyFlagAndFreshTrialIdentity) {
  for (const std::optional<bool> flag :
       {std::optional<bool>(false), std::optional<bool>(true), std::optional<bool>()}) {
    c3::C3Plus nominal(MakeLCS(), MakeCosts(true), Target(), MakeOptions(flag));
    c3::C3Plus trial(MakeLCS(), MakeCosts(true), Target(), MakeOptions(flag));
    InputTrace first, fresh, second;
    ObserveInputs(&nominal, &first);
    nominal.Solve(VectorXd::Zero(1));
    ExpectComplete(nominal.GetLastSolveReport());
    const auto nominal_u = nominal.GetInputSolution();
    ASSERT_GT(nominal_u[0].norm(), 0.1);
    ObserveInputs(&trial, &fresh);
    ExpectComplete(trial.SolveChecked(VectorXd::Zero(1)));
    ObserveInputs(&nominal, &second);
    nominal.Solve(VectorXd::Zero(1));
    ExpectComplete(nominal.GetLastSolveReport());
    ASSERT_EQ(first.size(), 3u);
    ASSERT_EQ(fresh.size(), 3u);
    ASSERT_EQ(second.size(), 3u);
    for (int stage = 0; stage < 3; ++stage) {
      for (int k = 0; k < kHorizon; ++k) {
        const auto& a = first[stage][k];
        const auto& b = fresh[stage][k];
        const auto& c = second[stage][k];
        EXPECT_TRUE(a.Q.isApprox(b.Q, 0));
        EXPECT_TRUE(a.b.isApprox(b.b, 0));
        EXPECT_DOUBLE_EQ(a.c, b.c);
        EXPECT_TRUE(a.Q.isApprox(c.Q, 0));
        EXPECT_DOUBLE_EQ(a.c, c.c);
        EXPECT_DOUBLE_EQ(a.b[0], 0.0);
        // Legacy engagement check is deliberately preserved for both false
        // and true; only missing/nullopt skips rewriting the linear term.
        EXPECT_NEAR(c.b[0], flag.has_value() ? -nominal_u[k][0] : 0.0, 1e-13);
      }
    }
  }
}

TEST(C3CheckedSolve, ActualTerminalQPStateIncludesFinalKnotBeforeHalfStep) {
  auto options = MakeOptions();
  options.scale_lcs = true;
  const auto physical_lcs = MakeLCS();
  c3::C3Plus solver(physical_lcs, MakeCosts(true), Target(), options);
  ExpectComplete(solver.SolveChecked(VectorXd::Zero(1)));
  const auto& qp = solver.GetQPStateSolution();
  ASSERT_EQ(qp.size(), kHorizon + 1u);
  EXPECT_EQ(solver.GetStateSolution().size(), kHorizon);
  for (int k = 0; k < kHorizon; ++k) {
    if (k == 0) {
      // The legacy half step replaces its public x[0] by the exact measured
      // initial state; the new getter retains the finite-tolerance QP value.
      EXPECT_LT((qp[k] - solver.GetStateSolution()[k]).norm(), 1e-6);
    } else {
      EXPECT_TRUE(qp[k].isApprox(solver.GetStateSolution()[k], 0));
    }
    const VectorXd predicted = physical_lcs.A()[k] * qp[k] +
        physical_lcs.B()[k] * solver.GetInputSolution()[k] +
        physical_lcs.D()[k] * solver.GetForceSolution()[k] + physical_lcs.d()[k];
    EXPECT_LT((predicted - qp[k + 1]).norm(), 1e-4);
  }
}
}  // namespace
