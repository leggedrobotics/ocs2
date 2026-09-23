/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>

#include <limits>
#include <ocs2_core/constraint/LinearStateConstraint.h>

#include "ocs2_ipm/IpmSolver.h"

#include <ocs2_core/initialization/DefaultInitializer.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/test/testProblemsGeneration.h>

namespace ocs2 {
namespace {

std::pair<PrimalSolution, std::vector<PerformanceIndex>> solveWithFeedbackSetting(
    bool feedback, bool emptyConstraint, const VectorFunctionLinearApproximation& dynamicsMatrices,
    const ScalarFunctionQuadraticApproximation& costMatrices) {
  int n = dynamicsMatrices.dfdu.rows();
  int m = dynamicsMatrices.dfdu.cols();

  ocs2::OptimalControlProblem problem;

  // System
  problem.dynamicsPtr = getOcs2Dynamics(dynamicsMatrices);

  // Cost
  problem.costPtr->add("intermediateCost", ocs2::getOcs2Cost(costMatrices));
  problem.finalCostPtr->add("finalCost", ocs2::getOcs2StateCost(costMatrices));

  // Reference Managaer
  ocs2::TargetTrajectories targetTrajectories({0.0}, {ocs2::vector_t::Ones(n)}, {ocs2::vector_t::Ones(m)});
  auto referenceManagerPtr = std::make_shared<ReferenceManager>(targetTrajectories);

  problem.targetTrajectoriesPtr = &referenceManagerPtr->getTargetTrajectories();

  if (emptyConstraint) {
    problem.equalityConstraintPtr->add("intermediateCost", ocs2::getOcs2Constraints(getRandomConstraints(n, m, 0)));
  }

  ocs2::DefaultInitializer zeroInitializer(m);

  // Solver settings
  ocs2::ipm::Settings settings;
  settings.dt = 0.05;
  settings.ipmIteration = 10;
  settings.useFeedbackPolicy = feedback;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 100;

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = ocs2::vector_t::Ones(n);

  // Construct solver
  ocs2::IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);

  // Solve
  solver.run(startTime, initState, finalTime);
  return {solver.primalSolution(finalTime), solver.getIterationsLog()};
}

}  // namespace
}  // namespace ocs2

TEST(test_unconstrained, withFeedback) {
  int n = 3;
  int m = 2;
  const double tol = 1e-9;
  const auto dynamics = ocs2::getRandomDynamics(n, m);
  const auto costs = ocs2::getRandomCost(n, m);
  const auto solWithEmptyConstraint = ocs2::solveWithFeedbackSetting(true, true, dynamics, costs);
  const auto solWithNullConstraint = ocs2::solveWithFeedbackSetting(true, false, dynamics, costs);

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(solWithEmptyConstraint.second.size(), 2);
  ASSERT_LE(solWithNullConstraint.second.size(), 2);
  ASSERT_LT(solWithEmptyConstraint.second.back().dynamicsViolationSSE, tol);
  ASSERT_LT(solWithNullConstraint.second.back().dynamicsViolationSSE, tol);

  // Compare
  const auto& withEmptyConstraint = solWithEmptyConstraint.first;
  const auto& withNullConstraint = solWithNullConstraint.first;
  for (int i = 0; i < withEmptyConstraint.timeTrajectory_.size(); i++) {
    ASSERT_DOUBLE_EQ(withEmptyConstraint.timeTrajectory_[i], withNullConstraint.timeTrajectory_[i]);
    ASSERT_TRUE(withEmptyConstraint.stateTrajectory_[i].isApprox(withNullConstraint.stateTrajectory_[i], tol));
    ASSERT_TRUE(withEmptyConstraint.inputTrajectory_[i].isApprox(withNullConstraint.inputTrajectory_[i], tol));
    const auto t = withEmptyConstraint.timeTrajectory_[i];
    const auto& x = withEmptyConstraint.stateTrajectory_[i];
    ASSERT_TRUE(
        withEmptyConstraint.controllerPtr_->computeInput(t, x).isApprox(withNullConstraint.controllerPtr_->computeInput(t, x), tol));
  }
}

TEST(test_unconstrained, noFeedback) {
  int n = 3;
  int m = 2;
  const double tol = 1e-9;
  const auto dynamics = ocs2::getRandomDynamics(n, m);
  const auto costs = ocs2::getRandomCost(n, m);
  const auto solWithEmptyConstraint = ocs2::solveWithFeedbackSetting(false, true, dynamics, costs);
  const auto solWithNullConstraint = ocs2::solveWithFeedbackSetting(false, false, dynamics, costs);

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(solWithEmptyConstraint.second.size(), 2);
  ASSERT_LE(solWithNullConstraint.second.size(), 2);
  ASSERT_LT(solWithEmptyConstraint.second.back().dynamicsViolationSSE, tol);
  ASSERT_LT(solWithNullConstraint.second.back().dynamicsViolationSSE, tol);

  // Compare
  const auto& withEmptyConstraint = solWithEmptyConstraint.first;
  const auto& withNullConstraint = solWithNullConstraint.first;
  for (int i = 0; i < withEmptyConstraint.timeTrajectory_.size(); i++) {
    ASSERT_DOUBLE_EQ(withEmptyConstraint.timeTrajectory_[i], withNullConstraint.timeTrajectory_[i]);
    ASSERT_TRUE(withEmptyConstraint.stateTrajectory_[i].isApprox(withNullConstraint.stateTrajectory_[i], tol));
    ASSERT_TRUE(withEmptyConstraint.inputTrajectory_[i].isApprox(withNullConstraint.inputTrajectory_[i], tol));

    const auto t = withEmptyConstraint.timeTrajectory_[i];
    const auto& x = withEmptyConstraint.stateTrajectory_[i];
    ASSERT_TRUE(
        withEmptyConstraint.controllerPtr_->computeInput(t, x).isApprox(withNullConstraint.controllerPtr_->computeInput(t, x), tol));
  }
}

namespace ocs2 {

TEST(IpmSharedStep, HonorsDualBoundaryWithoutChangingIndependentMode) {
  constexpr scalar_t mu = 0.01;
  const auto solveOneStep = [&](bool sharedStep) {
    // A zero-input seed has h(u)=1+u=1. The quadratic cost pushes
    // into the feasible interior, so only the dual fraction limits the step.
    auto dynamics = VectorFunctionLinearApproximation::Zero(1, 1, 1);
    auto cost = ScalarFunctionQuadraticApproximation::Zero(1, 1);
    cost.dfdxx(0, 0) = 1.0;
    cost.dfduu(0, 0) = 1.0;
    OptimalControlProblem problem;
    problem.dynamicsPtr = getOcs2Dynamics(dynamics);
    problem.costPtr->add("quadratic", getOcs2Cost(cost));
    problem.finalCostPtr->add("terminal", getOcs2StateCost(cost));
    problem.inequalityConstraintPtr->add(
        "positiveInput", std::make_unique<LinearStateInputConstraint>(
                             vector_t::Ones(1), matrix_t::Zero(1, 1), matrix_t::Ones(1, 1)));
    auto reference = std::make_shared<ReferenceManager>(
        TargetTrajectories({0.0}, {vector_t::Zero(1)}, {vector_t::Constant(1, 10.0)}));

    ipm::Settings settings;
    settings.dt = 1.0;
    settings.ipmIteration = 1;
    settings.nThreads = 1;
    settings.useFeedbackPolicy = false;
    settings.usePrimalStepSizeForDual = sharedStep;
    settings.initialBarrierParameter = mu;
    settings.targetBarrierParameter = mu;
    settings.initialSlackMarginRate = 0.0;
    settings.initialDualMarginRate = 0.0;
    DefaultInitializer initializer(1);
    IpmSolver solver(settings, problem, initializer);
    solver.setReferenceManager(reference);
    solver.run(0.0, vector_t::Zero(1), 1.0);
    const auto policy = solver.primalSolution(1.0);
    const scalar_t input = policy.inputTrajectory_.front()(0);
    const scalar_t dual = solver.getDualSolution()->intermediates.front().stateInputIneq.front().lagrangian(0);
    return std::make_pair(input, dual);
  };

  const auto shared = solveOneStep(true);
  const auto independent = solveOneStep(false);
  EXPECT_GT(shared.first, 0.5);
  EXPECT_LT(shared.first, 2.0);
  // The uncapped Newton input step is (10+mu)/(1+mu).
  EXPECT_NEAR(independent.first, (10.0 + mu) / (1.0 + mu), 1e-8);
  EXPECT_GT(shared.second, 0.0);
  EXPECT_GT(independent.second, 0.0);
}

}  // namespace ocs2

namespace ocs2 {
namespace {

OptimalControlProblem restorationProblem(scalar_t terminalUpper = 0.0) {
  auto dynamics = VectorFunctionLinearApproximation::Zero(1, 1, 1);
  dynamics.dfdx(0, 0) = 1.0;
  dynamics.dfdu(0, 0) = 1.0;
  auto cost = ScalarFunctionQuadraticApproximation::Zero(1, 1);
  cost.dfdxx(0, 0) = 0.001;
  cost.dfduu(0, 0) = 1.0;
  OptimalControlProblem problem;
  problem.dynamicsPtr = getOcs2Dynamics(dynamics);
  problem.costPtr->add("quadratic", getOcs2Cost(cost));
  problem.finalCostPtr->add("terminal", getOcs2StateCost(cost));
  problem.inequalityConstraintPtr->add(
      "positiveInput", std::make_unique<LinearStateInputConstraint>(
                           vector_t::Ones(1), matrix_t::Zero(1, 1), matrix_t::Ones(1, 1)));
  if (terminalUpper > 0.0) {
    problem.finalInequalityConstraintPtr->add(
        "terminalBound", std::make_unique<LinearStateConstraint>(vector_t::Constant(1, terminalUpper), -matrix_t::Ones(1, 1)));
  }
  return problem;
}

ipm::Settings restorationSettings() {
  ipm::Settings settings;
  settings.dt = 1.0;
  settings.ipmIteration = 1;
  settings.nThreads = 1;
  settings.integratorType = SensitivityIntegratorType::RK4;
  settings.initialBarrierParameter = 0.01;
  settings.targetBarrierParameter = 0.01;
  settings.initialSlackMarginRate = 0.0;
  settings.initialDualMarginRate = 0.0;
  settings.restoreFinalNominal = true;
  return settings;
}

std::shared_ptr<ReferenceManager> restorationReference() {
  return std::make_shared<ReferenceManager>(
      TargetTrajectories({0.0}, {vector_t::Zero(1)}, {vector_t::Constant(1, 10.0)}));
}

class NonfiniteTerminalCost final : public StateCost {
 public:
  NonfiniteTerminalCost* clone() const override { return new NonfiniteTerminalCost(*this); }
  scalar_t getValue(scalar_t, const vector_t& state, const TargetTrajectories&, const PreComputation&) const override {
    return state(0) > 3.5 ? std::numeric_limits<scalar_t>::infinity() : 0.0;
  }
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t, const vector_t&, const TargetTrajectories&, const PreComputation&) const override {
    return ScalarFunctionQuadraticApproximation::Zero(1, 0);
  }
};

}  // namespace

TEST(IpmFinalNominalRestoration, RestoresDynamicsAndRecentersFeedbackWithoutChangingRawLog) {
  auto problem = restorationProblem();
  auto settings = restorationSettings();
  DefaultInitializer initializer(1);
  IpmSolver solver(settings, problem, initializer);
  auto reference = restorationReference();
  solver.setReferenceManager(reference);
  solver.run(0.0, vector_t::Ones(1), 1.0);

  const auto result = solver.getFinalNominalRestorationResult();
  ASSERT_TRUE(result.attempted);
  ASSERT_TRUE(result.applied) << result.rejectionReason;
  EXPECT_TRUE(result.rejectionReason.empty());
  ASSERT_EQ(result.maxStateCorrection.size(), 1);
  EXPECT_GT(result.maxStateCorrection(0), 0.0);
  EXPECT_GT(result.originalDynamicsViolationSSE, 1e-8);
  EXPECT_EQ(solver.getIterationsLog().back().dynamicsViolationSSE, result.originalDynamicsViolationSSE);
  EXPECT_LT(solver.getPerformanceIndeces().dynamicsViolationSSE, 1e-20);
  EXPECT_LT(solver.getPerformanceIndeces().equalityConstraintsSSE, 1e-20);

  const auto policy = solver.primalSolution(1.0);
  // RK4 for x'=x+u over one second: x1 = (65*x0 + 41*u0)/24.
  EXPECT_NEAR(policy.stateTrajectory_.back()(0), (65.0 + 41.0 * policy.inputTrajectory_.front()(0)) / 24.0, 1e-12);
  // The terminal input/controller entries are unchanged hold-last placeholders.
  for (size_t i = 0; i + 1 < policy.inputTrajectory_.size(); ++i) {
    EXPECT_TRUE(policy.controllerPtr_->computeInput(policy.timeTrajectory_[i], policy.stateTrajectory_[i])
                    .isApprox(policy.inputTrajectory_[i], 1e-10));
  }
  EXPECT_GT(solver.getDualSolution()->intermediates.front().stateInputIneq.front().lagrangian(0), 0.0);

  // A second solve starts from the coherent restored warm cache; no old status survives.
  solver.run(0.0, vector_t::Ones(1), 1.0);
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().attempted);
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().applied);
  EXPECT_EQ(solver.getFinalNominalRestorationResult().maxStateCorrection.size(), 0);
  solver.reset();
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().attempted);
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().applied);
  EXPECT_EQ(solver.getFinalNominalRestorationResult().originalDynamicsViolationSSE, 0.0);
}

TEST(IpmFinalNominalRestoration, RejectsRolloutOutsideActualTerminalBoundWithoutClipping) {
  auto problem = restorationProblem(1.1);
  auto settings = restorationSettings();
  DefaultInitializer initializer(1);
  IpmSolver solver(settings, problem, initializer);
  solver.setReferenceManager(restorationReference());
  solver.run(0.0, vector_t::Ones(1), 1.0);
  const auto& result = solver.getFinalNominalRestorationResult();
  ASSERT_TRUE(result.attempted);
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.rejectionReason, "nonpositive or nonfinite terminal margin");
  EXPECT_GT(solver.getPerformanceIndeces().dynamicsViolationSSE, 1e-12);
  EXPECT_EQ(solver.getPerformanceIndeces().dynamicsViolationSSE, solver.getIterationsLog().back().dynamicsViolationSSE);
}

TEST(IpmFinalNominalRestoration, RejectsNonfiniteRestoredCost) {
  auto problem = restorationProblem();
  problem.finalCostPtr->add("nonfinite", std::make_unique<NonfiniteTerminalCost>());
  auto settings = restorationSettings();
  DefaultInitializer initializer(1);
  IpmSolver solver(settings, problem, initializer);
  solver.setReferenceManager(restorationReference());
  solver.run(0.0, vector_t::Ones(1), 1.0);
  const auto& result = solver.getFinalNominalRestorationResult();
  ASSERT_TRUE(result.attempted);
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.rejectionReason, "nonfinite cost or merit");
  EXPECT_TRUE(std::isfinite(solver.getPerformanceIndeces().cost));
  EXPECT_GT(solver.getPerformanceIndeces().dynamicsViolationSSE, 1e-12);
}

TEST(IpmFinalNominalRestoration, OptInRejectsUnsupportedModesAndClearsStatusBeforeFailure) {
  auto problem = restorationProblem();
  auto settings = restorationSettings();
  DefaultInitializer initializer(1);
  settings.createValueFunction = true;
  EXPECT_THROW(IpmSolver(settings, problem, initializer), std::invalid_argument);
  settings.createValueFunction = false;
  settings.computeLagrangeMultipliers = true;
  EXPECT_THROW(IpmSolver(settings, problem, initializer), std::invalid_argument);
  settings.restoreFinalNominal = false;
  EXPECT_NO_THROW(IpmSolver(settings, problem, initializer));

  settings = restorationSettings();
  IpmSolver solver(settings, problem, initializer);
  auto reference = restorationReference();
  solver.setReferenceManager(reference);
  solver.run(0.0, vector_t::Ones(1), 1.0);
  ASSERT_TRUE(solver.getFinalNominalRestorationResult().applied);
  reference->setModeSchedule(ModeSchedule({0.5}, {0, 1}));
  EXPECT_THROW(solver.run(0.0, vector_t::Ones(1), 1.0), std::invalid_argument);
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().attempted);
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().applied);
  EXPECT_EQ(solver.getFinalNominalRestorationResult().maxStateCorrection.size(), 0);
}

TEST(IpmFinalNominalRestoration, DefaultOffPreservesUnrestoredCandidate) {
  auto problem = restorationProblem();
  auto settings = restorationSettings();
  settings.restoreFinalNominal = ipm::Settings{}.restoreFinalNominal;
  ASSERT_FALSE(settings.restoreFinalNominal);
  DefaultInitializer initializer(1);
  IpmSolver solver(settings, problem, initializer);
  solver.setReferenceManager(restorationReference());
  solver.run(0.0, vector_t::Ones(1), 1.0);
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().attempted);
  EXPECT_FALSE(solver.getFinalNominalRestorationResult().applied);
  EXPECT_GT(solver.getPerformanceIndeces().dynamicsViolationSSE, 1e-8);
  EXPECT_EQ(solver.getPerformanceIndeces().dynamicsViolationSSE, solver.getIterationsLog().back().dynamicsViolationSSE);
}

}  // namespace ocs2
