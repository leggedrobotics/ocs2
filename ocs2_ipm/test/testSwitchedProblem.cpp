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

#include "ocs2_ipm/IpmSolver.h"

#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/oc_data/TimeDiscretization.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_oc/test/testProblemsGeneration.h>

namespace ocs2 {
namespace {
/**
 * A constraint that in mode 0 sets u[0] = 0 and in mode 1 sets u[1] = 1
 */
class SwitchedConstraint : public StateInputConstraint {
 public:
  explicit SwitchedConstraint(std::shared_ptr<ReferenceManager> referenceManagerPtr)
      : StateInputConstraint(ConstraintOrder::Linear), referenceManagerPtr_(std::move(referenceManagerPtr)) {
    constexpr int n = 3;
    constexpr int m = 2;
    constexpr int nc = 1;

    auto stateInputConstraints0 = VectorFunctionLinearApproximation::Zero(nc, n, m);
    stateInputConstraints0.dfdu << 1.0, 0.0;

    auto stateInputConstraints1 = VectorFunctionLinearApproximation::Zero(nc, n, m);
    stateInputConstraints1.dfdu << 0.0, 1.0;

    subsystemConstraintsPtr_.emplace_back(getOcs2Constraints(stateInputConstraints0));
    subsystemConstraintsPtr_.emplace_back(getOcs2Constraints(stateInputConstraints1));
  }

  ~SwitchedConstraint() override = default;
  SwitchedConstraint* clone() const override { return new SwitchedConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const override {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(time);
    return subsystemConstraintsPtr_[activeMode]->getNumConstraints(time);
  }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(time);
    return subsystemConstraintsPtr_[activeMode]->getValue(time, state, input, preComp);
  };

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(time);
    return subsystemConstraintsPtr_[activeMode]->getLinearApproximation(time, state, input, preComp);
  }

 private:
  SwitchedConstraint(const SwitchedConstraint& other)
      : ocs2::StateInputConstraint(other), referenceManagerPtr_(other.referenceManagerPtr_) {
    for (const auto& constraint : other.subsystemConstraintsPtr_) {
      subsystemConstraintsPtr_.emplace_back(constraint->clone());
    }
  }

  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
  std::vector<std::unique_ptr<ocs2::StateInputConstraint>> subsystemConstraintsPtr_;
};

std::pair<PrimalSolution, std::vector<PerformanceIndex>> solveWithEventTime(scalar_t eventTime) {
  constexpr int n = 3;
  constexpr int m = 2;

  ocs2::OptimalControlProblem problem;

  // System
  const auto dynamics = ocs2::getRandomDynamics(n, m);
  const auto jumpMap = matrix_t::Random(n, n);
  problem.dynamicsPtr.reset(new ocs2::LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu, jumpMap));

  // Cost
  problem.costPtr->add("intermediateCost", ocs2::getOcs2Cost(ocs2::getRandomCost(n, m)));
  problem.softConstraintPtr->add("intermediateCost", ocs2::getOcs2Cost(ocs2::getRandomCost(n, m)));
  problem.preJumpCostPtr->add("eventCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(n, 0)));
  problem.preJumpSoftConstraintPtr->add("eventCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(n, 0)));
  problem.finalCostPtr->add("finalCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(n, 0)));
  problem.finalSoftConstraintPtr->add("finalCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(n, 0)));

  // Reference Manager
  const ocs2::ModeSchedule modeSchedule({eventTime}, {0, 1});
  const ocs2::TargetTrajectories targetTrajectories({0.0}, {ocs2::vector_t::Random(n)}, {ocs2::vector_t::Random(m)});
  auto referenceManagerPtr = std::make_shared<ocs2::ReferenceManager>(targetTrajectories, modeSchedule);

  problem.targetTrajectoriesPtr = &targetTrajectories;

  // Constraint
  problem.equalityConstraintPtr->add("switchedConstraint", std::make_unique<SwitchedConstraint>(referenceManagerPtr));

  ocs2::DefaultInitializer zeroInitializer(m);

  // Solver settings
  ocs2::ipm::Settings settings;
  settings.dt = 0.05;
  settings.ipmIteration = 20;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = ocs2::vector_t::Random(n);

  // Set up solver
  ocs2::IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);

  // Solve
  solver.run(startTime, initState, finalTime);
  return {solver.primalSolution(finalTime), solver.getIterationsLog()};
}

}  // namespace
}  // namespace ocs2

TEST(test_switched_problem, switched_constraint) {
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::scalar_t eventTime = 0.1875;
  const double tol = 1e-9;
  const auto solution = ocs2::solveWithEventTime(eventTime);
  const auto& primalSolution = solution.first;
  const auto& performanceLog = solution.second;

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(performanceLog.size(), 2);
  ASSERT_LT(performanceLog.back().dynamicsViolationSSE, tol);

  // Should have correct node pre and post event time, with corresponding inputs
  ASSERT_EQ(primalSolution.timeTrajectory_[4], eventTime);
  ASSERT_EQ(primalSolution.timeTrajectory_[5], eventTime);
  ASSERT_LT(std::abs(primalSolution.inputTrajectory_[4][0]), tol);
  ASSERT_LT(std::abs(primalSolution.inputTrajectory_[5][1]), tol);

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  ocs2::scalar_t t_check = startTime;
  ocs2::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    ocs2::vector_t xNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    ocs2::vector_t uNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    ocs2::vector_t uControl = primalSolution.controllerPtr_->computeInput(t_check, xNominal);
    if (t_check < eventTime) {
      ASSERT_LT(std::abs(uNominal[0]), tol);
      ASSERT_LT(std::abs(uControl[0]), tol);
    } else if (t_check > eventTime) {
      ASSERT_LT(std::abs(uNominal[1]), tol);
      ASSERT_LT(std::abs(uControl[1]), tol);
    }
    t_check += dt_check;
  }
}

TEST(test_switched_problem, event_at_beginning) {
  // The event should replace the start time, all inputs should be after the event.
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::scalar_t eventTime = 1e-8;
  const double tol = 1e-9;
  const auto solution = ocs2::solveWithEventTime(eventTime);
  const auto& primalSolution = solution.first;
  const auto& performanceLog = solution.second;

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(performanceLog.size(), 2);
  ASSERT_LT(performanceLog.back().dynamicsViolationSSE, tol);

  // Should have correct post event time start
  ASSERT_EQ(primalSolution.timeTrajectory_[0], eventTime);
  ASSERT_NE(primalSolution.timeTrajectory_[1], eventTime);

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  ocs2::scalar_t t_check = eventTime;
  ocs2::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    ocs2::vector_t xNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    ocs2::vector_t uNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    ocs2::vector_t uControl = primalSolution.controllerPtr_->computeInput(t_check, xNominal);
    ASSERT_LT(std::abs(uNominal[1]), tol);
    ASSERT_LT(std::abs(uControl[1]), tol);
    t_check += dt_check;
  }
}

TEST(test_switched_problem, event_at_end) {
  // The event should be ignored because its too close to the final time, all inputs should be before the event.
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::scalar_t eventTime = 1.0 - 1e-8;
  const double tol = 1e-9;
  const auto solution = ocs2::solveWithEventTime(eventTime);
  const auto& primalSolution = solution.first;
  const auto& performanceLog = solution.second;

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(performanceLog.size(), 2);
  ASSERT_LT(performanceLog.back().dynamicsViolationSSE, tol);

  // Should have correct node pre and post event time
  ASSERT_TRUE(std::none_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                           [=](ocs2::scalar_t t) { return t == eventTime; }));

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  ocs2::scalar_t t_check = startTime;
  ocs2::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    ocs2::vector_t xNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    ocs2::vector_t uNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    ocs2::vector_t uControl = primalSolution.controllerPtr_->computeInput(t_check, xNominal);
    ASSERT_LT(std::abs(uNominal[0]), tol);
    ASSERT_LT(std::abs(uControl[0]), tol);
    t_check += dt_check;
  }
}
