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

#include "ocs2_sqp/MultipleShootingSolver.h"
#include "ocs2_sqp/TimeDiscretization.h"

#include <ocs2_core/constraint/LinearConstraint.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include <ocs2_qp_solver/test/testProblemsGeneration.h>

namespace ocs2 {
namespace {
/**
 * A constraint that in mode 0 sets u[0] = 0 and in mode 1 sets u[1] = 1
 */
class SwitchedConstraint : public ocs2::ConstraintBase {
 public:
  explicit SwitchedConstraint(std::shared_ptr<ReferenceManager> referenceManagerPtr)
      : referenceManagerPtr_(std::move(referenceManagerPtr)), subsystemConstraintsPtr_(2) {
    int n = 3;
    int m = 2;
    int nc = 1;

    auto stateInputConstraints0 = VectorFunctionLinearApproximation::Zero(nc, n, m);
    stateInputConstraints0.dfdu << 1.0, 0.0;

    auto stateInputConstraints1 = VectorFunctionLinearApproximation::Zero(nc, n, m);
    stateInputConstraints1.dfdu << 0.0, 1.0;

    subsystemConstraintsPtr_[0] =
        qp_solver::getOcs2Constraints(stateInputConstraints0, VectorFunctionLinearApproximation(), VectorFunctionLinearApproximation());
    subsystemConstraintsPtr_[1] =
        qp_solver::getOcs2Constraints(stateInputConstraints1, VectorFunctionLinearApproximation(), VectorFunctionLinearApproximation());
  }

  ~SwitchedConstraint() override = default;

  SwitchedConstraint(const SwitchedConstraint& other)
      : ocs2::ConstraintBase(other), referenceManagerPtr_(other.referenceManagerPtr_) {
    for (const auto& constraint : other.subsystemConstraintsPtr_) {
      subsystemConstraintsPtr_.emplace_back(constraint->clone());
    }
  }

  SwitchedConstraint* clone() const override { return new SwitchedConstraint(*this); }

  vector_t stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemConstraintsPtr_[activeMode]->stateInputEqualityConstraint(t, x, u);
  }

  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                    const vector_t& u) override {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemConstraintsPtr_[activeMode]->stateInputEqualityConstraintLinearApproximation(t, x, u);
  }

 public:
  std::vector<std::unique_ptr<ConstraintBase>> subsystemConstraintsPtr_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
};

std::pair<PrimalSolution, std::vector<PerformanceIndex>> solveWithEventTime(scalar_t eventTime) {
  int n = 3;
  int m = 2;

  // System
  const auto dynamics = ocs2::qp_solver::getRandomDynamics(n, m);
  const auto jumpMap = matrix_t::Random(n, n);
  std::unique_ptr<ocs2::LinearSystemDynamics> systemPtr(new ocs2::LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu, jumpMap));

  // Cost
  auto costPtr = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(n, m), ocs2::qp_solver::getRandomCost(n, 0));
  ocs2::TargetTrajectories targetTrajectories({0.0}, {ocs2::vector_t::Random(n)}, {ocs2::vector_t::Random(m)});
  costPtr->setTargetTrajectoriesPtr(&targetTrajectories);

  // Constraint
  const ocs2::scalar_array_t eventTimes{eventTime};
  const std::vector<size_t> subsystemsSequence{0, 1};
  std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr(new ocs2::ReferenceManager({eventTimes, subsystemsSequence}));
  ocs2::SwitchedConstraint switchedConstraint(referenceManagerPtr);

  // Solver settings
  ocs2::multiple_shooting::Settings settings;
  settings.dt = 0.05;
  settings.sqpIteration = 20;
  settings.projectStateInputEqualityConstraints = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = ocs2::vector_t::Random(n);
  const ocs2::scalar_array_t partitioningTimes{0.0};
  ocs2::DefaultInitializer zeroInitializer(m);

  // Set up solver
  ocs2::MultipleShootingSolver solver(settings, systemPtr.get(), costPtr.get(), &zeroInitializer, &switchedConstraint);
  solver.setReferenceManager(referenceManagerPtr);
  solver.setTargetTrajectories(targetTrajectories);

  // Solve
  solver.run(startTime, initState, finalTime, partitioningTimes);
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
  ASSERT_LT(performanceLog.back().stateEqConstraintISE, tol);

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
  ASSERT_LT(performanceLog.back().stateEqConstraintISE, tol);

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
  ASSERT_LT(performanceLog.back().stateEqConstraintISE, tol);

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
