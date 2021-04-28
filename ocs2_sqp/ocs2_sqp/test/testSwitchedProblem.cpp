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
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include <ocs2_qp_solver/test/testProblemsGeneration.h>

namespace ocs2 {
namespace {
/**
 * A constraint that in mode 0 sets u[0] = 0 and in mode 1 sets u[1] = 1
 */
class SwitchedConstraint : public ocs2::ConstraintBase {
 public:
  explicit SwitchedConstraint(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemConstraintsPtr_(2) {
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
      : ocs2::ConstraintBase(other), modeScheduleManagerPtr_(other.modeScheduleManagerPtr_) {
    for (const auto& constraint : other.subsystemConstraintsPtr_) {
      subsystemConstraintsPtr_.emplace_back(constraint->clone());
    }
  }

  SwitchedConstraint* clone() const override { return new SwitchedConstraint(*this); }

  vector_t stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemConstraintsPtr_[activeMode]->stateInputEqualityConstraint(t, x, u);
  }

  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                    const vector_t& u) override {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemConstraintsPtr_[activeMode]->stateInputEqualityConstraintLinearApproximation(t, x, u);
  }

 public:
  std::vector<std::unique_ptr<ConstraintBase>> subsystemConstraintsPtr_;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
};

PrimalSolution solveWithEventTime(scalar_t eventTime) {
  int n = 3;
  int m = 2;
  int nc = 1;

  // System
  auto systemPtr = ocs2::qp_solver::getOcs2Dynamics(ocs2::qp_solver::getRandomDynamics(n, m));

  // Cost
  auto costPtr = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(n, m), ocs2::qp_solver::getRandomCost(n, 0));
  ocs2::CostDesiredTrajectories costDesiredTrajectories({0.0}, {ocs2::vector_t::Random(n)}, {ocs2::vector_t::Random(m)});
  costPtr->setCostDesiredTrajectoriesPtr(&costDesiredTrajectories);

  // Constraint
  const ocs2::scalar_array_t eventTimes{eventTime};
  const std::vector<size_t> subsystemsSequence{0, 1};
  std::shared_ptr<ocs2::ModeScheduleManager> modeScheduleManagerPtr(new ocs2::ModeScheduleManager({eventTimes, subsystemsSequence}));
  ocs2::SwitchedConstraint switchedConstraint(modeScheduleManagerPtr);

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
  ocs2::OperatingPoints operatingPoints(initState, ocs2::vector_t::Zero(m));

  // Set up solver
  ocs2::MultipleShootingSolver solver(settings, systemPtr.get(), costPtr.get(), &operatingPoints, &switchedConstraint);
  solver.setModeScheduleManager(modeScheduleManagerPtr);
  solver.setCostDesiredTrajectories(costDesiredTrajectories);

  // Solve
  solver.run(startTime, initState, finalTime, partitioningTimes);
  return solver.primalSolution(finalTime);
}

}  // namespace
}  // namespace ocs2

TEST(test_switched_problem, switched_constraint) {
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::scalar_t evenTime = 0.1875;
  const auto primalSolution = ocs2::solveWithEventTime(evenTime);

  // Should have correct node pre and post event time
  const auto preEventTime = ocs2::getIntervalEnd({evenTime, true});
  const auto postEventTime = ocs2::getIntervalStart({evenTime, true});
  ASSERT_TRUE(std::any_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                          [=](ocs2::scalar_t t) { return t == preEventTime; }));
  ASSERT_TRUE(std::any_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                          [=](ocs2::scalar_t t) { return t == postEventTime; }));

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  ocs2::scalar_t t_check = startTime;
  ocs2::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    ocs2::vector_t xNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    ocs2::vector_t uNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    ocs2::vector_t uControl = primalSolution.controllerPtr_->computeInput(t_check, xNominal);
    if (t_check < preEventTime) {
      ASSERT_LT(std::abs(uNominal[0]), 1e-9);
      ASSERT_LT(std::abs(uControl[0]), 1e-9);
    } else if (t_check > postEventTime) {
      ASSERT_LT(std::abs(uNominal[1]), 1e-9);
      ASSERT_LT(std::abs(uControl[1]), 1e-9);
    }
    t_check += dt_check;
  }
}

TEST(test_switched_problem, event_at_beginning) {
  // The event should replace the start time, all inputs should be after the event.
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::scalar_t evenTime = 1e-8;
  const auto primalSolution = ocs2::solveWithEventTime(evenTime);

  // Should have correct node pre and post event time
  const auto preEventTime = ocs2::getIntervalEnd({evenTime, true});
  const auto postEventTime = ocs2::getIntervalStart({evenTime, true});
  ASSERT_TRUE(std::none_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                           [=](ocs2::scalar_t t) { return t == preEventTime; }));
  ASSERT_TRUE(std::any_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                          [=](ocs2::scalar_t t) { return t == postEventTime; }));

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  ocs2::scalar_t t_check = postEventTime;
  ocs2::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    ocs2::vector_t xNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    ocs2::vector_t uNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    ocs2::vector_t uControl = primalSolution.controllerPtr_->computeInput(t_check, xNominal);
    ASSERT_LT(std::abs(uNominal[1]), 1e-9);
    ASSERT_LT(std::abs(uControl[1]), 1e-9);
    t_check += dt_check;
  }
}

TEST(test_switched_problem, event_at_end) {
  // The event should be ignored because its too close to the final time, all inputs should be before the event.
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::scalar_t evenTime = 1.0 - 1e-8;
  const auto primalSolution = ocs2::solveWithEventTime(evenTime);

  // Should have correct node pre and post event time
  const auto preEventTime = ocs2::getIntervalEnd({evenTime, true});
  const auto postEventTime = ocs2::getIntervalStart({evenTime, true});
  ASSERT_TRUE(std::none_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                           [=](ocs2::scalar_t t) { return t == preEventTime; }));
  ASSERT_TRUE(std::none_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                           [=](ocs2::scalar_t t) { return t == postEventTime; }));

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  ocs2::scalar_t t_check = startTime;
  ocs2::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    ocs2::vector_t xNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    ocs2::vector_t uNominal =
        ocs2::LinearInterpolation::interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    ocs2::vector_t uControl = primalSolution.controllerPtr_->computeInput(t_check, xNominal);
    ASSERT_LT(std::abs(uNominal[0]), 1e-9);
    ASSERT_LT(std::abs(uControl[0]), 1e-9);
    t_check += dt_check;
  }
}
