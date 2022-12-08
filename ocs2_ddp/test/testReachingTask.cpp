/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/DoubleIntegratorReachingTask.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

namespace ocs2 {

namespace {
// rollout settings
rollout::Settings getRolloutSettings(ddp::Algorithm alg) {
  rollout::Settings s;
  s.absTolODE = 1e-9;
  s.relTolODE = 1e-6;
  s.timeStep = DoubleIntegratorReachingTask::timeStep;
  s.integratorType = (alg == ddp::Algorithm::SLQ) ? IntegratorType::ODE45 : IntegratorType::RK4;
  s.maxNumStepsPerSecond = 100000;
  s.checkNumericalStability = true;
  return s;
}

// DDP settings
ddp::Settings getDdpSettings(ddp::Algorithm alg, bool display) {
  ddp::Settings s;
  s.algorithm_ = alg;
  s.nThreads_ = 2;
  s.maxNumIterations_ = 50;
  s.displayInfo_ = false;
  s.displayShortSummary_ = display;
  s.debugPrintRollout_ = false;
  s.minRelCost_ = DoubleIntegratorReachingTask::minRelCost;
  s.constraintTolerance_ = DoubleIntegratorReachingTask::constraintTolerance;
  s.timeStep_ = DoubleIntegratorReachingTask::timeStep;
  s.backwardPassIntegratorType_ = (alg == ddp::Algorithm::SLQ) ? IntegratorType::ODE45 : IntegratorType::RK4;
  s.strategy_ = search_strategy::Type::LINE_SEARCH;
  s.lineSearch_.minStepLength = 0.01;
  s.lineSearch_.maxStepLength = 1.0;
  return s;
}
}  // unnamed namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/* Reaching task at event time (tGoal) for a problem with horizon 2*tGoal. The solution input should be zero in [tGoal, 2*tGoal]. */
class PreJumpDoubleIntegratorReachingTask : public DoubleIntegratorReachingTask,
                                            public testing::Test,
                                            public testing::WithParamInterface<DoubleIntegratorReachingTask::PenaltyType> {
 protected:
  PreJumpDoubleIntegratorReachingTask() {
    // reference manager
    referenceManagerPtr = getReferenceManagerPtr();
    // optimal control problem
    ocp.dynamicsPtr = getDynamicsPtr();
    ocp.costPtr->add("cost", getCostPtr());
    ocp.equalityConstraintPtr->add("zero_force", std::make_unique<ZeroInputConstraint>(*referenceManagerPtr));
    ocp.preJumpEqualityLagrangianPtr->add("goal_reaching", getGoalReachingAugmentedLagrangian(xGoal, GetParam()));
  }

  void test(const PrimalSolution& primalSolution) const {
    for (size_t i = 0; i < primalSolution.timeTrajectory_.size(); ++i) {
      // zero input after tGoal
      if (primalSolution.timeTrajectory_[i] > tGoal) {
        EXPECT_NEAR(primalSolution.inputTrajectory_[i](0), 0.0, constraintTolerance) << " at time " << primalSolution.timeTrajectory_[i];
      }
      // reaching to the target
      if (primalSolution.timeTrajectory_[i] > tGoal) {
        EXPECT_TRUE(primalSolution.stateTrajectory_[i].isApprox(xGoal, constraintTolerance))
            << " at time " << primalSolution.timeTrajectory_[i];
      }
    }  // end of i loop
  }

  OptimalControlProblem ocp;
  std::shared_ptr<ReferenceManager> referenceManagerPtr;
};

TEST_P(PreJumpDoubleIntegratorReachingTask, SLQ) {
  constexpr bool display = true;
  constexpr auto alg = ddp::Algorithm::SLQ;
  // rollout
  TimeTriggeredRollout rollout(*ocp.dynamicsPtr, getRolloutSettings(alg));
  // solver
  SLQ ddp(getDdpSettings(alg, display), rollout, ocp, *getInitializer());
  ddp.setReferenceManager(referenceManagerPtr);
  // run solver
  ddp.run(0.0, xInit, 2.0 * tGoal);
  PrimalSolution primalSolution;
  ddp.getPrimalSolution(2.0 * tGoal, &primalSolution);
  // test
  test(primalSolution);
  printSolution(primalSolution, display);
}

INSTANTIATE_TEST_CASE_P(PreJumpDoubleIntegratorReachingTaskCase, PreJumpDoubleIntegratorReachingTask,
                        testing::ValuesIn({DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty,
                                           DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty}),
                        [](const testing::TestParamInfo<PreJumpDoubleIntegratorReachingTask::ParamType>& info) {
                          if (info.param == DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty) {
                            return std::string("QuadraticPenalty");
                          } else if (info.param == DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty) {
                            return std::string("SmoothAbsolutePenalty");
                          }
                        });

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/* Reaching task at final time for a problem with horizon tGoal. */
class FinalDoubleIntegratorReachingTask : public DoubleIntegratorReachingTask,
                                          public testing::Test,
                                          public testing::WithParamInterface<DoubleIntegratorReachingTask::PenaltyType> {
 protected:
  FinalDoubleIntegratorReachingTask() {
    // reference manager
    referenceManagerPtr = getReferenceManagerPtr();
    // optimal control problem
    ocp.dynamicsPtr = getDynamicsPtr();
    ocp.costPtr->add("cost", getCostPtr());
    ocp.finalEqualityLagrangianPtr->add("goal_reaching", getGoalReachingAugmentedLagrangian(xGoal, GetParam()));
  }

  void test(const PrimalSolution& primalSolution) const {
    // reaching to the target
    EXPECT_TRUE(primalSolution.stateTrajectory_.back().isApprox(xGoal, constraintTolerance))
        << " at time " << primalSolution.timeTrajectory_.back();
  }

  OptimalControlProblem ocp;
  std::shared_ptr<ReferenceManager> referenceManagerPtr;
};

TEST_P(FinalDoubleIntegratorReachingTask, SLQ) {
  constexpr bool display = true;
  constexpr auto alg = ddp::Algorithm::SLQ;
  // rollout
  TimeTriggeredRollout rollout(*ocp.dynamicsPtr, getRolloutSettings(alg));
  // solver
  SLQ ddp(getDdpSettings(alg, display), rollout, ocp, *getInitializer());
  ddp.setReferenceManager(referenceManagerPtr);
  // run solver
  ddp.run(0.0, xInit, tGoal);
  PrimalSolution primalSolution;
  ddp.getPrimalSolution(tGoal, &primalSolution);
  // test
  test(primalSolution);
  printSolution(primalSolution, display);
}

INSTANTIATE_TEST_CASE_P(FinalDoubleIntegratorReachingTaskCase, FinalDoubleIntegratorReachingTask,
                        testing::ValuesIn({DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty,
                                           DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty}),
                        [](const testing::TestParamInfo<PreJumpDoubleIntegratorReachingTask::ParamType>& info) {
                          if (info.param == DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty) {
                            return std::string("QuadraticPenalty");
                          } else if (info.param == DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty) {
                            return std::string("SmoothAbsolutePenalty");
                          } else {
                            throw std::runtime_error("Undefined PenaltyTyp!");
                          }
                        });

}  // namespace ocs2
