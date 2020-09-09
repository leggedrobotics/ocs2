/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_qp_solver/Ocs2QpSolver.h>
#include <ocs2_qp_solver/test/testProblemsGeneration.h>

#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_oc/rollout/PerformanceIndicesRollout.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

class CorrectnessUnconstrainedTest : public testing::Test {
 protected:
  static constexpr size_t N = 50;
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr ocs2::scalar_t solutionPrecision = 2e-3;

  CorrectnessUnconstrainedTest() {
    srand(0);
    // dynamics
    systemPtr = ocs2::qp_solver::getOcs2Dynamics(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));

    // cost
    costPtr = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                        ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM));
    costDesiredTrajectories =
        ocs2::CostDesiredTrajectories({0.0}, {ocs2::vector_t::Random(STATE_DIM)}, {ocs2::vector_t::Random(INPUT_DIM)});
    costPtr->setCostDesiredTrajectoriesPtr(&costDesiredTrajectories);

    // constraint
    constraintPtr.reset(new ocs2::ConstraintBase());

    // system operating points
    nominalTrajectory = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, 1e-3);
    ocs2::vector_array_t stateTrajectoryTemp(N + 1);
    std::copy(nominalTrajectory.stateTrajectory.begin(), nominalTrajectory.stateTrajectory.end(), stateTrajectoryTemp.begin());
    ocs2::vector_array_t inputTrajectoryTemp(N);
    std::copy(nominalTrajectory.inputTrajectory.begin(), nominalTrajectory.inputTrajectory.end(), inputTrajectoryTemp.begin());
    inputTrajectoryTemp.emplace_back(inputTrajectoryTemp.back());
    operatingPointsPtr.reset(new ocs2::OperatingPoints(nominalTrajectory.timeTrajectory, stateTrajectoryTemp, inputTrajectoryTemp));

    initState = ocs2::vector_t::Random(STATE_DIM);
    qpSolution = ocs2::qp_solver::solveLinearQuadraticOptimalControlProblem(*costPtr, *systemPtr, nominalTrajectory, initState);
    qpCost = getQpCost(qpSolution);

    // rollout settings
    const ocs2::rollout::Settings rolloutSettings = []() {
      ocs2::rollout::Settings rolloutSettings;
      rolloutSettings.absTolODE_ = 1e-10;
      rolloutSettings.relTolODE_ = 1e-7;
      rolloutSettings.maxNumStepsPerSecond_ = 10000;
      return rolloutSettings;
    }();

    // rollout
    rolloutPtr.reset(new ocs2::TimeTriggeredRollout(*systemPtr, rolloutSettings));

    startTime = nominalTrajectory.timeTrajectory.front();
    finalTime = nominalTrajectory.timeTrajectory.back();
  }

  ocs2::ddp::Settings getSettings(ocs2::ddp::algorithm algorithmType, size_t numPartitions,
                                  ocs2::ddp_strategy::type strategy, bool display = false) const {
    ocs2::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.minRelCost_ = 1e-3;
    ddpSettings.useNominalTimeForBackwardPass_ = false;
    ddpSettings.nThreads_ = numPartitions;
    ddpSettings.maxNumIterations_ = 2 + (numPartitions - 1);  // need an extra iteration for each added time partition
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength_ = 1e-4;
    return ddpSettings;
  }

  ocs2::scalar_t getQpCost(const ocs2::qp_solver::ContinuousTrajectory& qpSolution) const {
    auto costFunc = [this](ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) { return costPtr->cost(t, x, u); };
    auto inputTrajectoryTemp = qpSolution.inputTrajectory;
    inputTrajectoryTemp.emplace_back(inputTrajectoryTemp.back());
    auto lAccum =
        ocs2::PerformanceIndicesRollout::rolloutCost(costFunc, qpSolution.timeTrajectory, qpSolution.stateTrajectory, inputTrajectoryTemp);

    return lAccum + costPtr->finalCost(qpSolution.timeTrajectory.back(), qpSolution.stateTrajectory.back());
  }

  std::string getTestName(const ocs2::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "Correctness Test { ";
    testName += "Algorithm: " + ocs2::ddp::toAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + ocs2::ddp_strategy::toString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void correctnessTest(const ocs2::ddp::Settings& ddpSettings, const ocs2::PerformanceIndex& performanceIndex,
                       const ocs2::PrimalSolution& ddpSolution) const {
    const auto testName = getTestName(ddpSettings);
    ASSERT_LT(fabs(performanceIndex.totalCost - qpCost), 10 * ddpSettings.minRelCost_)
        << "MESSAGE: " << testName << ": failed in the optimal cost test!";
    EXPECT_LT(relError(ddpSolution.stateTrajectory_.back(), qpSolution.stateTrajectory.back()), solutionPrecision)
       << "MESSAGE: " << testName << ": failed in the optimal final state test!";
    EXPECT_LT(relError(ddpSolution.inputTrajectory_.front(), qpSolution.inputTrajectory.front()), solutionPrecision)
       << "MESSAGE: " << testName << ": failed in the optimal initial input test!";
  }

  ocs2::scalar_t relError(ocs2::vector_t ddpSol, const ocs2::vector_t& qpSol) const {
    return (ddpSol - qpSol).norm() / ddpSol.norm();
  }

  ocs2::vector_t initState;
  ocs2::scalar_t startTime;
  ocs2::scalar_t finalTime;

  std::unique_ptr<ocs2::CostFunctionBase> costPtr;
  ocs2::CostDesiredTrajectories costDesiredTrajectories;
  std::unique_ptr<ocs2::SystemDynamicsBase> systemPtr;
  std::unique_ptr<ocs2::ConstraintBase> constraintPtr;
  std::unique_ptr<ocs2::OperatingPoints> operatingPointsPtr;
  ocs2::qp_solver::ContinuousTrajectory nominalTrajectory;
  std::unique_ptr<ocs2::TimeTriggeredRollout> rolloutPtr;

  ocs2::scalar_t qpCost;
  ocs2::qp_solver::ContinuousTrajectory qpSolution;
};

constexpr size_t CorrectnessUnconstrainedTest::N;
constexpr size_t CorrectnessUnconstrainedTest::STATE_DIM;
constexpr size_t CorrectnessUnconstrainedTest::INPUT_DIM;
constexpr ocs2::scalar_t CorrectnessUnconstrainedTest::solutionPrecision;


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(CorrectnessUnconstrainedTest, slq_single_partition_linesearch) {
  // settings
  ocs2::scalar_array_t partitioningTimes{startTime, finalTime};
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, partitioningTimes.size() - 1, ocs2::ddp_strategy::type::LINE_SEARCH);

  // ddp
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setCostDesiredTrajectories(costDesiredTrajectories);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto performanceIndex = ddp.getPerformanceIndeces();
  const auto solution = ddp.primalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(CorrectnessUnconstrainedTest, slq_multi_partition_linesearch) {
  // settings
  ocs2::scalar_array_t partitioningTimes{startTime, (startTime + finalTime) / 2.0, finalTime};
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, partitioningTimes.size() - 1, ocs2::ddp_strategy::type::LINE_SEARCH);

  // ddp
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setCostDesiredTrajectories(costDesiredTrajectories);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto performanceIndex = ddp.getPerformanceIndeces();
  const auto solution = ddp.primalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(CorrectnessUnconstrainedTest, ilqr_single_partition_linesearch) {
  // settings
  ocs2::scalar_array_t partitioningTimes{startTime, finalTime};
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::ILQR, partitioningTimes.size() - 1, ocs2::ddp_strategy::type::LINE_SEARCH);

  // ddp
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setCostDesiredTrajectories(costDesiredTrajectories);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto performanceIndex = ddp.getPerformanceIndeces();
  const auto solution = ddp.primalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(CorrectnessUnconstrainedTest, ilqr_multi_partition_linesearch) {
  // settings
  ocs2::scalar_array_t partitioningTimes{startTime, (startTime + finalTime) / 2.0, finalTime};
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::ILQR, partitioningTimes.size() - 1, ocs2::ddp_strategy::type::LINE_SEARCH);

  // ddp
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setCostDesiredTrajectories(costDesiredTrajectories);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto performanceIndex = ddp.getPerformanceIndeces();
  const auto solution = ddp.primalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}
