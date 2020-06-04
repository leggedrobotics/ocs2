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

class DdpCorrectnessTest : public testing::Test {
 protected:
  static constexpr size_t N = 50;
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr double solutionPrecision = 1e-3;

  DdpCorrectnessTest() {
    srand(0);
    // dynamics
    system = ocs2::qp_solver::getOcs2Dynamics(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));

    // cost
    cost = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                        ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), ocs2::vector_t::Random(STATE_DIM),
                                        ocs2::vector_t::Random(INPUT_DIM), ocs2::vector_t::Random(STATE_DIM));

    // constraint
    constraint.reset(new ocs2::ConstraintBase(STATE_DIM, INPUT_DIM));

    // system operating points
    nominalTrajectory = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, 1e-3);
    ocs2::vector_array_t stateTrajectoryTemp(N + 1);
    std::copy(nominalTrajectory.stateTrajectory.begin(), nominalTrajectory.stateTrajectory.end(), stateTrajectoryTemp.begin());
    ocs2::vector_array_t inputTrajectoryTemp(N);
    std::copy(nominalTrajectory.inputTrajectory.begin(), nominalTrajectory.inputTrajectory.end(), inputTrajectoryTemp.begin());
    inputTrajectoryTemp.emplace_back(inputTrajectoryTemp.back());
    operatingPoints.reset(new ocs2::OperatingPoints(nominalTrajectory.timeTrajectory, stateTrajectoryTemp, inputTrajectoryTemp));

    initState = ocs2::vector_t::Random(STATE_DIM);
    qpSolution = ocs2::qp_solver::solveLinearQuadraticOptimalControlProblem(*cost, *system, nominalTrajectory, initState);
    qpCost = getQpCost(qpSolution);

    // rollout settings
    const ocs2::Rollout_Settings rolloutSettings = []() {
      ocs2::Rollout_Settings rolloutSettings;
      rolloutSettings.absTolODE_ = 1e-10;
      rolloutSettings.relTolODE_ = 1e-7;
      rolloutSettings.maxNumStepsPerSecond_ = 10000;
      return rolloutSettings;
    }();

    // system rollout
    rollout.reset(new ocs2::TimeTriggeredRollout(STATE_DIM, INPUT_DIM, *system, rolloutSettings));

    startTime = nominalTrajectory.timeTrajectory.front();
    finalTime = nominalTrajectory.timeTrajectory.back();
  }

  ocs2::DDP_Settings ddpSettings(int numPartitions) const {
    ocs2::DDP_Settings ddpSettings;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = false;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.lineSearch_.minStepLength_ = 0.0001;
    ddpSettings.minRelCost_ = 5e-4;
    ddpSettings.nThreads_ = numPartitions;
    ddpSettings.maxNumIterations_ = 2 + (numPartitions - 1);  // need an extra iteration for each added time partition
    return ddpSettings;
  }

  ocs2::scalar_t getQpCost(const ocs2::qp_solver::ContinuousTrajectory& qpSolution) const {
    auto costFunc = [this](ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) {
      cost->setCurrentStateAndControl(t, x, u);
      return cost->getCost();
    };
    auto inputTrajectoryTemp = qpSolution.inputTrajectory;
    inputTrajectoryTemp.emplace_back(inputTrajectoryTemp.back());
    auto lAccum =
        ocs2::PerformanceIndicesRollout::rolloutCost(costFunc, qpSolution.timeTrajectory, qpSolution.stateTrajectory, inputTrajectoryTemp);

    cost->setCurrentStateAndControl(qpSolution.timeTrajectory.back(), qpSolution.stateTrajectory.back(), ocs2::vector_t::Zero(INPUT_DIM));
    return lAccum + cost->getTerminalCost();
  }

  ocs2::scalar_t relError(ocs2::vector_t ddpSol, const ocs2::vector_t& qpSol) const { return (ddpSol - qpSol).norm() / ddpSol.norm(); }

  std::unique_ptr<ocs2::CostFunctionBase> cost;
  std::unique_ptr<ocs2::SystemDynamicsBase> system;
  std::unique_ptr<ocs2::ConstraintBase> constraint;
  std::unique_ptr<ocs2::OperatingPoints> operatingPoints;
  ocs2::qp_solver::ContinuousTrajectory nominalTrajectory;
  std::unique_ptr<ocs2::TimeTriggeredRollout> rollout;

  ocs2::scalar_t qpCost;
  ocs2::qp_solver::ContinuousTrajectory qpSolution;

  ocs2::vector_t initState;
  ocs2::scalar_t startTime;
  ocs2::scalar_t finalTime;
};

constexpr size_t DdpCorrectnessTest::N;
constexpr size_t DdpCorrectnessTest::STATE_DIM;
constexpr size_t DdpCorrectnessTest::INPUT_DIM;
constexpr double DdpCorrectnessTest::solutionPrecision;

TEST_F(DdpCorrectnessTest, slq_solution_single_partition) {
  ocs2::scalar_array_t partitioningTimes{startTime, finalTime};

  ocs2::SLQ_Settings settings;
  settings.useNominalTimeForBackwardPass_ = false;
  settings.ddpSettings_ = ddpSettings(partitioningTimes.size() - 1);

  // DDP
  ocs2::SLQ ddp(STATE_DIM, INPUT_DIM, rollout.get(), system.get(), constraint.get(), cost.get(), operatingPoints.get(), settings);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto ddpSolution = ddp.primalSolution(finalTime);
  const auto ddpPerformanceIndeces = ddp.getPerformanceIndeces();

  ASSERT_LT((ddpPerformanceIndeces.totalCost - qpCost), 10.0 * settings.ddpSettings_.minRelCost_)
      << "MESSAGE: SLQ failed in the optimal cost test!";

  ASSERT_LT(relError(ddpSolution.inputTrajectory_.front(), qpSolution.inputTrajectory.front()), solutionPrecision)
      << "MESSAGE: SLQ failed in the optimal initial input test!";

  ASSERT_LT(relError(ddpSolution.stateTrajectory_.back(), qpSolution.stateTrajectory.back()), solutionPrecision)
      << "MESSAGE: SLQ failed in the optimal final state test!";
}

TEST_F(DdpCorrectnessTest, slq_solution_multiple_partition) {
  ocs2::scalar_array_t partitioningTimes{startTime, (startTime + finalTime) / 2.0, finalTime};

  ocs2::SLQ_Settings settings;
  settings.useNominalTimeForBackwardPass_ = false;
  settings.ddpSettings_ = ddpSettings(partitioningTimes.size() - 1);

  // DDP
  ocs2::SLQ ddp(STATE_DIM, INPUT_DIM, rollout.get(), system.get(), constraint.get(), cost.get(), operatingPoints.get(), settings);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto ddpSolution = ddp.primalSolution(finalTime);
  const auto ddpPerformanceIndeces = ddp.getPerformanceIndeces();

  ASSERT_LT((ddpPerformanceIndeces.totalCost - qpCost), 10.0 * settings.ddpSettings_.minRelCost_)
      << "MESSAGE: multi-threaded SLQ failed in the optimal cost test!";

  ASSERT_LT(relError(ddpSolution.inputTrajectory_.front(), qpSolution.inputTrajectory.front()), solutionPrecision)
      << "MESSAGE: multi-threaded SLQ failed in the optimal initial input test!";

  ASSERT_LT(relError(ddpSolution.stateTrajectory_.back(), qpSolution.stateTrajectory.back()), solutionPrecision)
      << "MESSAGE: multi-threaded SLQ failed in the optimal final state test!";
}

TEST_F(DdpCorrectnessTest, ilqr_solution_single_partition) {
  ocs2::scalar_array_t partitioningTimes{startTime, finalTime};

  ocs2::ILQR_Settings settings;
  settings.ddpSettings_ = ddpSettings(partitioningTimes.size() - 1);

  // DDP
  ocs2::ILQR ddp(STATE_DIM, INPUT_DIM, rollout.get(), system.get(), constraint.get(), cost.get(), operatingPoints.get(), settings);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto ddpSolution = ddp.primalSolution(finalTime);
  const auto ddpPerformanceIndeces = ddp.getPerformanceIndeces();

  ASSERT_LT((ddpPerformanceIndeces.totalCost - qpCost), 10.0 * settings.ddpSettings_.minRelCost_)
      << "MESSAGE: ILQR failed in the optimal cost test!";

  ASSERT_LT(relError(ddpSolution.inputTrajectory_.front(), qpSolution.inputTrajectory.front()), solutionPrecision)
      << "MESSAGE: ILQR failed in the optimal initial input test!";

  ASSERT_LT(relError(ddpSolution.stateTrajectory_.back(), qpSolution.stateTrajectory.back()), solutionPrecision)
      << "MESSAGE: ILQR failed in the optimal final state test!";
}

TEST_F(DdpCorrectnessTest, ilqr_solution_multiple_partition) {
  ocs2::scalar_array_t partitioningTimes{startTime, (startTime + finalTime) / 2.0, finalTime};

  ocs2::ILQR_Settings settings;
  settings.ddpSettings_ = ddpSettings(partitioningTimes.size() - 1);

  // DDP
  ocs2::ILQR ddp(STATE_DIM, INPUT_DIM, rollout.get(), system.get(), constraint.get(), cost.get(), operatingPoints.get(), settings);
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  const auto ddpSolution = ddp.primalSolution(finalTime);
  const auto ddpPerformanceIndeces = ddp.getPerformanceIndeces();

  ASSERT_LT((ddpPerformanceIndeces.totalCost - qpCost), 10.0 * settings.ddpSettings_.minRelCost_)
      << "MESSAGE: multi-threaded ILQR failed in the optimal cost test!";

  ASSERT_LT(relError(ddpSolution.inputTrajectory_.front(), qpSolution.inputTrajectory.front()), solutionPrecision)
      << "MESSAGE: multi-threaded ILQR failed in the optimal initial input test!";

  ASSERT_LT(relError(ddpSolution.stateTrajectory_.back(), qpSolution.stateTrajectory.back()), solutionPrecision)
      << "MESSAGE: multi-threaded ILQR failed in the optimal final state test!";
}
