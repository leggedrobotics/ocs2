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
#include <ocs2_qp_solver/QpDiscreteTranscription.h>
#include <ocs2_qp_solver/QpSolver.h>
#include <ocs2_qp_solver/test/testProblemsGeneration.h>

#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_oc/rollout/PerformanceIndicesRollout.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/testProblemsGeneration.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

enum class NumThreads { SINGLE, MULTI };
enum class Constraining { CONSTARINED, UNCONSTRAINED };

class DDPCorrectness : public testing::TestWithParam<std::tuple<ocs2::search_strategy::Type, Constraining, NumThreads>> {
 protected:
  static constexpr size_t N = 50;
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr ocs2::scalar_t solutionPrecision = 5e-3;
  static constexpr size_t numStateInputConstraints = 2;

  DDPCorrectness() {
    srand(0);

    // Try generateing problem
    for (int retries = 0; !createFeasibleRandomProblem(); retries++) {
      std::cerr << "Random problem was infeasible, retry ...\n";
      if (retries > 10) {
        throw std::runtime_error("Failed generating feasible problem");
      }
    }

    qpSolution = ocs2::qp_solver::solveLinearQuadraticOptimalControlProblem(*problemPtr, nominalTrajectory, initState);
    qpCost = getQpCost(qpSolution);

    // system operating points
    ocs2::vector_array_t inputTrajectoryTemp(N);
    std::copy(nominalTrajectory.inputTrajectory.begin(), nominalTrajectory.inputTrajectory.end(), inputTrajectoryTemp.begin());
    inputTrajectoryTemp.push_back(inputTrajectoryTemp.back());
    operatingPointsPtr.reset(
        new ocs2::OperatingPoints(nominalTrajectory.timeTrajectory, nominalTrajectory.stateTrajectory, inputTrajectoryTemp));

    // rollout settings
    const ocs2::rollout::Settings rolloutSettings = []() {
      ocs2::rollout::Settings rolloutSettings;
      rolloutSettings.absTolODE = 1e-10;
      rolloutSettings.relTolODE = 1e-7;
      rolloutSettings.timeStep = 1e-3;
      rolloutSettings.maxNumStepsPerSecond = 10000;
      return rolloutSettings;
    }();

    // rollout
    rolloutPtr.reset(new ocs2::TimeTriggeredRollout(*systemPtr, rolloutSettings));

    startTime = nominalTrajectory.timeTrajectory.front();
    finalTime = nominalTrajectory.timeTrajectory.back();
  }

  bool createFeasibleRandomProblem() {
    // dynamics
    systemPtr = ocs2::getOcs2Dynamics(ocs2::getRandomDynamics(STATE_DIM, INPUT_DIM));
    problemPtr.reset(new ocs2::OptimalControlProblem);
    problemPtr->dynamicsPtr.reset(systemPtr->clone());

    // cost
    problemPtr->costPtr->add("cost", ocs2::getOcs2Cost(ocs2::getRandomCost(STATE_DIM, INPUT_DIM)));
    problemPtr->finalCostPtr->add("finalCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(STATE_DIM, 0)));
    targetTrajectories = ocs2::TargetTrajectories({0.0}, {ocs2::vector_t::Random(STATE_DIM)}, {ocs2::vector_t::Random(INPUT_DIM)});

    // constraint
    if (std::get<1>(GetParam()) == Constraining::CONSTARINED) {
      problemPtr->equalityConstraintPtr->add(
          "equality", ocs2::getOcs2Constraints(ocs2::getRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints)));
    }

    // system operating points
    nominalTrajectory = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, 1e-3);
    initState = ocs2::vector_t::Random(STATE_DIM);

    // get QP
    problemPtr->targetTrajectoriesPtr = &targetTrajectories;
    ocs2::ScalarFunctionQuadraticApproximation qpCosts;
    ocs2::VectorFunctionLinearApproximation qpConstraints;
    const auto lqApproximation = getLinearQuadraticApproximation(*problemPtr, nominalTrajectory);
    const ocs2::vector_t dx0 = initState - nominalTrajectory.stateTrajectory.front();
    std::tie(qpCosts, qpConstraints) = getDenseQp(lqApproximation, dx0);

    // Check feasibility
    if (!ocs2::qp_solver::isQpFeasible(qpCosts, qpConstraints)) {
      return false;
    }

    // Correct the nominal trajectory to not violate the constraints.
    nominalTrajectory = getFeasibleTrajectory(qpConstraints, nominalTrajectory);

    return true;
  }

  /** Modifies given trajectory to satisfy the constraints */
  ocs2::qp_solver::ContinuousTrajectory getFeasibleTrajectory(const ocs2::VectorFunctionLinearApproximation& qpConstraints,
                                                              const ocs2::qp_solver::ContinuousTrajectory& trajectory) const {
    const auto& A = qpConstraints.dfdx;  // A w + b = 0,  A must be full row-rank such that (A A') is invertible
    const auto& b = qpConstraints.f;     // b = [x0; e[0]; b[0]; ... e[N-1]; b[N-1]; e[N]]

    /* Find the trajectory correction w to satisfy the constraint by solving
     *   min  1/2 w' w
     *   s.t. A w + b = 0  */
    const ocs2::vector_t w = -A.transpose() * (A * A.transpose()).inverse() * b;  // w = [dx[0], du[0], dx[1],  du[1], ..., dx[N]]

    // Make trajectory feasible
    auto feasibleTrajectory = trajectory;
    int nextIndex = 0;
    const auto N = feasibleTrajectory.inputTrajectory.size();
    for (int k = 0; k < N; k++) {
      const auto nx = feasibleTrajectory.stateTrajectory[k].size();
      const auto nu = feasibleTrajectory.inputTrajectory[k].size();
      feasibleTrajectory.stateTrajectory[k] += w.segment(nextIndex, nx);       // dx[k]
      feasibleTrajectory.inputTrajectory[k] += w.segment(nextIndex + nx, nu);  // du[k]
      nextIndex += nx + nu;
    }
    feasibleTrajectory.stateTrajectory[N] += w.segment(nextIndex, feasibleTrajectory.stateTrajectory[N].size());  // dx[N]

    return feasibleTrajectory;
  }

  ocs2::search_strategy::Type getSearchStrategy() { return std::get<0>(GetParam()); }

  size_t getNumThreads() const {
    const auto n = std::get<2>(GetParam());
    if (n == NumThreads::SINGLE) {
      return 1;
    } else {
      return 2;
    }
  }

  ocs2::ddp::Settings getSettings(ocs2::ddp::Algorithm algorithmType, size_t numThreads, ocs2::search_strategy::Type strategy,
                                  bool display = false) const {
    ocs2::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.minRelCost_ = 1e-3;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.maxNumIterations_ = 2 + (numThreads - 1);  // need an extra iteration for each added time partition
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength_ = 1e-4;
    return ddpSettings;
  }

  ocs2::scalar_t getQpCost(const ocs2::qp_solver::ContinuousTrajectory& qpSolution) const {
    auto costFunc = [this](ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) {
      return problemPtr->costPtr->getValue(t, x, u, targetTrajectories, ocs2::PreComputation());
    };
    auto inputTrajectoryTemp = qpSolution.inputTrajectory;
    inputTrajectoryTemp.emplace_back(inputTrajectoryTemp.back());
    auto lAccum =
        ocs2::PerformanceIndicesRollout::rolloutCost(costFunc, qpSolution.timeTrajectory, qpSolution.stateTrajectory, inputTrajectoryTemp);

    return lAccum + problemPtr->finalCostPtr->getValue(qpSolution.timeTrajectory.back(), qpSolution.stateTrajectory.back(),
                                                       targetTrajectories, ocs2::PreComputation());
  }

  std::string getTestName(const ocs2::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "Correctness Test { ";
    testName += "Algorithm: " + ocs2::ddp::toAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + ocs2::search_strategy::toString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void correctnessTest(const ocs2::ddp::Settings& ddpSettings, const ocs2::PerformanceIndex& performanceIndex,
                       const ocs2::PrimalSolution& ddpSolution) const {
    const auto testName = getTestName(ddpSettings);
    EXPECT_LT(fabs(performanceIndex.cost - qpCost), 10 * ddpSettings.minRelCost_)
        << "MESSAGE: " << testName << ": failed in the optimal cost test!";
    EXPECT_LT(relError(ddpSolution.stateTrajectory_.back(), qpSolution.stateTrajectory.back()), solutionPrecision)
        << "MESSAGE: " << testName << ": failed in the optimal final state test!";
    EXPECT_LT(relError(ddpSolution.inputTrajectory_.front(), qpSolution.inputTrajectory.front()), solutionPrecision)
        << "MESSAGE: " << testName << ": failed in the optimal initial input test!";
  }

  ocs2::scalar_t relError(ocs2::vector_t ddpSol, const ocs2::vector_t& qpSol) const { return (ddpSol - qpSol).norm() / ddpSol.norm(); }

  ocs2::vector_t initState;
  ocs2::scalar_t startTime;
  ocs2::scalar_t finalTime;

  ocs2::TargetTrajectories targetTrajectories;
  std::unique_ptr<ocs2::SystemDynamicsBase> systemPtr;
  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr;
  std::unique_ptr<ocs2::OperatingPoints> operatingPointsPtr;
  ocs2::qp_solver::ContinuousTrajectory nominalTrajectory;
  std::unique_ptr<ocs2::TimeTriggeredRollout> rolloutPtr;

  ocs2::scalar_t qpCost;
  ocs2::qp_solver::ContinuousTrajectory qpSolution;
};

constexpr size_t DDPCorrectness::N;
constexpr size_t DDPCorrectness::STATE_DIM;
constexpr size_t DDPCorrectness::INPUT_DIM;
constexpr size_t DDPCorrectness::numStateInputConstraints;
constexpr ocs2::scalar_t DDPCorrectness::solutionPrecision;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(DDPCorrectness, TestSLQ) {
  // settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::SLQ, getNumThreads(), getSearchStrategy());

  // ddp
  ocs2::SLQ ddp(ddpSettings, *rolloutPtr, *problemPtr, *operatingPointsPtr);

  ddp.getReferenceManager().setTargetTrajectories(targetTrajectories);
  ddp.run(startTime, initState, finalTime);
  const auto performanceIndex = ddp.getPerformanceIndeces();
  const auto solution = ddp.primalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(DDPCorrectness, TestILQR) {
  // settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::ILQR, getNumThreads(), getSearchStrategy());

  // ddp
  ocs2::ILQR ddp(ddpSettings, *rolloutPtr, *problemPtr, *operatingPointsPtr);

  ddp.getReferenceManager().setTargetTrajectories(targetTrajectories);
  ddp.run(startTime, initState, finalTime);
  const auto performanceIndex = ddp.getPerformanceIndeces();
  const auto solution = ddp.primalSolution(finalTime);

  correctnessTest(ddpSettings, performanceIndex, solution);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/* Test name printed in gtest results */
std::string testName(const testing::TestParamInfo<DDPCorrectness::ParamType>& info) {
  std::string name;
  name += ocs2::search_strategy::toString(std::get<0>(info.param)) + "__";
  name += std::get<1>(info.param) == Constraining::CONSTARINED ? "CONSTARINED" : "UNCONSTRAINED";
  name += "__";
  name += std::get<2>(info.param) == NumThreads::SINGLE ? "SINGLE_THREAD" : "MULTI_THREAD";
  return name;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(DDPCorrectnessTestCase, DDPCorrectness,
                        testing::Combine(testing::ValuesIn({ocs2::search_strategy::Type::LINE_SEARCH,
                                                            ocs2::search_strategy::Type::LEVENBERG_MARQUARDT}),
                                         testing::ValuesIn({Constraining::CONSTARINED, Constraining::UNCONSTRAINED}),
                                         testing::ValuesIn({NumThreads::SINGLE, NumThreads::MULTI})),
                        testName);
