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
#include <string>

#include <boost/filesystem.hpp>

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/circular_kinematics.h>

class CircularKinematicsTest : public testing::TestWithParam<std::tuple<ocs2::search_strategy::Type, size_t>> {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr ocs2::scalar_t timeStep = 0.01;
  static constexpr ocs2::scalar_t minRelCost = 1e-3;
  static constexpr ocs2::scalar_t constraintTolerance = 1e-5;
  static constexpr ocs2::scalar_t expectedCost = 0.1;

  CircularKinematicsTest() {
    // optimal control problem
    boost::filesystem::path filePath(__FILE__);
    problem = ocs2::createCircularKinematicsProblem("/tmp/ocs2/ddp_test_generated");

    // initializer
    initializerPtr.reset(new ocs2::DefaultInitializer(INPUT_DIM));
  }

  ocs2::search_strategy::Type getSearchStrategy() { return std::get<0>(GetParam()); }

  size_t getNumThreads() { return std::get<1>(GetParam()); }

  ocs2::rollout::Settings rolloutSettings(ocs2::ddp::Algorithm algorithmType) const {
    ocs2::rollout::Settings rolloutSettings;
    rolloutSettings.absTolODE = 1e-9;
    rolloutSettings.relTolODE = 1e-7;
    rolloutSettings.timeStep = timeStep;
    rolloutSettings.maxNumStepsPerSecond = 10000;

    switch (algorithmType) {
      case ocs2::ddp::Algorithm::SLQ:
        rolloutSettings.integratorType = ocs2::IntegratorType::ODE45;
        break;
      case ocs2::ddp::Algorithm::ILQR:
        rolloutSettings.integratorType = ocs2::IntegratorType::RK4;
        break;
      default:
        throw std::runtime_error("Not supported Algorithm!");
    }

    return rolloutSettings;
  };

  ocs2::ddp::Settings getSettings(ocs2::ddp::Algorithm algorithmType, size_t numThreads, ocs2::search_strategy::Type strategy,
                                  bool display = false) const {
    ocs2::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.checkNumericalStability_ = false;
    ddpSettings.debugPrintRollout_ = false;
    ddpSettings.absTolODE_ = 1e-9;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.timeStep_ = timeStep;
    switch (algorithmType) {
      case ocs2::ddp::Algorithm::SLQ:
        ddpSettings.backwardPassIntegratorType_ = ocs2::IntegratorType::ODE45;
        break;
      case ocs2::ddp::Algorithm::ILQR:
        ddpSettings.backwardPassIntegratorType_ = ocs2::IntegratorType::RK4;
        break;
      default:
        throw std::runtime_error("Not supported Algorithm!");
    }
    ddpSettings.maxNumIterations_ = 150;
    ddpSettings.minRelCost_ = minRelCost;
    ddpSettings.constraintTolerance_ = constraintTolerance;
    ddpSettings.constraintPenaltyInitialValue_ = 2.0;
    ddpSettings.constraintPenaltyIncreaseRate_ = 1.5;
    ddpSettings.preComputeRiccatiTerms_ = false;
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength = 0.01;
    ddpSettings.lineSearch_.hessianCorrectionStrategy = ocs2::hessian_correction::Strategy::CHOLESKY_MODIFICATION;
    ddpSettings.lineSearch_.hessianCorrectionMultiple = 1e-3;
    return ddpSettings;
  }

  std::string getTestName(const ocs2::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "Circular-Kinematics Test { ";
    testName += "Algorithm: " + ocs2::ddp::toAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + ocs2::search_strategy::toString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void performanceIndexTest(const ocs2::ddp::Settings& ddpSettings, const ocs2::PerformanceIndex& performanceIndex) const {
    const auto testName = getTestName(ddpSettings);
    EXPECT_LT(performanceIndex.cost - expectedCost, 0.0) << "MESSAGE: " << testName << ": failed in the cost test!";
    EXPECT_NEAR(performanceIndex.equalityConstraintsSSE, 0.0, 10.0 * constraintTolerance)
        << "MESSAGE: " << testName << ": failed in state-input equality constraint ISE test!";
  }

  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 10.0;
  const ocs2::vector_t initState = (ocs2::vector_t(STATE_DIM) << 1.0, 0.0).finished();  // radius 1.0

  ocs2::OptimalControlProblem problem;
  std::unique_ptr<ocs2::Initializer> initializerPtr;
};

constexpr size_t CircularKinematicsTest::STATE_DIM;
constexpr size_t CircularKinematicsTest::INPUT_DIM;
constexpr ocs2::scalar_t CircularKinematicsTest::timeStep;
constexpr ocs2::scalar_t CircularKinematicsTest::minRelCost;
constexpr ocs2::scalar_t CircularKinematicsTest::constraintTolerance;
constexpr ocs2::scalar_t CircularKinematicsTest::expectedCost;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(CircularKinematicsTest, SLQ) {
  const auto algorithm = ocs2::ddp::Algorithm::SLQ;

  // ddp settings
  const auto ddpSettings = getSettings(algorithm, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  const ocs2::CircularKinematicsSystem systemDynamics;
  const ocs2::TimeTriggeredRollout rollout(systemDynamics, rolloutSettings(algorithm));

  // instantiate
  ocs2::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(CircularKinematicsTest, ILQR) {
  const auto algorithm = ocs2::ddp::Algorithm::ILQR;

  // ddp settings
  const auto ddpSettings = getSettings(algorithm, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  const ocs2::CircularKinematicsSystem systemDynamics;
  const ocs2::TimeTriggeredRollout rollout(systemDynamics, rolloutSettings(algorithm));

  // instantiate
  ocs2::ILQR ddp(ddpSettings, rollout, problem, *initializerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(CircularKinematicsTestCase, CircularKinematicsTest,
                        testing::Combine(testing::ValuesIn({ocs2::search_strategy::Type::LINE_SEARCH
                                                            /* , ocs2::search_strategy::Type::LEVENBERG_MARQUARDT */}),
                                         testing::ValuesIn({size_t(1), size_t(3)})), /* num threads */
                        [](const testing::TestParamInfo<CircularKinematicsTest::ParamType>& info) {
                          /* returns test name for gtest summary */
                          std::string name;
                          name += ocs2::search_strategy::toString(std::get<0>(info.param)) + "__";
                          name += std::get<1>(info.param) == 1 ? "SINGLE_THREAD" : "MULTI_THREAD";
                          return name;
                        });
