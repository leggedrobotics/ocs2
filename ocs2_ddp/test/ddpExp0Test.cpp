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

#include <ocs2_core/Types.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/EXP0.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

class ddpExp0Test : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr ocs2::scalar_t expectedCost = 9.766;
  static constexpr ocs2::scalar_t expectedStateInputEqConstraintISE = 0.0;
  static constexpr ocs2::scalar_t expectedStateEqConstraintISE = 0.0;

  ddpExp0Test() {
    // event times
    const ocs2::scalar_array_t eventTimes{0.1897};
    const std::vector<size_t> subsystemsSequence{0, 1};
    modeScheduleManagerPtr.reset(new ocs2::ModeScheduleManager({eventTimes, subsystemsSequence}));

    // partitioning times
    partitioningTimes = ocs2::scalar_array_t{startTime, eventTimes[0], finalTime};

    // rollout settings
    const auto rolloutSettings = []() {
      ocs2::rollout::Settings rolloutSettings;
      rolloutSettings.absTolODE_ = 1e-10;
      rolloutSettings.relTolODE_ = 1e-7;
      rolloutSettings.maxNumStepsPerSecond_ = 10000;
      return rolloutSettings;
    }();

    // dynamics and rollout
    systemPtr.reset(new ocs2::EXP0_System(modeScheduleManagerPtr));
    rolloutPtr.reset(new ocs2::TimeTriggeredRollout(*systemPtr, rolloutSettings));

    // cost function
    costPtr.reset(new ocs2::EXP0_CostFunction(modeScheduleManagerPtr));

    // constraint
    constraintPtr.reset(new ocs2::ConstraintBase);

    // operatingTrajectories
    const auto stateOperatingPoint = ocs2::vector_t::Zero(STATE_DIM);
    const auto inputOperatingPoint = ocs2::vector_t::Zero(INPUT_DIM);
    operatingPointsPtr.reset(new ocs2::OperatingPoints(stateOperatingPoint, inputOperatingPoint));
  }

  ocs2::ddp::Settings getSettings(ocs2::ddp::algorithm algorithmType, size_t numThreads,
                                  ocs2::ddp_strategy::type strategy, bool display = false) const {
    ocs2::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.preComputeRiccatiTerms_ = true;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.maxNumIterations_ = 30;
    ddpSettings.minRelCost_ = 1e-3;
    ddpSettings.checkNumericalStability_ = true;
    ddpSettings.useNominalTimeForBackwardPass_ = false;
    ddpSettings.useFeedbackPolicy_ = true;
    ddpSettings.debugPrintRollout_ = false;
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength_ = 0.0001;
    return ddpSettings;
  }

  std::string getTestName(const ocs2::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "EXP0 Test { ";
    testName += "Algorithm: " + ocs2::ddp::toAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + ocs2::ddp_strategy::toString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void performanceIndexTest(const ocs2::ddp::Settings& ddpSettings, const ocs2::PerformanceIndex& performanceIndex) const {
    const auto testName = getTestName(ddpSettings);
    ASSERT_LT(fabs(performanceIndex.totalCost - expectedCost), 10 * ddpSettings.minRelCost_)
        << "MESSAGE: " << testName << ": failed in the total cost test!";
    ASSERT_LT(fabs(performanceIndex.stateInputEqConstraintISE - expectedStateInputEqConstraintISE), 10 * ddpSettings.constraintTolerance_)
        << "MESSAGE: " << testName << ": failed in state-input equality constraint ISE test!";
    ASSERT_LT(fabs(performanceIndex.stateEqConstraintISE - expectedStateEqConstraintISE), 10 * ddpSettings.constraintTolerance_)
       << "MESSAGE: " << testName << ": failed in state-only equality constraint ISE test!";
  }

  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 2.0;
  const ocs2::vector_t initState = (ocs2::vector_t(STATE_DIM) << 0.0, 2.0).finished();
  ocs2::scalar_array_t partitioningTimes;
  std::shared_ptr<ocs2::ModeScheduleManager> modeScheduleManagerPtr;

  std::unique_ptr<ocs2::SystemDynamicsBase> systemPtr;
  std::unique_ptr<ocs2::TimeTriggeredRollout> rolloutPtr;
  std::unique_ptr<ocs2::CostFunctionBase> costPtr;
  std::unique_ptr<ocs2::ConstraintBase> constraintPtr;
  std::unique_ptr<ocs2::OperatingPoints> operatingPointsPtr;
};

constexpr size_t ddpExp0Test::STATE_DIM;
constexpr size_t ddpExp0Test::INPUT_DIM;
constexpr ocs2::scalar_t ddpExp0Test::expectedCost;
constexpr ocs2::scalar_t ddpExp0Test::expectedStateInputEqConstraintISE;
constexpr ocs2::scalar_t ddpExp0Test::expectedStateEqConstraintISE;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, slq_single_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, 1, ocs2::ddp_strategy::type::LINE_SEARCH);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, slq_multi_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, 3, ocs2::ddp_strategy::type::LINE_SEARCH);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, ilqr_single_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::ILQR, 1, ocs2::ddp_strategy::type::LINE_SEARCH);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, ilqr_multi_thread_linesearch) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::ILQR, 3, ocs2::ddp_strategy::type::LINE_SEARCH);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, slq_single_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, 1, ocs2::ddp_strategy::type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, slq_multi_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, 3, ocs2::ddp_strategy::type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, ilqr_single_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::ILQR, 1, ocs2::ddp_strategy::type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, ilqr_multi_thread_levenberg_marquardt) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::algorithm::ILQR, 3, ocs2::ddp_strategy::type::LEVENBERG_MARQUARDT);

  // instantiate
  ocs2::ILQR ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n" << getTestName(ddpSettings) << "\n";
  }

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get performance index
  const auto performanceIndex = ddp.getPerformanceIndeces();

  // performanceIndeces test
  performanceIndexTest(ddpSettings, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, ddp_feedback_policy) {
  // ddp settings
  auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, 2, ocs2::ddp_strategy::type::LINE_SEARCH);
  ddpSettings.useFeedbackPolicy_ = true;
  ddpSettings.displayInfo_ = false;
  ddpSettings.displayShortSummary_ = false;

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get solution
  const auto solution = ddp.primalSolution(finalTime);
  const auto ctrlFinalTime = dynamic_cast<ocs2::LinearController*>(solution.controllerPtr_.get())->timeStamp_.back();

  ASSERT_DOUBLE_EQ(solution.timeTrajectory_.back(), finalTime) << "MESSAGE: SLQ failed in policy final time of trajectory!";
  ASSERT_DOUBLE_EQ(ctrlFinalTime, finalTime) << "MESSAGE: SLQ failed in policy final time of controller!";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, ddp_feedforward_policy) {
  // ddp settings
  auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, 2, ocs2::ddp_strategy::type::LINE_SEARCH);
  ddpSettings.useFeedbackPolicy_ = false;
  ddpSettings.displayInfo_ = false;
  ddpSettings.displayShortSummary_ = false;

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  // run ddp
  ddp.run(startTime, initState, finalTime, partitioningTimes);
  // get solution
  const auto solution = ddp.primalSolution(finalTime);
  const auto ctrlFinalTime = dynamic_cast<ocs2::FeedforwardController*>(solution.controllerPtr_.get())->timeStamp_.back();

  ASSERT_DOUBLE_EQ(solution.timeTrajectory_.back(), finalTime) << "MESSAGE: SLQ failed in policy final time of trajectory!";
  ASSERT_DOUBLE_EQ(ctrlFinalTime, finalTime) << "MESSAGE: SLQ failed in policy final time of controller!";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(ddpExp0Test, ddp_caching) {
  // ddp settings
  auto ddpSettings = getSettings(ocs2::ddp::algorithm::SLQ, 2, ocs2::ddp_strategy::type::LINE_SEARCH);
  ddpSettings.displayInfo_ = false;
  ddpSettings.displayShortSummary_ = false;

  // event times
  const ocs2::scalar_array_t eventTimes{1.0};
  const std::vector<size_t> subsystemsSequence{0, 1};
  modeScheduleManagerPtr.reset(new ocs2::ModeScheduleManager({eventTimes, subsystemsSequence}));

  // instantiate
  ocs2::SLQ ddp(rolloutPtr.get(), systemPtr.get(), constraintPtr.get(), costPtr.get(), operatingPointsPtr.get(), ddpSettings);
  ddp.setModeScheduleManager(modeScheduleManagerPtr);

  // run single core SLQ (no active event)
  ocs2::scalar_t startTime = 0.2;
  ocs2::scalar_t finalTime = 0.7;
  ASSERT_NO_THROW(ddp.run(startTime, initState, finalTime, partitioningTimes));

  // run similar to the MPC setup (a new partition)
  startTime = 0.4;
  finalTime = 0.9;
  ASSERT_NO_THROW(ddp.run(startTime, initState, finalTime, partitioningTimes, std::vector<ocs2::ControllerBase*>()));

  // run similar to the MPC setup (one active event)
  startTime = 0.6;
  finalTime = 1.2;
  ASSERT_NO_THROW(ddp.run(startTime, initState, finalTime, partitioningTimes, std::vector<ocs2::ControllerBase*>()));

  // run similar to the MPC setup (no active event + a new partition)
  startTime = 1.1;
  finalTime = 1.5;
  ASSERT_NO_THROW(ddp.run(startTime, initState, finalTime, partitioningTimes, std::vector<ocs2::ControllerBase*>()));

  // run similar to the MPC setup (no overlap)
  startTime = 1.6;
  finalTime = 2.0;
  ASSERT_NO_THROW(ddp.run(startTime, initState, finalTime, partitioningTimes, std::vector<ocs2::ControllerBase*>()));
}
