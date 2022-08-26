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

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/EXP1.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

class Exp1 : public testing::TestWithParam<std::tuple<ocs2::search_strategy::Type, size_t>> {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr ocs2::scalar_t timeStep = 1e-2;
  static constexpr ocs2::scalar_t minRelCost = 1e-3;
  static constexpr ocs2::scalar_t expectedCost = 5.4399;

  Exp1() {
    // event times
    const ocs2::scalar_array_t eventTimes{0.2262, 1.0176};
    const std::vector<size_t> modeSequence{0, 1, 2};
    referenceManagerPtr = ocs2::getExp1ReferenceManager(eventTimes, modeSequence);

    problem = ocs2::createExp1Problem(referenceManagerPtr);

    // operatingTrajectories
    initializerPtr.reset(new ocs2::DefaultInitializer(INPUT_DIM));
  }

  ocs2::search_strategy::Type getSearchStrategy() { return std::get<0>(GetParam()); }

  size_t getNumThreads() { return std::get<1>(GetParam()); }

  // rollout settings
  ocs2::rollout::Settings rolloutSettings() const {
    ocs2::rollout::Settings rolloutSettings;
    rolloutSettings.absTolODE = 1e-10;
    rolloutSettings.relTolODE = 1e-7;
    rolloutSettings.timeStep = timeStep;
    rolloutSettings.integratorType = ocs2::IntegratorType::ODE45;
    rolloutSettings.maxNumStepsPerSecond = 10000;
    return rolloutSettings;
  };

  ocs2::ddp::Settings getSettings(ocs2::ddp::Algorithm algorithmType, size_t numThreads, ocs2::search_strategy::Type strategy,
                                  bool display = false) const {
    ocs2::ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = algorithmType;
    ddpSettings.nThreads_ = numThreads;
    ddpSettings.preComputeRiccatiTerms_ = false;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = display;
    ddpSettings.maxNumIterations_ = 30;
    ddpSettings.minRelCost_ = minRelCost;
    ddpSettings.checkNumericalStability_ = true;
    ddpSettings.absTolODE_ = 1e-10;
    ddpSettings.relTolODE_ = 1e-7;
    ddpSettings.timeStep_ = timeStep;
    ddpSettings.backwardPassIntegratorType_ = ocs2::IntegratorType::ODE45;
    ddpSettings.maxNumStepsPerSecond_ = 10000;
    ddpSettings.useFeedbackPolicy_ = false;
    ddpSettings.debugPrintRollout_ = false;
    ddpSettings.strategy_ = strategy;
    ddpSettings.lineSearch_.minStepLength = 0.0001;
    return ddpSettings;
  }

  std::string getTestName(const ocs2::ddp::Settings& ddpSettings) const {
    std::string testName;
    testName += "EXP1 Test { ";
    testName += "Algorithm: " + ocs2::ddp::toAlgorithmName(ddpSettings.algorithm_) + ",  ";
    testName += "Strategy: " + ocs2::search_strategy::toString(ddpSettings.strategy_) + ",  ";
    testName += "#threads: " + std::to_string(ddpSettings.nThreads_) + " }";
    return testName;
  }

  void performanceIndexTest(const ocs2::ddp::Settings& ddpSettings, const ocs2::PerformanceIndex& performanceIndex) const {
    const auto testName = getTestName(ddpSettings);
    EXPECT_NEAR(performanceIndex.cost, expectedCost, 10.0 * minRelCost) << "MESSAGE: " << testName << ": failed in the total cost test!";
    EXPECT_NEAR(performanceIndex.equalityConstraintsSSE, 0.0, 10.0 * ddpSettings.constraintTolerance_)
        << "MESSAGE: " << testName << ": failed in state-input equality constraint ISE test!";
  }

  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 3.0;
  const ocs2::vector_t initState = (ocs2::vector_t(STATE_DIM) << 2.0, 3.0).finished();
  std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr;

  ocs2::OptimalControlProblem problem;
  std::unique_ptr<ocs2::Initializer> initializerPtr;
};

constexpr size_t Exp1::STATE_DIM;
constexpr size_t Exp1::INPUT_DIM;
constexpr ocs2::scalar_t Exp1::timeStep;
constexpr ocs2::scalar_t Exp1::minRelCost;
constexpr ocs2::scalar_t Exp1::expectedCost;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_F(Exp1, ddp_hamiltonian) {
  // ddp settings
  auto ddpSettings = getSettings(ocs2::ddp::Algorithm::SLQ, 2, ocs2::search_strategy::Type::LINE_SEARCH);
  ddpSettings.useFeedbackPolicy_ = true;
  ddpSettings.maxNumIterations_ = 50;
  ddpSettings.minRelCost_ = 1e-9;  // to allow more iterations that the effect of final linesearch is negligible

  // dynamics and rollout
  ocs2::EXP1_System systemDynamics(referenceManagerPtr);
  ocs2::TimeTriggeredRollout rollout(systemDynamics, rolloutSettings());

  // instantiate
  ocs2::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.setReferenceManager(referenceManagerPtr);

  // run ddp
  ddp.run(startTime, initState, finalTime);
  // get solution
  const auto solution = ddp.primalSolution(finalTime);

  // define precision for tests
  constexpr ocs2::scalar_t precision = 1e-3;
  // note: in the following highly non-linear system means more than quadratic in the state

  // get Hamiltonian at current solution
  // expected outcome: true, because the current solution should be optimal
  ocs2::scalar_t time = solution.timeTrajectory_.front();
  ocs2::vector_t state = solution.stateTrajectory_.front();
  ocs2::vector_t input = solution.controllerPtr_->computeInput(time, state);
  auto hamiltonian = ddp.getHamiltonian(time, state, input);
  const ocs2::vector_t dHdu1a = hamiltonian.dfdu;
  EXPECT_TRUE(dHdu1a.isZero(precision)) << "MESSAGE for test 1a: Derivative of Hamiltonian w.r.t. to u is not zero: " << dHdu1a.transpose();

  // evaluate Hamiltonian at different state (but using feedback policy)
  // expected outcome: true, because for a highly non-linear system the LQ approximation of H is not exact,
  //                         however since the subsystems' dynamics are control-affine and the cost is quadratic w.r.t. to input,
  //                         this looks like a LQR problem and the linear feedback policy appears to be globally optimal
  ocs2::scalar_t querryTime = solution.timeTrajectory_.front();
  ocs2::vector_t querryState = ocs2::vector_t::Random(solution.stateTrajectory_.front().size());
  ocs2::vector_t querryInput = solution.controllerPtr_->computeInput(querryTime, querryState);
  const ocs2::vector_t dHdu1b = hamiltonian.dfdux * (querryState - state) + hamiltonian.dfduu * (querryInput - input) + hamiltonian.dfdu;
  EXPECT_TRUE(dHdu1b.isZero(precision)) << "MESSAGE for test 1b: Derivative of Hamiltonian w.r.t. to u is not zero: " << dHdu1b.transpose();

  // evaluate Hamiltonian at different input
  // expected outcome: false, because for a highly non-linear system the LQ approximation of H is not exact
  //                          but still a random input does not appear to be optimal
  querryTime = solution.timeTrajectory_.front();
  querryState = solution.stateTrajectory_.front();
  querryInput = ocs2::vector_t::Random(solution.inputTrajectory_.front().size());
  const ocs2::vector_t dHdu1c = hamiltonian.dfdux * (querryState - state) + hamiltonian.dfduu * (querryInput - input) + hamiltonian.dfdu;
  EXPECT_FALSE(dHdu1c.isZero(precision)) << "MESSAGE for test 1c: Derivative of Hamiltonian w.r.t. to u is zero: " << dHdu1c.transpose();

  // get Hamiltonian at different state (but using feedback policy)
  // expected outcome: false, because for a highly non-linear system the linear feedback policy is not globally optimal
  time = solution.timeTrajectory_.front();
  state = ocs2::vector_t::Random(solution.stateTrajectory_.front().size());
  input = solution.controllerPtr_->computeInput(time, state);
  hamiltonian = ddp.getHamiltonian(time, state, input);
  const ocs2::vector_t dHdu2 = hamiltonian.dfdu;
  EXPECT_FALSE(dHdu2.isZero(precision)) << "MESSAGE for test 2: Derivative of Hamiltonian w.r.t. to u is zero: " << dHdu2.transpose();

  // get Hamiltonian at different input
  // expected outcome: false, because a random input is not optimal
  time = solution.timeTrajectory_.front();
  state = solution.stateTrajectory_.front();
  input = ocs2::vector_t::Random(solution.inputTrajectory_.front().size());
  hamiltonian = ddp.getHamiltonian(time, state, input);
  const ocs2::vector_t dHdu3 = hamiltonian.dfdu;
  EXPECT_FALSE(dHdu3.isZero(precision)) << "MESSAGE for test 3: Derivative of Hamiltonian w.r.t. to u is zero: " << dHdu3.transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(Exp1, SLQ) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::SLQ, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  ocs2::EXP1_System systemDynamics(referenceManagerPtr);
  ocs2::TimeTriggeredRollout rollout(systemDynamics, rolloutSettings());

  // instantiate
  ocs2::SLQ ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.setReferenceManager(referenceManagerPtr);

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
TEST_P(Exp1, ILQR) {
  // ddp settings
  const auto ddpSettings = getSettings(ocs2::ddp::Algorithm::ILQR, getNumThreads(), getSearchStrategy());

  // dynamics and rollout
  ocs2::EXP1_System systemDynamics(referenceManagerPtr);
  ocs2::TimeTriggeredRollout rollout(systemDynamics, rolloutSettings());

  // instantiate
  ocs2::ILQR ddp(ddpSettings, rollout, problem, *initializerPtr);
  ddp.setReferenceManager(referenceManagerPtr);

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
INSTANTIATE_TEST_CASE_P(Exp1Case, Exp1,
                        testing::Combine(testing::ValuesIn({ocs2::search_strategy::Type::LINE_SEARCH,
                                                            ocs2::search_strategy::Type::LEVENBERG_MARQUARDT}),
                                         testing::ValuesIn({size_t(1), size_t(3)})), /* num threads */
                        [](const testing::TestParamInfo<Exp1::ParamType>& info) {
                          /* returns test name for gtest summary */
                          std::string name;
                          name += ocs2::search_strategy::toString(std::get<0>(info.param)) + "__";
                          name += std::get<1>(info.param) == 1 ? "SINGLE_THREAD" : "MULTI_THREAD";
                          return name;
                        });
