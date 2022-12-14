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

#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/synchronized_module/SolverObserver.h>

#include "ocs2_cartpole/CartPoleInterface.h"
#include "ocs2_cartpole/package_path.h"

using namespace ocs2;
using namespace cartpole;

enum class PenaltyType {
  SlacknessSquaredHingePenalty,
  ModifiedRelaxedBarrierPenalty,
};

class TestCartpole : public testing::TestWithParam<std::tuple<ddp::Algorithm, PenaltyType>> {
 protected:
  static constexpr scalar_t timeHorizon = 15.0;
  static constexpr scalar_t goalViolationTolerance = 1e-1;
  static constexpr scalar_t constraintViolationTolerance = 1e-1;

  TestCartpole() {
    // interface
    taskFile = ocs2::cartpole::getPath() + "/config/mpc/task.info";
    const std::string libFolder = ocs2::cartpole::getPath() + "/auto_generated";
    cartPoleInterfacePtr.reset(new CartPoleInterface(taskFile, libFolder, false /*verbose*/));

    // Since the problem only uses final cost for swing-up, the final cost should be scaled proportional to
    // the increase of time horizon. We fist remove the final cost and then add a scaled version
    const std::string finalCostName = "finalCost";
    if (!cartPoleInterfacePtr->optimalControlProblem().finalCostPtr->erase(finalCostName)) {
      throw std::runtime_error("[TestCartpole::TestCartpole]: " + finalCostName + " was not found!");
    }
    auto createFinalCost = [&]() {
      matrix_t Qf(STATE_DIM, STATE_DIM);
      loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
      Qf *= (timeHorizon / cartPoleInterfacePtr->mpcSettings().timeHorizon_);  // scale cost
      return std::make_unique<QuadraticStateCost>(Qf);
    };
    cartPoleInterfacePtr->optimalControlProblem().finalCostPtr->add(finalCostName, createFinalCost());

    // remove InputLimits as it will be added later in each test
    inputLimitsConstraint = popInequalityLagrangian(inputLimitsName, cartPoleInterfacePtr->optimalControlProblem());

    // initial command
    initTargetTrajectories.timeTrajectory.push_back(0.0);
    initTargetTrajectories.stateTrajectory.push_back(cartPoleInterfacePtr->getInitialTarget());
    initTargetTrajectories.inputTrajectory.push_back(vector_t::Zero(ocs2::cartpole::INPUT_DIM));
  }

  std::unique_ptr<GaussNewtonDDP> getAlgorithm() const {
    const auto algorithm = std::get<0>(GetParam());
    auto ddpSettings = cartPoleInterfacePtr->ddpSettings();
    ddpSettings.algorithm_ = algorithm;
    ddpSettings.maxNumIterations_ = 100;
    ddpSettings.minRelCost_ = 1e-12;  // to avoid early termination
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = true;

    switch (algorithm) {
      case ddp::Algorithm::SLQ:
        return std::make_unique<SLQ>(std::move(ddpSettings), cartPoleInterfacePtr->getRollout(),
                                     createOptimalControlProblem(PenaltyType::ModifiedRelaxedBarrierPenalty),
                                     cartPoleInterfacePtr->getInitializer());

      case ddp::Algorithm::ILQR:
        return std::make_unique<ILQR>(std::move(ddpSettings), cartPoleInterfacePtr->getRollout(),
                                      createOptimalControlProblem(PenaltyType::ModifiedRelaxedBarrierPenalty),
                                      cartPoleInterfacePtr->getInitializer());

      default:
        throw std::runtime_error("[TestCartpole::getAlgorithm] undefined algorithm");
    }
  }

  std::unique_ptr<augmented::AugmentedPenaltyBase> getPenalty() const {
    const auto penaltyType = std::get<1>(GetParam());

    switch (penaltyType) {
      case PenaltyType::SlacknessSquaredHingePenalty: {
        using penalty_type = augmented::SlacknessSquaredHingePenalty;
        penalty_type::Config boundsConfig;
        loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, false);
        return penalty_type::create(boundsConfig);
      }

      case PenaltyType::ModifiedRelaxedBarrierPenalty: {
        using penalty_type = augmented::ModifiedRelaxedBarrierPenalty;
        penalty_type::Config boundsConfig;
        loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, false);
        return penalty_type::create(boundsConfig);
      }

      default:
        throw std::runtime_error("[TestCartpole::getPenalty] undefined penaltyType");
    }
  }

  OptimalControlProblem createOptimalControlProblem(PenaltyType penaltyType) const {
    OptimalControlProblem problem = cartPoleInterfacePtr->getOptimalControlProblem();
    problem.inequalityLagrangianPtr->add(inputLimitsName,
                                         create(std::unique_ptr<StateInputConstraint>(inputLimitsConstraint->clone()), getPenalty()));
    return problem;
  }

  std::unique_ptr<StateInputConstraint> popInequalityLagrangian(const std::string& name, OptimalControlProblem& ocp) const {
    auto termLagrangianPtr = ocp.inequalityLagrangianPtr->extract(name);
    if (termLagrangianPtr == nullptr) {
      throw std::runtime_error("[TestCartpole::popInequalityLagrangian]: " + name + " was not found!");
    }

    auto termStateInpuLagrangianPtr = dynamic_cast<StateInputAugmentedLagrangian*>(termLagrangianPtr.get());
    if (termStateInpuLagrangianPtr == nullptr) {
      throw std::runtime_error("[TestCartpole::popInequalityLagrangian]: term " + name + " is not of type StateInputAugmentedLagrangian!");
    }

    return std::unique_ptr<StateInputConstraint>(termStateInpuLagrangianPtr->get().clone());
  }

  void testInputLimitsViolation(const scalar_array_t& timeTrajectory, const std::vector<LagrangianMetricsConstRef>& termMetrics) const {
    for (size_t i = 0; i < timeTrajectory.size(); i++) {
      const vector_t constraintViolation = termMetrics[i].constraint.cwiseMin(0.0);
      EXPECT_NEAR(constraintViolation(0), 0.0, constraintViolationTolerance)
          << "Input lower limit is violated at time " + std::to_string(timeTrajectory[i]);
      EXPECT_NEAR(constraintViolation(0), 0.0, constraintViolationTolerance)
          << "Input upper limit is violated at time " + std::to_string(timeTrajectory[i]);
    }
  }

  void testFinalState(const PrimalSolution& primalSolution) const {
    const auto& finalState = primalSolution.stateTrajectory_.back();
    const auto& desiredState = cartPoleInterfacePtr->getInitialTarget();
    EXPECT_NEAR(finalState(0), desiredState(0), goalViolationTolerance) << "Pole final angle is not " + std::to_string(desiredState(0));
    EXPECT_NEAR(finalState(1), desiredState(1), goalViolationTolerance) << "Cart final position is not " + std::to_string(desiredState(1));
    EXPECT_NEAR(finalState(2), desiredState(2), goalViolationTolerance) << "Pole final velocity is not " + std::to_string(desiredState(2));
    EXPECT_NEAR(finalState(3), desiredState(3), goalViolationTolerance) << "Cart final velocity is not " + std::to_string(desiredState(3));
  }

  const std::string inputLimitsName = "InputLimits";
  TargetTrajectories initTargetTrajectories;
  std::unique_ptr<CartPoleInterface> cartPoleInterfacePtr;

 private:
  std::string taskFile;
  std::unique_ptr<StateInputConstraint> inputLimitsConstraint;
};

constexpr scalar_t TestCartpole::timeHorizon;
constexpr scalar_t TestCartpole::goalViolationTolerance;
constexpr scalar_t TestCartpole::constraintViolationTolerance;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(TestCartpole, testDDP) {
  // construct solver
  auto ddpPtr = getAlgorithm();

  // set TargetTrajectories
  ddpPtr->getReferenceManager().setTargetTrajectories(initTargetTrajectories);

  // observer for InputLimits violation
  auto inputLimitsObserverModulePtr = SolverObserver::LagrangianTermObserver(
      SolverObserver::Type::Intermediate, "InputLimits",
      [&](const scalar_array_t& timeTrajectory, const std::vector<LagrangianMetricsConstRef>& termMetrics) {
        testInputLimitsViolation(timeTrajectory, termMetrics);
      });
  ddpPtr->addSolverObserver(std::move(inputLimitsObserverModulePtr));

  // run solver
  ddpPtr->run(0.0, cartPoleInterfacePtr->getInitialState(), timeHorizon);

  // test final state
  testFinalState(ddpPtr->primalSolution(timeHorizon));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/* Test name printed in gtest results */
std::string testName(const testing::TestParamInfo<TestCartpole::ParamType>& info) {
  std::string name;
  name += ddp::toAlgorithmName(std::get<0>(info.param)) + "_";

  switch (std::get<1>(info.param)) {
    case PenaltyType::SlacknessSquaredHingePenalty:
      name += "SlacknessSquaredHingePenalty";
      break;
    case PenaltyType::ModifiedRelaxedBarrierPenalty:
      name += "ModifiedRelaxedBarrierPenalty";
      break;
  }

  return name;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(TestCaseCartpole, TestCartpole,
                        testing::Combine(testing::ValuesIn({ddp::Algorithm::SLQ, ddp::Algorithm::ILQR}),
                                         testing::ValuesIn({PenaltyType::SlacknessSquaredHingePenalty,
                                                            PenaltyType::ModifiedRelaxedBarrierPenalty})),
                        testName);
