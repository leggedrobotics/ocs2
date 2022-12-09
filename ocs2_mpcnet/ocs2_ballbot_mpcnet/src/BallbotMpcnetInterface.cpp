/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include "ocs2_ballbot_mpcnet/BallbotMpcnetInterface.h"

#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpcnet_core/control/MpcnetOnnxController.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "ocs2_ballbot_mpcnet/BallbotMpcnetDefinition.h"

namespace ocs2 {
namespace ballbot {

BallbotMpcnetInterface::BallbotMpcnetInterface(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads, bool raisim) {
  // create ONNX environment
  auto onnxEnvironmentPtr = ocs2::mpcnet::createOnnxEnvironment();
  // path to config file
  const std::string taskFile = ros::package::getPath("ocs2_ballbot") + "/config/mpc/task.info";
  // path to save auto-generated libraries
  const std::string libraryFolder = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  // set up MPC-Net rollout manager for data generation and policy evaluation
  std::vector<std::unique_ptr<MPC_BASE>> mpcPtrs;
  std::vector<std::unique_ptr<ocs2::mpcnet::MpcnetControllerBase>> mpcnetPtrs;
  std::vector<std::unique_ptr<RolloutBase>> rolloutPtrs;
  std::vector<std::shared_ptr<ocs2::mpcnet::MpcnetDefinitionBase>> mpcnetDefinitionPtrs;
  std::vector<std::shared_ptr<ReferenceManagerInterface>> referenceManagerPtrs;
  mpcPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  mpcnetPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  rolloutPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  mpcnetDefinitionPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  referenceManagerPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  for (int i = 0; i < (nDataGenerationThreads + nPolicyEvaluationThreads); i++) {
    BallbotInterface ballbotInterface(taskFile, libraryFolder);
    auto mpcnetDefinitionPtr = std::make_shared<BallbotMpcnetDefinition>();
    mpcPtrs.push_back(getMpc(ballbotInterface));
    mpcnetPtrs.push_back(std::make_unique<ocs2::mpcnet::MpcnetOnnxController>(
        mpcnetDefinitionPtr, ballbotInterface.getReferenceManagerPtr(), onnxEnvironmentPtr));
    if (raisim) {
      throw std::runtime_error("[BallbotMpcnetInterface::BallbotMpcnetInterface] raisim rollout not yet implemented for ballbot.");
    } else {
      rolloutPtrs.push_back(std::unique_ptr<RolloutBase>(ballbotInterface.getRollout().clone()));
    }
    mpcnetDefinitionPtrs.push_back(mpcnetDefinitionPtr);
    referenceManagerPtrs.push_back(ballbotInterface.getReferenceManagerPtr());
  }
  mpcnetRolloutManagerPtr_.reset(new ocs2::mpcnet::MpcnetRolloutManager(nDataGenerationThreads, nPolicyEvaluationThreads,
                                                                        std::move(mpcPtrs), std::move(mpcnetPtrs), std::move(rolloutPtrs),
                                                                        mpcnetDefinitionPtrs, referenceManagerPtrs));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_BASE> BallbotMpcnetInterface::getMpc(BallbotInterface& ballbotInterface) {
  // ensure MPC and DDP settings are as needed for MPC-Net
  const auto mpcSettings = [&]() {
    auto settings = ballbotInterface.mpcSettings();
    settings.debugPrint_ = false;
    settings.coldStart_ = false;
    return settings;
  }();
  const auto ddpSettings = [&]() {
    auto settings = ballbotInterface.ddpSettings();
    settings.algorithm_ = ocs2::ddp::Algorithm::SLQ;
    settings.nThreads_ = 1;
    settings.displayInfo_ = false;
    settings.displayShortSummary_ = false;
    settings.checkNumericalStability_ = false;
    settings.debugPrintRollout_ = false;
    settings.useFeedbackPolicy_ = true;
    return settings;
  }();
  // create one MPC instance
  auto mpcPtr = std::make_unique<GaussNewtonDDP_MPC>(mpcSettings, ddpSettings, ballbotInterface.getRollout(),
                                                     ballbotInterface.getOptimalControlProblem(), ballbotInterface.getInitializer());
  mpcPtr->getSolverPtr()->setReferenceManager(ballbotInterface.getReferenceManagerPtr());
  return mpcPtr;
}

}  // namespace ballbot
}  // namespace ocs2
