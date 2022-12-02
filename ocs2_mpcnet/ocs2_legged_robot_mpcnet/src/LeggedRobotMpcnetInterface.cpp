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

#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetInterface.h"

#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpcnet_core/control/MpcnetOnnxController.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_raisim_core/RaisimRollout.h>
#include <ocs2_raisim_core/RaisimRolloutSettings.h>

#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetDefinition.h"

namespace ocs2 {
namespace legged_robot {

LeggedRobotMpcnetInterface::LeggedRobotMpcnetInterface(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads, bool raisim) {
  // create ONNX environment
  auto onnxEnvironmentPtr = ocs2::mpcnet::createOnnxEnvironment();
  // paths to files
  const std::string taskFile = ros::package::getPath("ocs2_legged_robot") + "/config/mpc/task.info";
  const std::string urdfFile = ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/urdf/anymal.urdf";
  const std::string referenceFile = ros::package::getPath("ocs2_legged_robot") + "/config/command/reference.info";
  const std::string raisimFile = ros::package::getPath("ocs2_legged_robot_raisim") + "/config/raisim.info";
  const std::string resourcePath = ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/meshes";
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
    leggedRobotInterfacePtrs_.push_back(std::make_unique<LeggedRobotInterface>(taskFile, urdfFile, referenceFile));
    auto mpcnetDefinitionPtr = std::make_shared<LeggedRobotMpcnetDefinition>(*leggedRobotInterfacePtrs_[i]);
    mpcPtrs.push_back(getMpc(*leggedRobotInterfacePtrs_[i]));
    mpcnetPtrs.push_back(std::unique_ptr<ocs2::mpcnet::MpcnetControllerBase>(new ocs2::mpcnet::MpcnetOnnxController(
        mpcnetDefinitionPtr, leggedRobotInterfacePtrs_[i]->getReferenceManagerPtr(), onnxEnvironmentPtr)));
    if (raisim) {
      RaisimRolloutSettings raisimRolloutSettings(raisimFile, "rollout");
      raisimRolloutSettings.portNumber_ += i;
      leggedRobotRaisimConversionsPtrs_.push_back(std::make_unique<LeggedRobotRaisimConversions>(
          leggedRobotInterfacePtrs_[i]->getPinocchioInterface(), leggedRobotInterfacePtrs_[i]->getCentroidalModelInfo(),
          leggedRobotInterfacePtrs_[i]->getInitialState()));
      leggedRobotRaisimConversionsPtrs_[i]->loadSettings(raisimFile, "rollout", true);
      rolloutPtrs.push_back(std::make_unique<RaisimRollout>(
          urdfFile, resourcePath,
          [&, i](const vector_t& state, const vector_t& input) {
            return leggedRobotRaisimConversionsPtrs_[i]->stateToRaisimGenCoordGenVel(state, input);
          },
          [&, i](const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
            return leggedRobotRaisimConversionsPtrs_[i]->raisimGenCoordGenVelToState(q, dq);
          },
          [&, i](double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
            return leggedRobotRaisimConversionsPtrs_[i]->inputToRaisimGeneralizedForce(time, input, state, q, dq);
          },
          nullptr, raisimRolloutSettings,
          [&, i](double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
            return leggedRobotRaisimConversionsPtrs_[i]->inputToRaisimPdTargets(time, input, state, q, dq);
          }));
      if (raisimRolloutSettings.generateTerrain_) {
        raisim::TerrainProperties terrainProperties;
        terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
        terrainProperties.seed = raisimRolloutSettings.terrainSeed_ + i;
        auto terrainPtr = static_cast<RaisimRollout*>(rolloutPtrs[i].get())->generateTerrain(terrainProperties);
        leggedRobotRaisimConversionsPtrs_[i]->setTerrain(*terrainPtr);
      }
    } else {
      rolloutPtrs.push_back(std::unique_ptr<RolloutBase>(leggedRobotInterfacePtrs_[i]->getRollout().clone()));
    }
    mpcnetDefinitionPtrs.push_back(mpcnetDefinitionPtr);
    referenceManagerPtrs.push_back(leggedRobotInterfacePtrs_[i]->getReferenceManagerPtr());
  }
  mpcnetRolloutManagerPtr_.reset(new ocs2::mpcnet::MpcnetRolloutManager(nDataGenerationThreads, nPolicyEvaluationThreads,
                                                                        std::move(mpcPtrs), std::move(mpcnetPtrs), std::move(rolloutPtrs),
                                                                        mpcnetDefinitionPtrs, referenceManagerPtrs));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_BASE> LeggedRobotMpcnetInterface::getMpc(LeggedRobotInterface& leggedRobotInterface) {
  // ensure MPC and DDP settings are as needed for MPC-Net
  const auto mpcSettings = [&]() {
    auto settings = leggedRobotInterface.mpcSettings();
    settings.debugPrint_ = false;
    settings.coldStart_ = false;
    return settings;
  }();
  const auto ddpSettings = [&]() {
    auto settings = leggedRobotInterface.ddpSettings();
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
  auto mpcPtr =
      std::make_unique<GaussNewtonDDP_MPC>(mpcSettings, ddpSettings, leggedRobotInterface.getRollout(),
                                           leggedRobotInterface.getOptimalControlProblem(), leggedRobotInterface.getInitializer());
  mpcPtr->getSolverPtr()->setReferenceManager(leggedRobotInterface.getReferenceManagerPtr());
  return mpcPtr;
}

}  // namespace legged_robot
}  // namespace ocs2
