#include "ocs2_ballbot_mpcnet/BallbotMpcnetInterface.h"

#include <ros/package.h>

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mpcnet/control/MpcnetOnnxController.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "ocs2_ballbot_mpcnet/BallbotMpcnetDefinition.h"

namespace ocs2 {
namespace ballbot {

BallbotMpcnetInterface::BallbotMpcnetInterface(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads) {
  // create ONNX environment
  auto onnxEnvironmentPtr = createOnnxEnvironment();
  // path to config file
  std::string taskFile = ros::package::getPath("ocs2_ballbot") + "/config/mpc/task.info";
  // path to save auto-generated libraries
  std::string libraryFolder = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  // set up MPC-Net rollout manager for data generation and policy evaluation
  std::vector<std::unique_ptr<MPC_BASE>> mpcPtrs;
  std::vector<std::unique_ptr<MpcnetControllerBase>> mpcnetPtrs;
  std::vector<std::unique_ptr<RolloutBase>> rolloutPtrs;
  std::vector<std::shared_ptr<MpcnetDefinitionBase>> mpcnetDefinitionPtrs;
  std::vector<std::shared_ptr<ReferenceManagerInterface>> referenceManagerPtrs;
  mpcPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  mpcnetPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  rolloutPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  mpcnetDefinitionPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  referenceManagerPtrs.reserve(nDataGenerationThreads + nPolicyEvaluationThreads);
  for (int i = 0; i < (nDataGenerationThreads + nPolicyEvaluationThreads); i++) {
    BallbotInterface ballbotInterface(taskFile, libraryFolder);
    std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr(new BallbotMpcnetDefinition());
    mpcPtrs.push_back(getMpc(ballbotInterface));
    mpcnetPtrs.push_back(std::unique_ptr<MpcnetControllerBase>(
        new MpcnetOnnxController(mpcnetDefinitionPtr, ballbotInterface.getReferenceManagerPtr(), onnxEnvironmentPtr)));
    rolloutPtrs.push_back(std::unique_ptr<RolloutBase>(ballbotInterface.getRollout().clone()));
    mpcnetDefinitionPtrs.push_back(mpcnetDefinitionPtr);
    referenceManagerPtrs.push_back(ballbotInterface.getReferenceManagerPtr());
  }
  mpcnetRolloutManagerPtr_.reset(new MpcnetRolloutManager(nDataGenerationThreads, nPolicyEvaluationThreads, std::move(mpcPtrs),
                                                          std::move(mpcnetPtrs), std::move(rolloutPtrs), mpcnetDefinitionPtrs,
                                                          referenceManagerPtrs));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_BASE> BallbotMpcnetInterface::getMpc(BallbotInterface& ballbotInterface) {
  std::unique_ptr<MPC_BASE> mpcPtr(new MPC_DDP(ballbotInterface.mpcSettings(), ballbotInterface.ddpSettings(),
                                               ballbotInterface.getRollout(), ballbotInterface.getOptimalControlProblem(),
                                               ballbotInterface.getInitializer()));
  mpcPtr->getSolverPtr()->setReferenceManager(ballbotInterface.getReferenceManagerPtr());
  return mpcPtr;
}

}  // namespace ballbot
}  // namespace ocs2
