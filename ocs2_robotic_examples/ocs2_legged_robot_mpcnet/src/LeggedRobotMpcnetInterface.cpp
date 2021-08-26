#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetInterface.h"

#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mpcnet/control/MpcnetOnnxController.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetDefinition.h"

namespace ocs2 {
namespace legged_robot {

LeggedRobotMpcnetInterface::LeggedRobotMpcnetInterface(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads) {
  // create ONNX environment
  auto onnxEnvironmentPtr = createOnnxEnvironment();
  // path to config files
  std::string taskFileFolderName = "mpc";
  std::string targetCommandFile = ros::package::getPath("ocs2_legged_robot") + "/config/command/targetTrajectories.info";
  // path to urdf file
  std::string urdfFile = ros::package::getPath("anymal_c_simple_description") + "/urdf/anymal.urdf";
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
    LeggedRobotInterface leggedRobotInterface(taskFileFolderName, targetCommandFile, urdf::parseURDFFile(urdfFile));
    std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr(new LeggedRobotMpcnetDefinition(leggedRobotInterface.getInitialState()));
    mpcPtrs.push_back(getMpc(leggedRobotInterface));
    mpcnetPtrs.push_back(std::unique_ptr<MpcnetControllerBase>(
        new MpcnetOnnxController(mpcnetDefinitionPtr, leggedRobotInterface.getReferenceManagerPtr(), onnxEnvironmentPtr)));
    rolloutPtrs.push_back(std::unique_ptr<RolloutBase>(leggedRobotInterface.getRollout().clone()));
    mpcnetDefinitionPtrs.push_back(mpcnetDefinitionPtr);
    referenceManagerPtrs.push_back(leggedRobotInterface.getReferenceManagerPtr());
  }
  mpcnetRolloutManagerPtr_.reset(new MpcnetRolloutManager(nDataGenerationThreads, nPolicyEvaluationThreads, std::move(mpcPtrs),
                                                          std::move(mpcnetPtrs), std::move(rolloutPtrs), mpcnetDefinitionPtrs,
                                                          referenceManagerPtrs));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_BASE> LeggedRobotMpcnetInterface::getMpc(LeggedRobotInterface& leggedRobotInterface) {
  std::unique_ptr<MPC_BASE> mpcPtr(new MPC_DDP(leggedRobotInterface.mpcSettings(), leggedRobotInterface.ddpSettings(),
                                               leggedRobotInterface.getRollout(), leggedRobotInterface.getOptimalControlProblem(),
                                               leggedRobotInterface.getInitializer()));
  mpcPtr->getSolverPtr()->setReferenceManager(leggedRobotInterface.getReferenceManagerPtr());
  return mpcPtr;
}

}  // namespace legged_robot
}  // namespace ocs2
