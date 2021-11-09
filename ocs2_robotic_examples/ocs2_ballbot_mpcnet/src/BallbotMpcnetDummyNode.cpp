#include <ros/package.h>
#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_ballbot/BallbotInterface.h>
#include <ocs2_ballbot_ros/BallbotDummyVisualization.h>
#include <ocs2_mpcnet/control/MpcnetOnnxController.h>
#include <ocs2_mpcnet/dummy/MpcnetDummyLoopRos.h>
#include <ocs2_mpcnet/dummy/MpcnetDummyObserverRos.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "ocs2_ballbot_mpcnet/BallbotMpcnetDefinition.h"

using namespace ocs2;
using namespace ballbot;

int main(int argc, char** argv) {
  const std::string robotName = "ballbot";

  // task and policy file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 2) {
    throw std::runtime_error("No task name and policy file path specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);
  std::string policyFilePath = std::string(programArgs[2]);

  // initialize ros node
  ros::init(argc, argv, robotName + "_mpcnet_dummy");
  ros::NodeHandle nodeHandle;

  // ballbot interface
  const std::string taskFile = ros::package::getPath("ocs2_ballbot") + "/config/" + taskFileFolderName + "/task.info";
  const std::string libraryFolder = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  BallbotInterface ballbotInterface(taskFile, libraryFolder);

  // ROS reference manager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, ballbotInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // policy (MPC-Net controller)
  auto onnxEnvironmentPtr = createOnnxEnvironment();
  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr(new BallbotMpcnetDefinition());
  std::unique_ptr<MpcnetControllerBase> mpcnetControllerPtr(
      new MpcnetOnnxController(mpcnetDefinitionPtr, rosReferenceManagerPtr, onnxEnvironmentPtr));
  mpcnetControllerPtr->loadPolicyModel(policyFilePath);

  // rollout
  std::unique_ptr<RolloutBase> rolloutPtr(ballbotInterface.getRollout().clone());

  // observer
  std::shared_ptr<MpcnetDummyObserverRos> mpcnetDummyObserverRosPtr(new MpcnetDummyObserverRos(nodeHandle, robotName));

  // visualization
  std::shared_ptr<BallbotDummyVisualization> ballbotDummyVisualization(new BallbotDummyVisualization(nodeHandle));

  // MPC-Net dummy loop ROS
  scalar_t controlFrequency = ballbotInterface.mpcSettings().mrtDesiredFrequency_;
  scalar_t rosFrequency = ballbotInterface.mpcSettings().mpcDesiredFrequency_;
  MpcnetDummyLoopRos mpcnetDummyLoopRos(controlFrequency, rosFrequency, std::move(mpcnetControllerPtr), std::move(rolloutPtr),
                                        rosReferenceManagerPtr);
  mpcnetDummyLoopRos.addObserver(mpcnetDummyObserverRosPtr);
  mpcnetDummyLoopRos.addObserver(ballbotDummyVisualization);

  // initial system observation
  SystemObservation systemObservation;
  systemObservation.mode = 0;
  systemObservation.time = 0.0;
  systemObservation.state = ballbotInterface.getInitialState();
  systemObservation.input = vector_t::Zero(ocs2::ballbot::INPUT_DIM);

  // initial target trajectories
  TargetTrajectories targetTrajectories({systemObservation.time}, {systemObservation.state}, {systemObservation.input});

  // run MPC-Net dummy loop ROS
  mpcnetDummyLoopRos.run(systemObservation, targetTrajectories);

  // successful exit
  return 0;
}