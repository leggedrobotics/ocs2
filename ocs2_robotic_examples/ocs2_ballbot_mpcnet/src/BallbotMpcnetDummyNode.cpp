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