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

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ballbot_example/BallbotInterface.h>
#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_ballbot_example/ros_comm/BallbotDummyVisualization.h>
#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

int main(int argc, char* argv[]) {
  const std::string robotName = "ballbot";
  using mrt_t = ocs2::MRT_ROS_Interface;
  using dummy_t = ocs2::MRT_ROS_Dummy_Loop;
  using vis_t = ocs2::ballbot::BallbotDummyVisualization;

  // task file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task tile specified.");
  }
  const std::string taskFileFolderName(programArgs[1]);

  // Init ros
  ros::init(argc, argv, robotName + "_raisim_dummy");
  ros::NodeHandle nodeHandle;

  // read urdf file
  const std::string urdfParamName = "/ocs2_ballbot_raisim_description";
  std::string urdf;
  if (!ros::param::get(urdfParamName, urdf)) {
    throw ros::Exception("Error reading urdf from parameter server: " + urdfParamName);
  }

  // setup raisim rollout
  ocs2::RaisimRolloutSettings raisimRolloutSettings(ros::package::getPath("ocs2_ballbot_raisim_example") + "/config/raisim_rollout.info",
                                                    "rollout", true);
  ocs2::ballbot::BallbotRaisimConversions conversions;
  ocs2::RaisimRollout simRollout(
      urdf,
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      nullptr, raisimRolloutSettings, nullptr);

  // Terrain
  std::unique_ptr<ocs2::RaisimHeightmapRosConverter> heightmapPub;  // keep object alive until end of program
  if (raisimRolloutSettings.generateTerrain_) {
    raisim::TerrainProperties terrainProperties;
    terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
    terrainProperties.seed = raisimRolloutSettings.terrainSeed_;
    auto terrain = simRollout.generateTerrain(terrainProperties);
    conversions.terrain_ = terrain;

    // ensure height at zero is zero
    const auto heightAtZero = terrain->getHeight(0.0, 0.0);
    for (auto& height : terrain->getHeightMap()) {
      height -= heightAtZero;
    }

    heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
    heightmapPub->publishGridmap(*terrain);
  }

  // setup MRT
  mrt_t mrt(robotName);
  mrt.initRollout(&simRollout);
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<vis_t> ballbotDummyVisualization(new vis_t(nodeHandle));

  // Dummy
  ocs2::ballbot::BallbotInterface interface(taskFileFolderName);
  dummy_t dummyBallbot(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  dummyBallbot.subscribeObservers({ballbotDummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(ocs2::ballbot::INPUT_DIM);
  initObservation.time = 0.0;

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories({initObservation.time}, {initObservation.state}, {initObservation.input});
  // run dummy
  dummyBallbot.run(initObservation, initTargetTrajectories);

  return 0;
}
