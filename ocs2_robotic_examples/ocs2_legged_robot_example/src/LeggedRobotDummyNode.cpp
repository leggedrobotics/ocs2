/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_legged_robot_example/visualization/LeggedRobotDummyVisualization.h>
#include "ocs2_legged_robot_example/LeggedRobotInterface.h"

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <urdf_parser/urdf_parser.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, ROBOT_NAME_ + "_mrt");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  const std::string descriptionName = "/legged_robot_description";
  if (!ros::param::get(descriptionName, urdfString)) {
    std::cerr << "Param " << descriptionName << " not found; unable to generate urdf" << std::endl;
  }

  LeggedRobotInterface leggedRobotInterface(taskName, urdf::parseURDF(urdfString));

  // MRT
  ocs2::MRT_ROS_Interface mrt(ROBOT_NAME_);
  mrt.initRollout(&leggedRobotInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<LeggedRobotDummyVisualization> leggedRobotDummyVisualization(
      new LeggedRobotDummyVisualization(leggedRobotInterface.getPinocchioInterface(), nodeHandle));

  // Dummy legged robot
  ocs2::MRT_ROS_Dummy_Loop leggedRobotDummySimulator(mrt, leggedRobotInterface.mpcSettings().mrtDesiredFrequency_,
                                                     leggedRobotInterface.mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummySimulator.subscribeObservers({leggedRobotDummyVisualization});

  // Initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = leggedRobotInterface.getInitialState();
  initObservation.input = vector_t::Zero(centroidalModelInfo.inputDim);
  initObservation.mode = ModeNumber::STANCE;

  // Initial command
  CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state}, {initObservation.input});

  // run dummy
  leggedRobotDummySimulator.run(initObservation, initCostDesiredTrajectories);

  // Successful exit
  return 0;
}
