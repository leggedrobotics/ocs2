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

#include <ocs2_cartpole/CartPoleInterface.h>
#include <ocs2_cartpole/definitions.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ocs2_cartpole_ros/CartpoleDummyVisualization.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  const std::string robotName = "cartpole";

  // task file
  std::vector<std::string> programArgs =
      rclcpp::remove_ros_arguments(argc, argv);

  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_mrt");

  // Robot interface
  const std::string taskFile =
      ament_index_cpp::get_package_share_directory("ocs2_cartpole") +
      "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder =
      ament_index_cpp::get_package_share_directory("ocs2_cartpole") +
      "/auto_generated";
  ocs2::cartpole::CartPoleInterface cartPoleInterface(taskFile, libFolder,
                                                      false /*verbose*/);

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&cartPoleInterface.getRollout());
  mrt.launchNodes(node);

  // Visualization
  auto cartpoleDummyVisualization =
      std::make_shared<ocs2::cartpole::CartpoleDummyVisualization>(node);

  // Dummy loop
  ocs2::MRT_ROS_Dummy_Loop dummyCartpole(
      mrt, cartPoleInterface.mpcSettings().mrtDesiredFrequency_,
      cartPoleInterface.mpcSettings().mpcDesiredFrequency_);
  dummyCartpole.subscribeObservers({cartpoleDummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = cartPoleInterface.getInitialState();
  initObservation.input.setZero(ocs2::cartpole::INPUT_DIM);
  initObservation.time = 0.0;

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories(
      {0.0}, {cartPoleInterface.getInitialTarget()},
      {ocs2::vector_t::Zero(ocs2::cartpole::INPUT_DIM)});

  // Run dummy (loops while ros is ok)
  dummyCartpole.run(initObservation, initTargetTrajectories);

  return 0;
}
