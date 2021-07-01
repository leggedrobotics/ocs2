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

#include <iostream>
#include <thread>

#include "ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h"

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesKeyboardPublisher::TargetTrajectoriesKeyboardPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                                         size_t targetCommandSize,
                                                                         const scalar_array_t& targetCommandLimits,
                                                                         CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun)
    : targetCommandSize_(targetCommandSize), commandLineToTargetTrajectoriesFun_(std::move(commandLineToTargetTrajectoriesFun)) {
  if (targetCommandLimits.size() != targetCommandSize_) {
    throw std::runtime_error("[TargetTrajectoriesKeyboardPublisher] Target command limits are not set properly!");
  }
  targetCommandLimits_ = Eigen::Map<const vector_t>(targetCommandLimits.data(), targetCommandSize_);

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishKeyboardCommand(const std::string& commadMsg) {
  while (ros::ok() && ros::master::check()) {
    // get command line
    std::cout << commadMsg << ": ";
    const vector_t commandLineInput = getCommandLine().cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);

    // display
    std::cout << "The following command is published: [";
    for (size_t i = 0; i < targetCommandSize_; i++) {
      std::cout << std::setprecision(4) << commandLineInput(i);
      if (i + 1 != targetCommandSize_) {
        std::cout << ", ";
      }
    }
    std::cout << "]\n\n";

    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(commandLineInput, observation);

    // publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t TargetTrajectoriesKeyboardPublisher::getCommandLine() {
  scalar_array_t targetCommand(0);

  // Set up a thread to read user inputs
  std::string line;
  bool lineRead;
  std::thread thr([&line, &lineRead]() {
    lineRead = false;
    getline(std::cin, line);
    lineRead = true;
  });

  // wait till line is read or terminate if ROS is gone.
  ::ros::WallRate rate(30);
  while (!lineRead) {
    if (!ros::ok() || !ros::master::check()) {
      std::terminate();  // Need to terminate thread that is still waiting for input
    }
    rate.sleep();
  }
  if (thr.joinable()) {
    thr.join();
  }

  std::istringstream stream(line);
  scalar_t in;
  while (stream >> in) {
    targetCommand.push_back(in);
  }

  // if the size is greater than targetCommandSize_
  const size_t n = targetCommand.size();
  if (n > targetCommandSize_) {
    targetCommand.erase(targetCommand.begin() + targetCommandSize_, targetCommand.end());
  } else {
    for (size_t i = n; i < targetCommandSize_; i++) {
      targetCommand.push_back(0.0);
    }  // end of i loop
  }

  const vector_t eigenTargetCommand = Eigen::Map<const vector_t>(targetCommand.data(), targetCommandSize_);

  return eigenTargetCommand;
}

}  // namespace ocs2
