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

#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include <ros/subscriber.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace ocs2 {

/**
 * This class lets the user to insert robot command form command line.
 */
class TargetTrajectoriesKeyboardPublisher final {
 public:
  using CommandLineToTargetTrajectories =
      std::function<TargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation)>;

  /**
   * Constructor
   *
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_target" topic. Moreover, the latest
   * observation is be expected on "topicPrefix_mpc_observation" topic.
   * @param [in] targetCommandLimits: The limits of the loaded command from command-line (for safety purposes).
   * @param [in] commandLineToTargetTrajectoriesFun: A function which transforms the command line input to TargetTrajectories.
   */
  TargetTrajectoriesKeyboardPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                      const scalar_array_t& targetCommandLimits,
                                      CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun);

  /** Gets the command vector size. */
  size_t targetCommandSize() const { return targetCommandLimits_.size(); }

  /**
   * Publishes command line input. If the input command is shorter than the expected command
   * size (targetCommandSize), the method will set the rest of the command to zero.
   *
   * @param [in] commadMsg: Message to be displayed on screen.
   */
  void publishKeyboardCommand(const std::string& commadMsg = "Enter command, separated by space");

 private:
  /** Gets the target from command line. */
  vector_t getCommandLine();

  const vector_t targetCommandLimits_;
  CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

  ::ros::Subscriber observationSubscriber_;
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace ocs2
