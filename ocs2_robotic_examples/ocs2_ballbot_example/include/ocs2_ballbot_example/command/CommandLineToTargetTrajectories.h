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

#pragma once

#include <mutex>

#include <ros/subscriber.h>

#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_msgs/mpc_observation.h>

namespace ocs2 {
namespace ballbot {

/**
 * This class changes the command line input to TargetTrajectories.
 */
class CommandLineToTargetTrajectories {
 public:
  /**
   * Constructor.
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_observation" topic.
   */
  CommandLineToTargetTrajectories(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix) {
    observationSubscriber_ =
        nodeHandle.subscribe("/" + topicPrefix + "_mpc_observation", 1, &CommandLineToTargetTrajectories::observationCallback, this);
  }

  /**
   * goalPose: [X, Y, Yaw, v_X, v_Y, \omega_Z]
   */
  TargetTrajectories compute(const vector_t& commadLineTarget) {
    auto deg2rad = [](scalar_t deg) { return (deg * M_PI / 180.0); };

    SystemObservation observation;
    ::ros::spinOnce();
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // desired state from command line (position is relative, velocity absolute)
    vector_t relativeState = commadLineTarget;
    relativeState(2) = deg2rad(commadLineTarget[2]);

    // Target reaching duration
    constexpr scalar_t averageSpeed = 2.0;
    const scalar_t targetReachingDuration1 = relativeState.head<3>().norm() / averageSpeed;
    constexpr scalar_t averageAcceleration = 10.0;
    const scalar_t targetReachingDuration2 = relativeState.tail<3>().norm() / averageAcceleration;
    const scalar_t targetReachingDuration = std::max(targetReachingDuration1, targetReachingDuration2);

    TargetTrajectories targetTrajectories(2);

    // Desired time trajectory
    targetTrajectories.timeTrajectory[0] = observation.time;
    targetTrajectories.timeTrajectory[1] = observation.time + targetReachingDuration;

    // Desired state trajectory
    targetTrajectories.stateTrajectory[0] = observation.state;
    targetTrajectories.stateTrajectory[1] = observation.state;
    targetTrajectories.stateTrajectory[1].head<3>() += relativeState.head<3>();
    targetTrajectories.stateTrajectory[1].tail<5>() << relativeState.tail<3>(), 0.0, 0.0;

    // Desired input trajectory
    targetTrajectories.inputTrajectory[0].setZero(3);
    targetTrajectories.inputTrajectory[1].setZero(3);

    return targetTrajectories;
  }

 private:
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    ros_msg_conversions::readObservationMsg(*msg, latestObservation_);
  }

  ::ros::Subscriber observationSubscriber_;

  std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace ballbot
}  // namespace ocs2
