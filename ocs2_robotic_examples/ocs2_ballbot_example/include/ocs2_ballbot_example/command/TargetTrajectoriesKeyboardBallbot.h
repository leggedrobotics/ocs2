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
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardInterface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {
namespace ballbot {

/**
 * This class implements TargetTrajectories communication using ROS.
 */
class TargetTrajectoriesKeyboardBallbot final : public TargetTrajectoriesKeyboardInterface {
 public:
  enum { COMMAND_DIM = 6 };

  /**
   * Constructor.
   *
   * @param robotName: The robot's name.
   * @param goalPoseLimit: Limits for the input command. It has size 12 with following entries.
   *
   * goalPoseLimit(0): X
   * goalPoseLimit(1): Y
   * goalPoseLimit(2): Yaw
   *
   * goalPoseLimit(3): v_X
   * goalPoseLimit(4): v_Y
   * goalPoseLimit(5): \omega_Z
   */
  TargetTrajectoriesKeyboardBallbot(int argc, char* argv[], const std::string& robotName = "robot",
                                    const scalar_array_t& goalPoseLimit = scalar_array_t{2.0, 2.0, 360.0, 2.0, 2.0, 2.0})
      : TargetTrajectoriesKeyboardInterface(argc, argv, robotName, COMMAND_DIM, goalPoseLimit) {
    observationSubscriber_ = this->nodeHandle_->subscribe("/" + robotName + "_mpc_observation", 1,
                                                          &TargetTrajectoriesKeyboardBallbot::observationCallback, this);
  }

  /**
   * Default destructor
   */
  ~TargetTrajectoriesKeyboardBallbot() override = default;

  CostDesiredTrajectories toCostDesiredTrajectories(const scalar_array_t& commadLineTarget) override {
    auto deg2rad = [](scalar_t deg) { return (deg * M_PI / 180.0); };

    SystemObservation observation;
    ::ros::spinOnce();
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      ros_msg_conversions::readObservationMsg(*latestObservation_, observation);
    }

    // desired state from command line (position is relative, velocity absolute)
    vector_t relativeState = Eigen::Map<const vector_t>(commadLineTarget.data(), COMMAND_DIM);
    relativeState(2) = deg2rad(commadLineTarget[2]);

    // Target reaching duration
    const scalar_t averageSpeed = 2.0;
    scalar_t targetReachingDuration1 = relativeState.template head<3>().norm() / averageSpeed;
    const scalar_t averageAcceleration = 10.0;
    scalar_t targetReachingDuration2 = relativeState.template tail<3>().norm() / averageAcceleration;
    scalar_t targetReachingDuration = std::max(targetReachingDuration1, targetReachingDuration2);

    // Desired time trajectory
    CostDesiredTrajectories costDesiredTrajectories(2);
    scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
    tDesiredTrajectory.resize(2);
    tDesiredTrajectory[0] = observation.time;
    tDesiredTrajectory[1] = observation.time + targetReachingDuration;

    // Desired state trajectory
    vector_array_t& xDesiredTrajectory = costDesiredTrajectories.desiredStateTrajectory();
    xDesiredTrajectory.resize(2);
    xDesiredTrajectory[0] = observation.state;
    xDesiredTrajectory[1] = observation.state;
    xDesiredTrajectory[1].template head<3>() += relativeState.template head<3>();
    xDesiredTrajectory[1].template tail<5>() << relativeState.template tail<3>(), 0.0, 0.0;

    // Desired input trajectory
    vector_array_t& uDesiredTrajectory = costDesiredTrajectories.desiredInputTrajectory();
    uDesiredTrajectory.resize(2);
    uDesiredTrajectory[0].setZero(3);
    uDesiredTrajectory[1].setZero(3);

    return costDesiredTrajectories;
  }

  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = msg;
  }

 private:
  ros::Subscriber observationSubscriber_;

  std::mutex latestObservationMutex_;
  ocs2_msgs::mpc_observation::ConstPtr latestObservation_;
};

}  // namespace ballbot
}  // namespace ocs2
