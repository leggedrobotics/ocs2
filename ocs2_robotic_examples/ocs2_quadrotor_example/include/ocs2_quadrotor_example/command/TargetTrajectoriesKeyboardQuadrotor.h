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

#include <ocs2_quadrotor_example/definitions.h>
#include <ocs2_ros_interfaces/command/TargetPoseTransformation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardInterface.h>

namespace ocs2 {
namespace quadrotor {

/**
 * This class implements TargetTrajectories communication using ROS.
 */
class TargetTrajectoriesKeyboardQuadrotor final : public ocs2::TargetTrajectoriesKeyboardInterface {
 public:
  enum { COMMAND_DIM = 12 };

  /**
   * Constructor.
   *
   * @param robotName: The robot's name.
   * @param goalPoseLimit: Limits for the input command. It has size 12 with following entries.
   *
   * goalPoseLimit(0): X
   * goalPoseLimit(1): Y
   * goalPoseLimit(2): Z
   *
   * goalPoseLimit(3): Roll
   * goalPoseLimit(4): Pitch
   * goalPoseLimit(5): Yaw
   *
   * goalPoseLimit(6): v_X
   * goalPoseLimit(7): v_Y
   * goalPoseLimit(8): v_Z
   *
   * goalPoseLimit(9): \omega_X
   * goalPoseLimit(10): \omega_Y
   * goalPoseLimit(11): \omega_Z
   */
  TargetTrajectoriesKeyboardQuadrotor(int argc, char* argv[], const std::string& robotName = "robot",
                                      const scalar_array_t& goalPoseLimit = scalar_array_t{10.0, 10.0, 10.0, 90.0, 90.0, 360.0, 2.0, 2.0,
                                                                                           2.0, 2.0, 2.0, 2.0})
      : TargetTrajectoriesKeyboardInterface(argc, argv, robotName, COMMAND_DIM, goalPoseLimit) {
    observationSubscriber_ = this->nodeHandle_->subscribe("/" + robotName + "_mpc_observation", 1,
                                                          &TargetTrajectoriesKeyboardQuadrotor::observationCallback, this);
  }

  /**
   * Default destructor
   */
  ~TargetTrajectoriesKeyboardQuadrotor() override = default;

  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = msg;
  }

  CostDesiredTrajectories toCostDesiredTrajectories(const scalar_array_t& commadLineTarget) override {
    SystemObservation observation;
    ::ros::spinOnce();
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      ros_msg_conversions::readObservationMsg(*latestObservation_, observation);
    }

    // reversing the order of the position and orientation.
    scalar_array_t commadLineTargetOrderCorrected(COMMAND_DIM);
    for (size_t j = 0; j < 3; j++) {
      // pose
      commadLineTargetOrderCorrected[j] = commadLineTarget[3 + j];
      commadLineTargetOrderCorrected[3 + j] = commadLineTarget[j];
      // velocities
      commadLineTargetOrderCorrected[6 + j] = commadLineTarget[9 + j];
      commadLineTargetOrderCorrected[9 + j] = commadLineTarget[6 + j];
    }

    // relative state target
    vector_t desiredStateRelative;
    target_pose_transformation::toCostDesiredState(commadLineTargetOrderCorrected, desiredStateRelative);

    // target transformation
    Eigen::Matrix<scalar_t, target_pose_transformation::POSE_DIM_, 1> targetPoseDisplacement, targetVelocity;
    target_pose_transformation::toTargetPoseDisplacement(desiredStateRelative, targetPoseDisplacement, targetVelocity);

    // reversing the order of the position and orientation.
    {
      Eigen::Matrix<scalar_t, 3, 1> temp;
      temp = targetPoseDisplacement.head<3>();
      targetPoseDisplacement.head<3>() = targetPoseDisplacement.tail<3>();
      targetPoseDisplacement.tail<3>() = temp;
      temp = targetVelocity.head<3>();
      targetVelocity.head<3>() = targetVelocity.tail<3>();
      targetVelocity.tail<3>() = temp;
    }

    // targetReachingDuration
    const scalar_t averageSpeed = 2.0;
    scalar_t targetReachingDuration1 = targetPoseDisplacement.norm() / averageSpeed;
    const scalar_t averageAcceleration = 10.0;
    scalar_t targetReachingDuration2 = targetVelocity.norm() / averageAcceleration;
    scalar_t targetReachingDuration = std::max(targetReachingDuration1, targetReachingDuration2);

    CostDesiredTrajectories costDesiredTrajectories(2);
    // Desired time trajectory
    scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
    tDesiredTrajectory.resize(2);
    tDesiredTrajectory[0] = observation.time;
    tDesiredTrajectory[1] = observation.time + targetReachingDuration;

    // Desired state trajectory
    auto& xDesiredTrajectory = costDesiredTrajectories.desiredStateTrajectory();
    xDesiredTrajectory.resize(2);
    xDesiredTrajectory[0].setZero(STATE_DIM);
    xDesiredTrajectory[0].segment<6>(0) = observation.state.segment<6>(0);
    xDesiredTrajectory[0].segment<6>(6) = observation.state.segment<6>(6);

    xDesiredTrajectory[1].resize(STATE_DIM);
    xDesiredTrajectory[1].setZero();
    xDesiredTrajectory[1].segment<6>(0) = observation.state.segment<6>(0) + targetPoseDisplacement;
    xDesiredTrajectory[1].segment<6>(6) = targetVelocity;

    // Desired input trajectory
    costDesiredTrajectories.desiredInputTrajectory().resize(2);
    costDesiredTrajectories.desiredInputTrajectory()[0] = vector_t::Zero(INPUT_DIM);
    costDesiredTrajectories.desiredInputTrajectory()[1] = vector_t::Zero(INPUT_DIM);

    return costDesiredTrajectories;
  }

 private:
  ros::Subscriber observationSubscriber_;

  std::mutex latestObservationMutex_;
  ocs2_msgs::mpc_observation::ConstPtr latestObservation_;
};

}  // namespace quadrotor
}  // namespace ocs2
