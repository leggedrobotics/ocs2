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

#include <ocs2_quadrotor_example/definitions.h>
#include <ocs2_ros_interfaces/command/TargetPoseTransformation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

using namespace ocs2;
using namespace quadrotor;

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [X, Y, Yaw, v_X, v_Y, \omega_Z]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation) {
  // reversing the order of the position and orientation.
  const vector_t commadLineTargetOrderCorrected = [&]() {
    vector_t commadLineTargetOrderCorrected(STATE_DIM);
    // pose
    commadLineTargetOrderCorrected.segment<3>(0) = commadLineTarget.segment<3>(3);
    commadLineTargetOrderCorrected.segment<3>(3) = commadLineTarget.segment<3>(0);
    // velocities
    commadLineTargetOrderCorrected.segment<3>(6) = commadLineTarget.segment<3>(9);
    commadLineTargetOrderCorrected.segment<3>(9) = commadLineTarget.segment<3>(6);
    return commadLineTargetOrderCorrected;
  }();

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

  // target reaching duration
  constexpr scalar_t averageSpeed = 2.0;
  const scalar_t targetReachingDuration1 = targetPoseDisplacement.norm() / averageSpeed;
  constexpr scalar_t averageAcceleration = 10.0;
  const scalar_t targetReachingDuration2 = targetVelocity.norm() / averageAcceleration;
  const scalar_t targetReachingDuration = std::max(targetReachingDuration1, targetReachingDuration2);

  constexpr size_t numPoints = 2;

  // Desired time trajectory
  scalar_array_t timeTrajectory(numPoints);
  timeTrajectory[0] = observation.time;
  timeTrajectory[1] = observation.time + targetReachingDuration;

  // Desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(STATE_DIM));
  stateTrajectory[0].segment<6>(0) = observation.state.segment<6>(0);
  stateTrajectory[0].segment<6>(6) = observation.state.segment<6>(6);
  stateTrajectory[1].segment<6>(0) = observation.state.segment<6>(0) + targetPoseDisplacement;
  stateTrajectory[1].segment<6>(6) = targetVelocity;

  // Desired input trajectory
  const vector_array_t inputTrajectory(numPoints, vector_t::Zero(INPUT_DIM));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  const std::string robotName = "quadrotor";
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  // goalPose: [X, Y, Z, Roll, Pitch, Yaw, v_X, v_Y, v_Z, \omega_X, \omega_Y, \omega_Z]
  const scalar_array_t relativeStateLimit{10.0, 10.0, 10.0, 90.0, 90.0, 360.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeStateLimit.size(), relativeStateLimit,
                                                        &commandLineToTargetTrajectories);

  const std::string commadMsg = "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
