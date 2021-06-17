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

#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

using namespace ocs2;
using namespace ballbot;

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [X, Y, Yaw, v_X, v_Y, \omega_Z]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation) {
  // desired state from command line (position is relative, velocity absolute)
  const vector_t relativeState = [&commadLineTarget]() {
    vector_t relativeState = commadLineTarget;
    relativeState(2) = commadLineTarget[2] * M_PI / 180.0;  //  deg2rad
    return relativeState;
  }();

  // Target reaching duration
  constexpr scalar_t averageSpeed = 2.0;
  const scalar_t targetReachingDuration1 = relativeState.head<3>().norm() / averageSpeed;
  constexpr scalar_t averageAcceleration = 10.0;
  const scalar_t targetReachingDuration2 = relativeState.tail<3>().norm() / averageAcceleration;
  const scalar_t targetReachingDuration = std::max(targetReachingDuration1, targetReachingDuration2);

  constexpr size_t numPoints = 2;

  // Desired time trajectory
  scalar_array_t timeTrajectory(numPoints);
  timeTrajectory[0] = observation.time;
  timeTrajectory[1] = observation.time + targetReachingDuration;

  // Desired state trajectory
  vector_array_t stateTrajectory(2);
  stateTrajectory[0] = observation.state;
  stateTrajectory[1] = observation.state;
  stateTrajectory[1].head<3>() += relativeState.head<3>();
  stateTrajectory[1].tail<5>() << relativeState.tail<3>(), 0.0, 0.0;

  // Desired input trajectory
  const vector_array_t inputTrajectory(numPoints, vector_t::Zero(INPUT_DIM));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * Main function
 */
int main(int argc, char* argv[]) {
  ::ros::init(argc, argv, "ballbot_target");
  ::ros::NodeHandle nodeHandle;
  const std::string topicPrefix = "ballbot";

  // goalPose: [X, Y, Yaw, v_X, v_Y, \omega_Z]
  const scalar_array_t relativeStateLimit{2.0, 2.0, 360.0, 2.0, 2.0, 2.0};
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, topicPrefix, relativeStateLimit.size(), relativeStateLimit,
                                                        &commandLineToTargetTrajectories);

  const std::string commadMsg = "Enter XY displacement and Yaw (deg) for the robot, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
