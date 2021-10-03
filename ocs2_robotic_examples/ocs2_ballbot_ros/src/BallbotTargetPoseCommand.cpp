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

#include <ocs2_ballbot/definitions.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

using namespace ocs2;
using namespace ballbot;

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaYaw]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation) {
  const vector_t targetState = [&]() {
    vector_t targetState = vector_t::Zero(STATE_DIM);
    targetState(0) = observation.state(0) + commadLineTarget(0);
    targetState(1) = observation.state(1) + commadLineTarget(1);
    targetState(2) = observation.state(2) + commadLineTarget(2) * M_PI / 180.0;  //  deg2rad
    targetState(3) = observation.state(3);
    targetState(4) = observation.state(4);
    return targetState;
  }();

  // Target reaching duration
  constexpr scalar_t averageSpeed = 2.0;
  const vector_t deltaPose = (targetState - observation.state).head<5>();
  const scalar_t targetTime = observation.time + deltaPose.norm() / averageSpeed;

  // Desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetTime};

  // Desired state trajectory
  const vector_array_t stateTrajectory{observation.state, targetState};

  // Desired input trajectory
  const vector_array_t inputTrajectory(2, vector_t::Zero(INPUT_DIM));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * Main function
 */
int main(int argc, char* argv[]) {
  const std::string robotName = "ballbot";
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  // [deltaX, deltaY, deltaYaw]
  const scalar_array_t relativeStateLimit{10.0, 10.0, 360.0};
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeStateLimit, &commandLineToTargetTrajectories);

  const std::string commadMsg = "Enter XY and Yaw (deg) displacements, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
