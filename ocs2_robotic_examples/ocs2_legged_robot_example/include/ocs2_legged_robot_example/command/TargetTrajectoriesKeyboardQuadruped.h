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

#pragma once

#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardInterface.h>

#include <iomanip>
#include <mutex>

namespace ocs2 {
namespace legged_robot {

/**
 * This class implements TargetTrajectories communication using ROS.
 */
class TargetTrajectoriesKeyboardQuadruped : public TargetTrajectoriesKeyboardInterface {
 public:
  enum class COMMAND_MODE { POSITION, VELOCITY };

  static constexpr size_t commandDim_ = 12;
  static constexpr size_t stateDim_ = 24;
  static constexpr size_t inputDim_ = 24;
  static constexpr size_t actuatedDofNum_ = 12;

  /**
   * Constructor.
   *
   * @param [in] robotName: The robot's name.
   * @param [in] goalPoseLimit: Limits for the input command. It has size 12 with following entries.
   * @param [in] command_mode: Whether to use position mode or velocity mode.
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
  TargetTrajectoriesKeyboardQuadruped(int argc, char* argv[], const std::string& robotName, scalar_t initZHeight,
                                      vector_t defaultJointCoordinates, scalar_t targetDisplacementVelocity,
                                      scalar_t targetRotationVelocity,
                                      const scalar_array_t& goalPoseLimit = scalar_array_t{2.0, 1.0, 0.3, 45.0, 45.0, 360.0, 0.5, 0.5, 0.5,
                                                                                           0.5, 0.5, 0.5},
                                      const COMMAND_MODE command_mode = COMMAND_MODE::POSITION)
      : TargetTrajectoriesKeyboardInterface(argc, argv, robotName, commandDim_, goalPoseLimit),
        command_mode_(command_mode),
        initZHeight_(initZHeight),
        defaultJointCoordinates_(defaultJointCoordinates),
        targetDisplacementVelocity_(targetDisplacementVelocity),
        targetRotationVelocity_(targetRotationVelocity) {
    observationSubscriber_ = this->nodeHandle_->subscribe("/" + robotName + "_mpc_observation", 1,
                                                          &TargetTrajectoriesKeyboardQuadruped::observationCallback, this);
  }

  ~TargetTrajectoriesKeyboardQuadruped() override = default;

  /**
   * From command line loaded command to desired time, state, and input.
   *
   * @param [out] commandLineTarget: The loaded command target.
   * @param [in] desiredTime: Desired time to be published.
   * @param [in] desiredState: Desired state to be published.
   * @param [in] desiredInput: Desired input to be published.
   */
  void toCostDesiredTimeStateInput(const scalar_array_t& commandLineTarget, scalar_t& desiredTime, vector_t& desiredState,
                                   vector_t& desiredInput) override {
    auto deg2rad = [](scalar_t deg) { return (deg * M_PI / 180.0); };

    desiredState.resize(commandDim_);
    if (command_mode_ == COMMAND_MODE::POSITION) {
      for (size_t j = 0; j < 3; j++) {
        // pose
        desiredState[9 + j] = deg2rad(commandLineTarget[5 - j]);
        desiredState[6 + j] = commandLineTarget[j];
        // velocities
        desiredState[3 + j] = commandLineTarget[9 + j];
        desiredState[j] = commandLineTarget[6 + j];
      }
      desiredTime = estimateTimeToTarget(desiredState[9], desiredState[6], desiredState[7]);
    } else if (command_mode_ == COMMAND_MODE::VELOCITY) {
      // velocity before position and orientation
      for (size_t j = 0; j < 3; j++) {
        // velocities
        desiredState[3 + j] = commandLineTarget[3 + j];
        desiredState[j] = commandLineTarget[j];
        // pose
        desiredState[9 + j] = deg2rad(commandLineTarget[11 - j]);
        desiredState[6 + j] = commandLineTarget[6 + j];
      }
      // it would take 3 seconds to accelerate to the commanded velocity
      desiredTime = 3;
    } else {
      std::runtime_error("Unknown command mode for target!");
    }

    // input
    // TODO(Ruben)
    desiredInput = vector_t::Zero(inputDim_);
    desiredInput[2 + 0] = 80.0;
    desiredInput[2 + 3] = 80.0;
    desiredInput[2 + 6] = 80.0;
    desiredInput[2 + 9] = 80.0;
  }

  CostDesiredTrajectories toCostDesiredTrajectories(const scalar_array_t& commandLineTarget) override {
    SystemObservation observation;
    ::ros::spinOnce();
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      ros_msg_conversions::readObservationMsg(*latestObservation_, observation);
    }

    // Convert commandline target to base desired
    scalar_t desiredTime;
    vector_t desiredBaseState;
    vector_t desiredInput;
    toCostDesiredTimeStateInput(commandLineTarget, desiredTime, desiredBaseState, desiredInput);

    // Desired time trajectory
    const scalar_array_t tDesiredTrajectory{observation.time, observation.time + desiredTime};

    // Desired state trajectory
    vector_array_t xDesiredTrajectory(2, vector_t::Zero(stateDim_));
    xDesiredTrajectory[0] << observation.state.head<12>(), defaultJointCoordinates_;

    // Roll and pitch are absolute
    xDesiredTrajectory[1].segment(10, 2) = desiredBaseState.segment(10, 2);
    // Yaw relative to current
    xDesiredTrajectory[1][9] = observation.state[9] + desiredBaseState[9];
    // base x, y relative to current state
    xDesiredTrajectory[1].segment(6, 2) = observation.state.segment(6, 2) + desiredBaseState.segment(6, 2);
    // base z relative to initialization
    xDesiredTrajectory[1][8] = initZHeight_ + desiredBaseState[8];
    // target velocities
    xDesiredTrajectory[1].segment(0, 6) = desiredBaseState.segment(0, 6);
    // joint angle from initialization
    xDesiredTrajectory[1].segment(12, actuatedDofNum_) = defaultJointCoordinates_;

    // Desired input trajectory
    vector_array_t uDesiredTrajectory(2, desiredInput);

    return {tDesiredTrajectory, xDesiredTrajectory, uDesiredTrajectory};
  }

  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = msg;
  }

 private:
  scalar_t estimateTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const {
    scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity_;
    scalar_t displacement = std::sqrt(dx * dx + dy * dy);
    scalar_t displacementTime = displacement / targetDisplacementVelocity_;

    return std::max(rotationTime, displacementTime);
  }

  COMMAND_MODE command_mode_;

  ros::Subscriber observationSubscriber_;

  std::mutex latestObservationMutex_;
  ocs2_msgs::mpc_observation::ConstPtr latestObservation_;
  scalar_t initZHeight_;
  vector_t defaultJointCoordinates_;

  scalar_t targetDisplacementVelocity_;
  scalar_t targetRotationVelocity_;
};

constexpr size_t TargetTrajectoriesKeyboardQuadruped::commandDim_;
constexpr size_t TargetTrajectoriesKeyboardQuadruped::stateDim_;
constexpr size_t TargetTrajectoriesKeyboardQuadruped::inputDim_;
constexpr size_t TargetTrajectoriesKeyboardQuadruped::actuatedDofNum_;

}  // namespace legged_robot
}  // namespace ocs2
