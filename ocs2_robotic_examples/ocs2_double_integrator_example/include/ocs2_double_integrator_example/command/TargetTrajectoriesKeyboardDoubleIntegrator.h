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

#include <ocs2_core/Types.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardInterface.h>

#include "ocs2_double_integrator_example/definitions.h"

namespace ocs2 {
namespace double_integrator {

/**
 * This class implements TargetTrajectories communication using ROS.
 */
class TargetTrajectoriesKeyboardDoubleIntegrator final : public ocs2::TargetTrajectoriesKeyboardInterface {
 public:
  enum { COMMAND_DIM = 2 };

  /**
   * Constructor.
   *
   * @param robotName: The robot's name.
   * @param goalPoseLimit: Limits for the input command. It has size 2 with following entries.
   *
   * goalPoseLimit(0): X
   * goalPoseLimit(1): V_X
   */
  TargetTrajectoriesKeyboardDoubleIntegrator(int argc, char* argv[], const std::string& robotName = "robot",
                                             const scalar_array_t& goalPoseLimit = scalar_array_t{10.0, 10.0})
      : ocs2::TargetTrajectoriesKeyboardInterface(argc, argv, robotName, COMMAND_DIM, goalPoseLimit) {}

  /**
   * Default destructor
   */
  ~TargetTrajectoriesKeyboardDoubleIntegrator() override = default;

  void toCostDesiredTimeStateInput(const scalar_array_t& commadLineTarget, scalar_t& desiredTime, vector_t& desiredState,
                                   vector_t& desiredInput) override {
    // time
    desiredTime = 0.0;
    // state
    desiredState = Eigen::Map<const vector_t>(commadLineTarget.data(), COMMAND_DIM);
    // input
    desiredInput = vector_t::Zero(INPUT_DIM);
  }

 private:
};

}  // namespace double_integrator
}  // namespace ocs2
