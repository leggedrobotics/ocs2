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

#include <string>

#include <ocs2_core/Types.h>

#include "ocs2_ros_interfaces/command/TargetTrajectoriesRosInterface.h"

namespace ocs2 {

/**
 * This class lets the user to insert robot command form joystick.
 */
class TargetTrajectoriesJoystickInterface : public TargetTrajectoriesRosInterface {
 public:
  /**
   * Constructor
   *
   * @param [in] robotName: robot's name.
   * @param [in] targetCommandSize: command expected length
   * @param [in] targetCommandLimits: The limits of the loaded command from
   * command-line (for safety purposes).
   */
  TargetTrajectoriesJoystickInterface(int argc, char* argv[], const std::string& robotName = "robot", const size_t targetCommandSize = 0,
                                      const scalar_array_t& targetCommandLimits = scalar_array_t());

  /**
   * Default destructor
   */
  ~TargetTrajectoriesJoystickInterface() override = default;

  /**
   * Gets the command vector size.
   *
   * @return The command vector size.
   */
  size_t& targetCommandSize();

  /**
   * Sets cost trajectories from input desired state, and publishes cost desired trajectories
   */
  void publishTargetTrajectoriesFromDesiredState(CostDesiredTrajectories costDesiredTrajectories);

 protected:
  /******
   * Variables
   ******/
  size_t targetCommandSize_;
  scalar_array_t targetCommandLimits_;

  scalar_array_t targetCommand_;
};

}  // namespace ocs2
