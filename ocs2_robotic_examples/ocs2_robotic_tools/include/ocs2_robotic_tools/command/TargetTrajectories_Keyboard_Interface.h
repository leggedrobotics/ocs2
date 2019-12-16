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

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/command/TargetTrajectories_ROS_Interface.h>

namespace ocs2 {

/**
 * This class lets the user to insert robot command form command line.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T>
class TargetTrajectories_Keyboard_Interface : public ocs2::TargetTrajectories_ROS_Interface<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::TargetTrajectories_ROS_Interface<SCALAR_T>;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;
  using dynamic_vector_array_t = typename BASE::dynamic_vector_array_t;

  /**
   * Constructor
   *
   * @param [in] argc: Number of command line arguments
   * @param [in] argv: Command line arguments
   * @param [in] targetCommandSize: command expected length
   * @param [in] robotName: robot's name.
   * @param [in] targetCommandSize: command expected length
   * @param [in] targetCommandLimits: The limits of the loaded command from
   * command-line (for safety purposes).
   */
  TargetTrajectories_Keyboard_Interface(int argc, char* argv[], const std::string& robotName = "robot", const size_t targetCommandSize = 0,
                                        const scalar_array_t& targetCommandLimits = scalar_array_t());

  /**
   * Default destructor
   */
  virtual ~TargetTrajectories_Keyboard_Interface() = default;

  /**
   * Gets the command vector size.
   *
   * @return The command vector size.
   */
  size_t& targetCommandSize();

  /**
   * From command line loaded command to desired time, state, and input.
   * This method should be overridden by the user if modification is
   * required on the loaded command from command-line.
   *
   * @param [out] commadLineTarget: The loaded command target.
   * @param [in] desiredTime: Desired time to be published.
   * @param [in] desiredState: Desired state to be published.
   * @param [in] desiredInput: Desired input to be published.
   */
  virtual void toCostDesiredTimeStateInput(const scalar_array_t& commadLineTarget, scalar_t& desiredTime, dynamic_vector_t& desiredState,
                                           dynamic_vector_t& desiredInput);

  /**
   * @brief Turns command line values into a costDesiredTrajectories object
   * @note Default implementation calls toCostDesiredTimeStateInput
   * @param[in] commadLineTarget
   * @return costDesiredTrajectories
   */
  virtual CostDesiredTrajectories toCostDesiredTrajectories(const scalar_array_t& commadLineTarget);

  /**
   * Gets command line input. If the input command is shorter than the expected command
   * size (targetCommandSize_), the method will set the rest of the command to zero.
   *
   * @param [in] commadMsg: Message to be displayed on screen.
   */
  void getKeyboardCommand(const std::string& commadMsg = "Enter command, separated by spaces");

 protected:
  /**
   * Gets the target from command line.
   *
   * @return targetCommand: The target command.
   */
  scalar_array_t getCommandLine();

  /******
   * Variables
   ******/
  size_t targetCommandSize_;
  scalar_array_t targetCommandLimits_;

  scalar_array_t targetCommand_;
};

}  // namespace ocs2

#include "implementation/TargetTrajectories_Keyboard_Interface.h"
