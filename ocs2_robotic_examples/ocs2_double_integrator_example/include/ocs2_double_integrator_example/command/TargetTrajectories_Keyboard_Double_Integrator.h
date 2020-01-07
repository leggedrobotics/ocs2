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

#include <ocs2_robotic_tools/command/TargetPoseTransformation.h>
#include <ocs2_robotic_tools/command/TargetTrajectories_Keyboard_Interface.h>

namespace ocs2 {
namespace double_integrator {

/**
 * This class implements TargetTrajectories communication using ROS.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T>
class TargetTrajectories_Keyboard_Double_Integrator final : public ocs2::TargetTrajectories_Keyboard_Interface<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { command_dim_ = 2 };

  using BASE = ocs2::TargetTrajectories_Keyboard_Interface<SCALAR_T>;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;
  using dynamic_vector_array_t = typename BASE::dynamic_vector_array_t;

  /**
   * Constructor.
   *
   * @param robotName: The robot's name.
   * @param goalPoseLimit: Limits for the input command. It has size 2 with following entries.
   *
   * goalPoseLimit(0): X
   * goalPoseLimit(1): V_X
   */
  TargetTrajectories_Keyboard_Double_Integrator(int argc, char* argv[], const std::string& robotName = "robot",
                                                const scalar_array_t& goalPoseLimit = scalar_array_t{10.0, 10.0})
      : BASE(argc, argv, robotName, command_dim_, goalPoseLimit) {}

  /**
   * Default destructor
   */
  ~TargetTrajectories_Keyboard_Double_Integrator() override = default;

  void toCostDesiredTimeStateInput(const scalar_array_t& commadLineTarget, scalar_t& desiredTime, dynamic_vector_t& desiredState,
                                   dynamic_vector_t& desiredInput) override {
    // time
    desiredTime = 0.0;
    // state
    desiredState = Eigen::Map<const dynamic_vector_t>(commadLineTarget.data(), command_dim_);
    // input
    desiredInput = dynamic_vector_t::Zero(1);
  }

 private:
};

}  // namespace double_integrator
}  // namespace ocs2
