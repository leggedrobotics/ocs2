/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include <ocs2_mobile_manipulator/MobileManipulatorModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * @brief Implementation of a wheel-based mobile manipulator.
 *
 * The wheel-based manipulator is simulated 2D-bicycle model for the base. The state
 * of the robot is: (base x, base y, base yaw, arm joints).
 *
 * The robot is assumed to be velocity controlled with the base commands as the forward
 * velocity and the angular velocity around z.
 */
class WheelBasedManipulatorDynamics final : public SystemDynamicsBaseAD {
 public:
  using Base = SystemDynamicsBaseAD;

  explicit WheelBasedManipulatorDynamics(const std::string& modelName, const MobileManipulatorModelInfo& modelInfo,
                                         const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);
  ~WheelBasedManipulatorDynamics() override = default;
  WheelBasedManipulatorDynamics* clone() const override { return new WheelBasedManipulatorDynamics(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override;

 private:
  WheelBasedManipulatorDynamics(const WheelBasedManipulatorDynamics& rhs) = default;

  MobileManipulatorModelInfo info_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
