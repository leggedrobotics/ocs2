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

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"
#include "ocs2_centroidal_model/CentroidalModelRbdConversions.h"

namespace ocs2 {

/**
 * Centroidal inverse dynamics with PD control using Pinocchio:
 * torque = torque_inverse_dynamics + pGains * (qDesired - qMeasured) + dGains * (vDesired - vMeasured)
 */
class PinocchioCentroidalInverseDynamicsPD final {
 public:
  /**
   * Constructor
   * @param [in] pinocchioInterface : The predefined pinocchio interface for the robot.
   * @param [in] centroidalModelInfo : The centroidal model information.
   */
  PinocchioCentroidalInverseDynamicsPD(PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& centroidalModelInfo);

  /**
   * Default destructor.
   */
  ~PinocchioCentroidalInverseDynamicsPD() = default;

  /**
   * Set the PD gains.
   * @param [in] pGains : The proportional gains.
   * @param [in] dGains : The derivative gains.
   */
  void setGains(const vector_t& pGains, const vector_t& dGains) {
    pGains_ = pGains;
    dGains_ = dGains;
  }

  /**
   * Get the torque.
   * @param [in] desiredState : The desired ocs2 state.
   * @param [in] desiredInput : The desired ocs2 input.
   * @param [in] desiredJointAccelerations : The desired joint accelerations.
   * @param [in] measuredState : The measured ocs2 state (required for PD control).
   * @param [in] measuredInput : The measured ocs2 input (required for PD control).
   */
  vector_t getTorque(const vector_t& desiredState, const vector_t& desiredInput, const vector_t& desiredJointAccelerations,
                     const vector_t& measuredState, const vector_t& measuredInput);

 private:
  PinocchioInterface* pinocchioInterfacePtr_;
  CentroidalModelPinocchioMapping centroidalModelPinocchioMapping_;
  CentroidalModelRbdConversions centroidalModelRbdConversions_;
  vector_t pGains_;
  vector_t dGains_;
};

}  // namespace ocs2
