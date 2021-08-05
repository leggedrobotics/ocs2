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

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

namespace ocs2 {

/**
 * Centroidal Dynamics:
 *
 * State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
 * @remark: The linear and angular momenta are expressed with respect to the centroidal frame (a frame centered at
 * the CoM and aligned with the inertial frame).
 *
 * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
 * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
 */
class PinocchioCentroidalDynamicsAD final {
 public:
  /**
   * Constructor
   * @param [in] pinocchioInterface : The pinocchio interface.
   * @param [in] CentroidalModelInfo : The centroidal model information.
   * @param [in] modelName : Name of the generate model library
   * @param [in] modelFolder : Folder to save the model library files to
   * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose : print information.
   */
  PinocchioCentroidalDynamicsAD(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info, const std::string& modelName,
                                const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = false);

  /** Copy Constructor */
  PinocchioCentroidalDynamicsAD(const PinocchioCentroidalDynamicsAD& rhs);

  /**
   * Computes system flow map x_dot = f(x, u)
   *
   * @param time: time
   * @param state: system state vector
   * @param input: system input vector
   * @return system flow map x_dot = f(x, u)
   */
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input) const;

  /**
   * Computes first order approximation of the system flow map x_dot = f(x, u)
   *
   * @param time: time
   * @param state: system state vector
   * @param input: system input vector
   * @return linear approximation of system flow map x_dot = f(x, u)
   */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input) const;

 private:
  ad_vector_t getValueCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const CentroidalModelPinocchioMappingCppAd& mapping,
                            const ad_vector_t& state, const ad_vector_t& input);

  std::unique_ptr<CppAdInterface> systemFlowMapCppAdInterfacePtr_;
};

}  // namespace ocs2
