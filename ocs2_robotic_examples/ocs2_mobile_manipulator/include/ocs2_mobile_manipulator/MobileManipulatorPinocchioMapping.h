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

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2 {
namespace mobile_manipulator {

template <typename SCALAR>
class MobileManipulatorPinocchioMappingTpl;

using MobileManipulatorPinocchioMapping = MobileManipulatorPinocchioMappingTpl<scalar_t>;
using MobileManipulatorPinocchioMappingCppAd = MobileManipulatorPinocchioMappingTpl<ad_scalar_t>;

/**
 * Pinocchio state and input mapping.
 */
template <typename SCALAR>
class MobileManipulatorPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR> {
 public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;

  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit MobileManipulatorPinocchioMappingTpl(ManipulatorModelInfo info);

  ~MobileManipulatorPinocchioMappingTpl() override = default;
  MobileManipulatorPinocchioMappingTpl<SCALAR>* clone() const override;

  /**
   * Computes the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state.
   *
   * @param [in] state: system state vector
   * @return pinocchio joint positions, which are also the robot's generalized positions with a ZYX-Euler angle
   * parameterization of the base orientation
   */
  vector_t getPinocchioJointPosition(const vector_t& state) const override;

  /**
   * Computes the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input.
   *
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return pinocchio joint velocities, which are also the time derivatives of the pinocchio joint positions
   */
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  /**
   * Maps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
   *
   * @param [in] state: system state vector
   * @param [in] Jq: jacobian with respect to pinocchio joint positions
   * @param [in] Jv: jacobian with respect to pinocchio joint velocities
   * @return a pair {dfdx, dfdu} containing the jacobians with respect to the system state and input
   */
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  /**
   * Returns the mobile manipulator model info.
   */
  const ManipulatorModelInfo& getManipulatorModelInfo() const { return modelInfo_; }

 private:
  MobileManipulatorPinocchioMappingTpl(const MobileManipulatorPinocchioMappingTpl& rhs) = default;

  const ManipulatorModelInfo modelInfo_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
