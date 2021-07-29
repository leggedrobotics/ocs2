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

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2 {
/**
 * Mapping between OCS2 and pinocchio state and input
 */
template <typename SCALAR>
class PinocchioStateInputMapping {
 public:
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  PinocchioStateInputMapping() = default;
  virtual ~PinocchioStateInputMapping() = default;
  PinocchioStateInputMapping<SCALAR>& operator=(const PinocchioStateInputMapping<SCALAR>& rhs) = delete;
  virtual PinocchioStateInputMapping<SCALAR>* clone() const = 0;

  /** Get the pinocchio joint configuration from OCS2 state and input vectors. */
  virtual vector_t getPinocchioJointPosition(const vector_t& state) const = 0;

  /** Get the pinocchio joint velocity from OCS2 state and input vectors. */
  virtual vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const = 0;

  /** Mapps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu. */
  virtual std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const = 0;

  /** If the mapping requires PinocchioInterface, use this method and set an updated PinocchioInterface. */
  virtual void setPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) {}

 protected:
  PinocchioStateInputMapping(const PinocchioStateInputMapping<SCALAR>& rhs) = default;
};

}  // namespace ocs2
