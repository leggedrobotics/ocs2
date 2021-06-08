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

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
CentroidalModelPinocchioMapping<SCALAR>::CentroidalModelPinocchioMapping(
    size_t stateDim, size_t inputDim, const CentroidalModelPinocchioInterface<SCALAR>& centroidalModelPinocchioInterface)
    : stateDim_(stateDim), inputDim_(inputDim), centroidalModelPinocchioInterface_(centroidalModelPinocchioInterface){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  assert(stateDim_ == state.rows());
  centroidalModelPinocchioInterface_.updatePinocchioJointPositions(state);
  return centroidalModelPinocchioInterface_.getCentroidalModelInfo().qPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
  assert(stateDim_ == state.rows());
  centroidalModelPinocchioInterface_.updatePinocchioJointPositions(state);
  centroidalModelPinocchioInterface_.updatePinocchioJointVelocities(state, input);
  return centroidalModelPinocchioInterface_.getCentroidalModelInfo().vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  assert(stateDim_ == state.rows());
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;

  matrix_t dfdx = matrix_t::Zero(Jq.rows(), stateDim_);
  dfdx.middleCols(6, GENERALIZED_VEL_NUM) = Jq;

  matrix_t dfdu = matrix_t::Zero(Jv.rows(), inputDim_);
  dfdu.rightCols(ACTUATED_DOF_NUM) = Jv.rightCols(ACTUATED_DOF_NUM);

  return {dfdx, dfdu};
}

// explicit template instantiation
template class ocs2::CentroidalModelPinocchioMapping<ocs2::scalar_t>;
template class ocs2::CentroidalModelPinocchioMapping<ocs2::ad_scalar_t>;

}  // namespace ocs2
