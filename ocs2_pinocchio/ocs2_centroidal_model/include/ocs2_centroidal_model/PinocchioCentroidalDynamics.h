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

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2 {

class PinocchioCentroidalDynamics {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using CentroidalModelType = CentroidalModelPinocchioMapping<scalar_t>::CentroidalModelType;

  using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
  using Matrix3x = Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>;
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

  PinocchioCentroidalDynamics(const PinocchioInterface& pinocchioInterface, const CentroidalModelPinocchioMapping<scalar_t>& mapping);

  ~PinocchioCentroidalDynamics() = default;

  vector_t getSystemFlowMap(scalar_t time, const vector_t& state, const vector_t& input);

  VectorFunctionLinearApproximation getSystemFlowMapLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input);

 private:
  void computeNormalizedCentroidalMomentumRateGradients(const vector_t& state, const vector_t& input);

  const PinocchioInterface& pinocchioInterface_;
  const CentroidalModelPinocchioMapping<scalar_t>& mapping_;

  // partial derivatives of the system dynamics
  Matrix3x normalizedLinearMomentumRateDerivativeState_;
  Matrix3x normalizedLinearMomentumRateDerivativeInput_;
  Matrix3x normalizedAngularMomentumRateDerivativeState_;
  Matrix3x normalizedAngularMomentumRateDerivativeInput_;
  Matrix6x floatingBaseVelocitiesDerivativeState_;
  Matrix6x floatingBaseVelocitiesDerivativeInput_;
  matrix_t jointVelocitiesDerivativeState_;
  matrix_t jointVelocitiesDerivativeInput_;

  // centroidal momentum derivatives
  Matrix6x dh_dq_;
  Matrix6x dhdot_dq_;
  Matrix6x dhdot_dv_;
  Matrix6x dhdot_da_;
};
}  // namespace ocs2