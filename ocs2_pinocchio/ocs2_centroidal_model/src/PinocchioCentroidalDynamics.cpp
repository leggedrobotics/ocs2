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

#include "ocs2_centroidal_model/PinocchioCentroidalDynamics.h"
#include "ocs2_centroidal_model/helpers.h"

#include <pinocchio/container/aligned-vector.hpp>
#include "pinocchio/algorithm/centroidal-derivatives.hpp"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalDynamics::PinocchioCentroidalDynamics(const PinocchioInterface& pinocchioInterface,
                                                         CentroidalModelPinocchioMapping<scalar_t>& mapping)
    : pinocchioInterfacePtr_(&pinocchioInterface), mappingPtr_(&mapping) {
  mappingPtr_->setPinocchioInterface(pinocchioInterface);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioCentroidalDynamics::getSystemFlowMap(scalar_t time, const vector_t& state, const vector_t& input) {
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const auto& info = mappingPtr_->getCentroidalModelInfo();
  assert(info.stateDim == state.rows());
  return (vector_t(state.rows()) << mappingPtr_->normalizedCentroidalMomentumRate(input),
          mappingPtr_->getPinocchioJointVelocity(state, input))
      .finished();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PinocchioCentroidalDynamics::getSystemFlowMapLinearApproximation(scalar_t time, const vector_t& state,
                                                                                                   const vector_t& input) {
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const auto& info = mappingPtr_->getCentroidalModelInfo();
  assert(info.stateDim == stateDim);
  assert(info.inputDim == inputDim);

  auto dynamics = ocs2::VectorFunctionLinearApproximation::Zero(state.rows(), state.rows(), inputDim);
  dynamics.f = getSystemFlowMap(time, state, input);

  // Partial derivatives of the normalized momentum rates
  computeNormalizedCentroidalMomentumRateGradients(state, input);

  // Partial derivatives of the floating base variables
  const auto& A = mappingPtr_->getCentroidalMomentumMatrix();
  const Matrix6 Ab = A.template leftCols<6>();
  // TODO: move getFloatingBaseCentroidalMomentumMatrixInverse(Ab) to PreComputation
  const auto& Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info.actuatedDofNum);

  floatingBaseVelocitiesDerivativeInput_.setZero(6, inputDim);
  floatingBaseVelocitiesDerivativeState_.setZero(6, stateDim);
  floatingBaseVelocitiesDerivativeState_.leftCols(6) = info.robotMass * Ab_inv;
  dh_dq_.resize(6, info.generalizedCoordinatesNum);
  dhdot_dq_.resize(6, info.generalizedCoordinatesNum);
  dhdot_dv_.resize(6, info.generalizedCoordinatesNum);
  dhdot_da_.resize(6, info.generalizedCoordinatesNum);

  // TODO: Check how to get the correct value for dh_dq_
  const pinocchio::Inertia& Ytot = data.oYcrb[0];
  const typename pinocchio::Inertia::Vector3& com = Ytot.lever();
  pinocchio::translateForceSet(data.dHdq, com, PINOCCHIO_EIGEN_CONST_CAST(Matrix6x, dh_dq_));
  dh_dq_.leftCols(3).setZero();

  if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
    floatingBaseVelocitiesDerivativeState_.rightCols(info.generalizedCoordinatesNum).noalias() = -Ab_inv * dh_dq_;
    floatingBaseVelocitiesDerivativeInput_.rightCols(info.actuatedDofNum).noalias() = -Ab_inv * Aj;
  } else if (info.centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    floatingBaseVelocitiesDerivativeState_.middleCols(6, 6).noalias() = -Ab_inv * dh_dq_.leftCols(6);
  }

  // Partial derivatives of the actuated joints
  jointVelocitiesDerivativeInput_.setZero(info.actuatedDofNum, inputDim);
  jointVelocitiesDerivativeInput_.rightCols(info.actuatedDofNum).setIdentity();
  jointVelocitiesDerivativeState_.setZero(info.actuatedDofNum, state.rows());

  dynamics.dfdx << normalizedLinearMomentumRateDerivativeState_, normalizedAngularMomentumRateDerivativeState_,
      floatingBaseVelocitiesDerivativeState_, jointVelocitiesDerivativeState_;
  dynamics.dfdu << normalizedLinearMomentumRateDerivativeInput_, normalizedAngularMomentumRateDerivativeInput_,
      floatingBaseVelocitiesDerivativeInput_, jointVelocitiesDerivativeInput_;

  return dynamics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioCentroidalDynamics::computeNormalizedCentroidalMomentumRateGradients(const vector_t& state, const vector_t& input) {
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const auto& info = mappingPtr_->getCentroidalModelInfo();
  assert(info.stateDim == stateDim);
  assert(info.inputDim == inputDim);

  // compute partial derivatives of the center of robotMass acceleration and normalized angular momentum
  normalizedLinearMomentumRateDerivativeState_.setZero(3, stateDim);
  normalizedLinearMomentumRateDerivativeInput_.setZero(3, inputDim);
  normalizedAngularMomentumRateDerivativeState_.setZero(3, stateDim);
  normalizedAngularMomentumRateDerivativeInput_.setZero(3, inputDim);
  Matrix3 f_hat, p_hat;

  auto getForce = [&](size_t index) { return getContactForce(input, index, info); };

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const Vector3 contactForceInWorldFrame = getForce(i);
    f_hat = skewSymmetricMatrix(contactForceInWorldFrame) / info.robotMass;
    normalizedAngularMomentumRateDerivativeState_.rightCols(info.generalizedCoordinatesNum) -=
        f_hat * (mappingPtr_->getTranslationalJacobianComToContactPointInWorldFrame(i));
    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i).diagonal().array() = 1.0 / info.robotMass;
    p_hat = skewSymmetricMatrix(mappingPtr_->getPositionComToContactPointInWorldFrame(i)) / info.robotMass;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i) = p_hat;
  }

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const size_t inputIdx = 3 * info.numThreeDofContacts + 6 * (i - info.numThreeDofContacts);
    const Vector3 contactForceInWorldFrame = getForce(i);
    f_hat = skewSymmetricMatrix(contactForceInWorldFrame) / info.robotMass;
    normalizedAngularMomentumRateDerivativeState_.rightCols(info.generalizedCoordinatesNum) -=
        f_hat * (mappingPtr_->getTranslationalJacobianComToContactPointInWorldFrame(i));

    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, inputIdx).diagonal().array() = 1.0 / info.robotMass;

    p_hat = skewSymmetricMatrix(mappingPtr_->getPositionComToContactPointInWorldFrame(i)) / info.robotMass;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx) = p_hat;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx + 3).diagonal().array() = 1.0 / info.robotMass;
  }
}

}  // namespace ocs2
