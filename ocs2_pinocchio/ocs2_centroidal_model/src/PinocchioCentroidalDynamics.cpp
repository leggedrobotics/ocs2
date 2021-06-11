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

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include "pinocchio/algorithm/centroidal-derivatives.hpp"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalDynamics::PinocchioCentroidalDynamics(const PinocchioInterface& pinocchioInterface,
                                                         const CentroidalModelPinocchioMapping<scalar_t>& mapping)
    : pinocchioInterface_(pinocchioInterface), mapping_(mapping) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioCentroidalDynamics::getSystemFlowMap(scalar_t time, const vector_t& state, const vector_t& input) {
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  assert(GENERALIZED_VEL_NUM == state.rows() - 6);
  const auto& info = mapping_.getCentroidalModelInfo();

  vector_t stateDerivative(state.rows());

  // It is assumed pinocchio::computeCentroidalMap and pinocchio::updateFramePlacements have been called externally

  // compute center of robotMass acceleration and derivative of the normalized angular momentum
  stateDerivative.head(6) = mapping_.normalizedCentroidalMomentumRate(input);

  // derivatives of the floating base variables + joint velocities
  stateDerivative.tail(GENERALIZED_VEL_NUM) = mapping_.getPinocchioJointVelocity(state, input);

  return stateDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PinocchioCentroidalDynamics::getSystemFlowMapLinearApproximation(scalar_t time, const vector_t& state,
                                                                                                   const vector_t& input) {
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  const pinocchio::Data& data = pinocchioInterface_.getData();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;
  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  assert(GENERALIZED_VEL_NUM == stateDim - 6);
  const auto& info = mapping_.getCentroidalModelInfo();

  // It is assumed pinocchio::computeCentroidalMap and pinocchio::updateFramePlacements have been called externally

  auto dynamics = ocs2::VectorFunctionLinearApproximation::Zero(state.rows(), state.rows(), inputDim);
  dynamics.f = getSystemFlowMap(time, state, input);

  const auto& qPinocchio = mapping_.getPinocchioJointPosition(state);
  const auto& vPinocchio = dynamics.f.tail(GENERALIZED_VEL_NUM);

  // Partial derivatives of the normalized momentum rates
  computeNormalizedCentroidalMomentumRateGradients(state, input);

  // Partial derivatives of the floating base variables
  const Matrix6x A = mapping_.getCentroidalMomentumMatrix();
  const Matrix6 Ab = A.template leftCols<6>();
  const Matrix6x Aj = A.rightCols(ACTUATED_DOF_NUM);
  const Matrix6 Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  floatingBaseVelocitiesDerivativeState_.setZero(6, stateDim);
  floatingBaseVelocitiesDerivativeState_.leftCols(6) = info.robotMass * Ab_inv;
  dh_dq_.resize(6, GENERALIZED_VEL_NUM);
  dhdot_dq_.resize(6, GENERALIZED_VEL_NUM);
  dhdot_dv_.resize(6, GENERALIZED_VEL_NUM);
  dhdot_da_.resize(6, GENERALIZED_VEL_NUM);

  // TODO: Check how to get the correct value for dh_dq_
  if (info.centroidalModelType == CentroidalModelType::FullCentroidalDynamics) {
    //    pinocchio::computeCentroidalDynamicsDerivatives(model, data,
    //                                                    qPinocchio, vPinocchio,
    //                                                    vector_t::Zero(GENERALIZED_VEL_NUM),
    //                                                    dh_dq_, dhdot_dq_, dhdot_dv_, dhdot_da_);

    // Assumes pinocchio::computeCentroidalDynamicsDerivatives has been called externally
    const pinocchio::Inertia& Ytot = data.oYcrb[0];
    const typename pinocchio::Inertia::Vector3& com = Ytot.lever();
    pinocchio::translateForceSet(data.dHdq, com, PINOCCHIO_EIGEN_CONST_CAST(Matrix6x, dh_dq_));
    dh_dq_.leftCols(3).setZero();
    floatingBaseVelocitiesDerivativeState_.rightCols(GENERALIZED_VEL_NUM) = -Ab_inv * dh_dq_;
  } else if (info.centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    //    vector_t qPinocchioNominal = info.qPinocchioNominal;
    //    qPinocchioNominal.head(6) = qPinocchio.head(6);
    //    vector_t vPinocchioNominal = vector_t::Zero(GENERALIZED_VEL_NUM);
    //    vPinocchioNominal.head(6) = vPinocchio.head(6);
    //    pinocchio::computeCentroidalDynamicsDerivatives(model, data, qPinocchioNominal, vPinocchioNominal,
    //                                                    vector_t::Zero(GENERALIZED_VEL_NUM), dh_dq_, dhdot_dq_, dhdot_dv_,
    //                                                    dhdot_da_);

    // Assumes pinocchio::computeCentroidalDynamicsDerivatives has been called externally
    const pinocchio::Inertia& Ytot = data.oYcrb[0];
    const typename pinocchio::Inertia::Vector3& com = Ytot.lever();
    pinocchio::translateForceSet(data.dHdq, com, PINOCCHIO_EIGEN_CONST_CAST(Matrix6x, dh_dq_));
    dh_dq_.leftCols(3).setZero();
    floatingBaseVelocitiesDerivativeState_.rightCols(GENERALIZED_VEL_NUM).leftCols(6) = -Ab_inv * dh_dq_.leftCols(6);
  }

  floatingBaseVelocitiesDerivativeInput_.setZero(6, inputDim);
  floatingBaseVelocitiesDerivativeInput_.rightCols(ACTUATED_DOF_NUM) = -Ab_inv * Aj;

  // Partial derivatives of the actuated joints
  jointVelocitiesDerivativeInput_.setZero(ACTUATED_DOF_NUM, inputDim);
  jointVelocitiesDerivativeInput_.rightCols(ACTUATED_DOF_NUM).setIdentity();
  jointVelocitiesDerivativeState_.setZero(ACTUATED_DOF_NUM, state.rows());

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
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;
  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const auto& info = mapping_.getCentroidalModelInfo();
  assert(3 * info.numThreeDofContacts + 6 * info.numSixDofContacts + ACTUATED_DOF_NUM == inputDim);

  // compute partial derivatives of the center of robotMass acceleration and normalized angular momentum
  normalizedLinearMomentumRateDerivativeState_.setZero(3, stateDim);
  normalizedLinearMomentumRateDerivativeInput_.setZero(3, inputDim);
  normalizedAngularMomentumRateDerivativeState_.setZero(3, stateDim);
  normalizedAngularMomentumRateDerivativeInput_.setZero(3, inputDim);
  Vector3 contactForceInWorldFrame;
  Matrix3 f_hat, p_hat;

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    contactForceInWorldFrame = input.segment<3>(3 * i);
    f_hat = skewSymmetricMatrix(contactForceInWorldFrame);
    normalizedAngularMomentumRateDerivativeState_.rightCols(GENERALIZED_VEL_NUM) -=
        1.0 / info.robotMass * f_hat * (mapping_.getTranslationalJacobianComToContactPointInWorldFrame(i));
    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i) = 1.0 / info.robotMass * Matrix3::Identity();
    p_hat = skewSymmetricMatrix(mapping_.getPositionComToContactPointInWorldFrame(i));
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i) = 1.0 / info.robotMass * p_hat;
  }

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const size_t inputIdx = 3 * info.numThreeDofContacts + 6 * (i - info.numThreeDofContacts);
    contactForceInWorldFrame = input.segment<3>(inputIdx);
    f_hat = skewSymmetricMatrix(contactForceInWorldFrame);
    normalizedAngularMomentumRateDerivativeState_.rightCols(GENERALIZED_VEL_NUM) -=
        1.0 / info.robotMass * f_hat * (mapping_.getTranslationalJacobianComToContactPointInWorldFrame(i));

    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, inputIdx) = 1.0 / info.robotMass * Matrix3::Identity();

    p_hat = skewSymmetricMatrix(mapping_.getPositionComToContactPointInWorldFrame(i));
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx) = 1.0 / info.robotMass * p_hat;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx + 3) = 1.0 / info.robotMass * Matrix3::Identity();
  }
}

}  // namespace ocs2
