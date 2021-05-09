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

#include <ocs2_centroidal_model/KinoCentroidalDynamics.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include "pinocchio/algorithm/centroidal-derivatives.hpp"

namespace ocs2 {

KinoCentroidalDynamics::KinoCentroidalDynamics(const CentroidalModelPinocchioInterface<scalar_t>& centroidalModelPinocchioInterface)
    : centroidalModelPinocchioInterface_(centroidalModelPinocchioInterface){};

vector_t KinoCentroidalDynamics::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input) {
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  assert(GENERALIZED_VEL_NUM == state.rows() - 6);
  auto& centroidalModelInfo = centroidalModelPinocchioInterface_.getCentroidalModelInfo();

  vector_t stateDerivative(state.rows());

  centroidalModelPinocchioInterface_.updatePinocchioJointPositions(state);
  centroidalModelPinocchioInterface_.updatePinocchioJointVelocities(state, input);
  if (centroidalModelInfo.centroidalModelType ==
      CentroidalModelPinocchioInterface<scalar_t>::CentroidalModelType::SingleRigidBodyDynamics) {
    pinocchio::forwardKinematics(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData(),
                                 centroidalModelInfo.qPinocchio);
  }
  pinocchio::updateFramePlacements(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData());

  // compute center of mass acceleration and derivative of the normalized angular momentum
  stateDerivative.head(6) = centroidalModelPinocchioInterface_.normalizedCentroidalMomentumRate(input);

  // derivatives of the floating base variables + joint velocities
  stateDerivative.tail(GENERALIZED_VEL_NUM) = centroidalModelInfo.vPinocchio;

  return stateDerivative;
}

VectorFunctionLinearApproximation KinoCentroidalDynamics::linearApproximation(scalar_t time, const vector_t& state, const vector_t& input) {
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;
  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  assert(GENERALIZED_VEL_NUM == stateDim - 6);

  const auto& centroidalModelInfo = centroidalModelPinocchioInterface_.getCentroidalModelInfo();

  centroidalModelPinocchioInterface_.updatePinocchioJointPositions(state);
  centroidalModelPinocchioInterface_.updatePinocchioJointVelocities(state, input);
  if (centroidalModelInfo.centroidalModelType ==
      CentroidalModelPinocchioInterface<scalar_t>::CentroidalModelType::SingleRigidBodyDynamics) {
    pinocchio::computeJointJacobians(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData(),
                                     centroidalModelInfo.qPinocchio);
    pinocchio::forwardKinematics(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData(),
                                 centroidalModelInfo.qPinocchio);
  }
  pinocchio::updateFramePlacements(centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData());

  // Partial derivatives of the normalized momentum rates
  computeNormalizedCentroidalMomentumRateGradients(state, input);

  // Partial derivatives of the floating base variables
  floatingBaseVelocitiesDerivativeState_.setZero(6, stateDim);
  floatingBaseVelocitiesDerivativeState_.leftCols(6) = centroidalModelInfo.mass * centroidalModelInfo.Ab_inv;
  dh_dq_.resize(6, GENERALIZED_VEL_NUM);
  dhdot_dq_.resize(6, GENERALIZED_VEL_NUM);
  dhdot_dv_.resize(6, GENERALIZED_VEL_NUM);
  dhdot_da_.resize(6, GENERALIZED_VEL_NUM);

  // TODO: Check how to get the correct value for dh_dq_
  if (centroidalModelInfo.centroidalModelType == CentroidalModelPinocchioInterface<scalar_t>::CentroidalModelType::FullCentroidalDynamics) {
    pinocchio::computeCentroidalDynamicsDerivatives(centroidalModelPinocchioInterface_.getRobotModel(),
                                                    centroidalModelPinocchioInterface_.getRobotData(), centroidalModelInfo.qPinocchio,
                                                    centroidalModelInfo.vPinocchio, vector_t::Zero(GENERALIZED_VEL_NUM), dh_dq_, dhdot_dq_,
                                                    dhdot_dv_, dhdot_da_);
    dh_dq_.leftCols(3).setZero();
    floatingBaseVelocitiesDerivativeState_.rightCols(GENERALIZED_VEL_NUM) = -centroidalModelInfo.Ab_inv * dh_dq_;
  } else if (centroidalModelInfo.centroidalModelType ==
             CentroidalModelPinocchioInterface<scalar_t>::CentroidalModelType::SingleRigidBodyDynamics) {
    vector_t qPinocchio_nominal = centroidalModelInfo.qPinocchio_nominal;
    qPinocchio_nominal.head(6) = centroidalModelInfo.qPinocchio.head(6);
    vector_t vPinocchio_nominal = vector_t::Zero(GENERALIZED_VEL_NUM);
    vPinocchio_nominal.head(6) = centroidalModelInfo.vPinocchio.head(6);
    pinocchio::computeCentroidalDynamicsDerivatives(
        centroidalModelPinocchioInterface_.getRobotModel(), centroidalModelPinocchioInterface_.getRobotData(), qPinocchio_nominal,
        vPinocchio_nominal, vector_t::Zero(GENERALIZED_VEL_NUM), dh_dq_, dhdot_dq_, dhdot_dv_, dhdot_da_);
    dh_dq_.leftCols(3).setZero();
    floatingBaseVelocitiesDerivativeState_.rightCols(GENERALIZED_VEL_NUM).leftCols(6) = -centroidalModelInfo.Ab_inv * dh_dq_.leftCols(6);
  }
  floatingBaseVelocitiesDerivativeInput_.setZero(6, inputDim);
  floatingBaseVelocitiesDerivativeInput_.rightCols(ACTUATED_DOF_NUM) = -centroidalModelInfo.Ab_inv * centroidalModelInfo.Aj;

  // Partial derivatives of the actuated joints
  jointVelocitiesDerivativeInput_.setZero(ACTUATED_DOF_NUM, inputDim);
  jointVelocitiesDerivativeInput_.rightCols(ACTUATED_DOF_NUM).setIdentity();
  jointVelocitiesDerivativeState_.setZero(ACTUATED_DOF_NUM, state.rows());

  // TODO(mspieler): optimize redundant computations
  auto dynamics = ocs2::VectorFunctionLinearApproximation::Zero(state.rows(), state.rows(), inputDim);
  dynamics.f = computeFlowMap(time, state, input);
  dynamics.dfdx << normalizedLinearMomentumRateDerivativeState_, normalizedAngularMomentumRateDerivativeState_,
      floatingBaseVelocitiesDerivativeState_, jointVelocitiesDerivativeState_;
  dynamics.dfdu << normalizedLinearMomentumRateDerivativeInput_, normalizedAngularMomentumRateDerivativeInput_,
      floatingBaseVelocitiesDerivativeInput_, jointVelocitiesDerivativeInput_;
  return dynamics;
}

void KinoCentroidalDynamics::computeNormalizedCentroidalMomentumRateGradients(const vector_t& state, const vector_t& input) {
  const size_t GENERALIZED_VEL_NUM = centroidalModelPinocchioInterface_.getRobotModel().nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;
  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const auto& centroidalModelInfo = centroidalModelPinocchioInterface_.getCentroidalModelInfo();
  assert(3 * centroidalModelInfo.numThreeDofContacts + 6 * centroidalModelInfo.numSixDofContacts + ACTUATED_DOF_NUM == inputDim);

  // compute partial derivatives of the center of mass acceleration and normalized angular momentum
  normalizedLinearMomentumRateDerivativeState_.setZero(3, stateDim);
  normalizedLinearMomentumRateDerivativeInput_.setZero(3, inputDim);
  normalizedAngularMomentumRateDerivativeState_.setZero(3, stateDim);
  normalizedAngularMomentumRateDerivativeInput_.setZero(3, inputDim);
  Vector3 contactForceInWorldFrame;
  Matrix3 f_hat, p_hat;

  for (size_t i = 0; i < centroidalModelInfo.numThreeDofContacts; i++) {
    contactForceInWorldFrame = input.segment<3>(3 * i);
    getSkewMatrix<scalar_t>(contactForceInWorldFrame, f_hat);
    normalizedAngularMomentumRateDerivativeState_.rightCols(GENERALIZED_VEL_NUM) -=
        1.0 / centroidalModelInfo.mass * f_hat * (centroidalModelPinocchioInterface_.translationalJacobianComToContactPointInWorldFrame(i));
    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i) = 1.0 / centroidalModelInfo.mass * Matrix3::Identity();
    getSkewMatrix<scalar_t>(centroidalModelPinocchioInterface_.positionComToContactPointInWorldFrame(i), p_hat);
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * i) = 1.0 / centroidalModelInfo.mass * p_hat;
  }

  for (size_t i = centroidalModelInfo.numThreeDofContacts;
       i < centroidalModelInfo.numThreeDofContacts + centroidalModelInfo.numSixDofContacts; i++) {
    const size_t inputIdx = 3 * centroidalModelInfo.numThreeDofContacts + 6 * (i - centroidalModelInfo.numThreeDofContacts);
    contactForceInWorldFrame = input.segment<3>(inputIdx);
    getSkewMatrix<scalar_t>(contactForceInWorldFrame, f_hat);
    normalizedAngularMomentumRateDerivativeState_.rightCols(GENERALIZED_VEL_NUM) -=
        1.0 / centroidalModelInfo.mass * f_hat * (centroidalModelPinocchioInterface_.translationalJacobianComToContactPointInWorldFrame(i));

    normalizedLinearMomentumRateDerivativeInput_.block<3, 3>(0, inputIdx) = 1.0 / centroidalModelInfo.mass * Matrix3::Identity();

    getSkewMatrix<scalar_t>(centroidalModelPinocchioInterface_.positionComToContactPointInWorldFrame(i), p_hat);
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx) = 1.0 / centroidalModelInfo.mass * p_hat;
    normalizedAngularMomentumRateDerivativeInput_.block<3, 3>(0, 3 * inputIdx + 3) = 1.0 / centroidalModelInfo.mass * Matrix3::Identity();
  }
}
}  // namespace ocs2
