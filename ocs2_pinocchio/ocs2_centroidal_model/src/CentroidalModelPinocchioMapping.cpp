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
#include "ocs2_centroidal_model/helpers.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
CentroidalModelPinocchioMapping<SCALAR>::CentroidalModelPinocchioMapping(const CentroidalModelInfo& centroidalModelInfo)
    : centroidalModelInfo_(centroidalModelInfo) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  assert(centroidalModelInfo_.stateDim == state.rows());
  const Model& model = pinocchioInterfacePtr_->getModel();
  return state.segment(6, centroidalModelInfo_.generalizedCoordinatesNum);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
  const Model& model = pinocchioInterfacePtr_->getModel();
  const Data& data = pinocchioInterfacePtr_->getData();
  const CentroidalModelInfo& info = centroidalModelInfo_;
  assert(info.stateDim == state.rows());
  assert(info.inputDim == input.rows());

  const auto& A = getCentroidalMomentumMatrix();
  const matrix6_t Ab = A.template leftCols<6>();
  const auto& Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info.actuatedDofNum);

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  const auto normalizedMomentum = getNormalizedMomentum(state, info);
  const auto jointVelocities = getJointVelocities(input, info).head(info.actuatedDofNum);

  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      vPinocchio.template head<6>() = Ab_inv * (info.robotMass * normalizedMomentum - Aj * jointVelocities);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      vPinocchio.template head<6>() = Ab_inv * (info.robotMass * normalizedMomentum);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
      break;
    }
  }

  vPinocchio.tail(info.actuatedDofNum) = jointVelocities;

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  const Model& model = pinocchioInterfacePtr_->getModel();
  const Data& data = pinocchioInterfacePtr_->getData();
  const auto& info = centroidalModelInfo_;
  assert(info.stateDim == state.rows());

  // TODO: Fix. This is currently wrong since dfdx = dfdq * dqdx + dfdv * dvdx, and dvdx is not zero
  matrix_t dfdx = matrix_t::Zero(Jq.rows(), centroidalModelInfo_.stateDim);
  dfdx.middleCols(6, info.generalizedCoordinatesNum) = Jq;

  // TODO: Fix. This is currently wrong since dfdu = dfdv * dvdu, and dvdu is wrong now
  matrix_t dfdu = matrix_t::Zero(Jv.rows(), centroidalModelInfo_.inputDim);
  dfdu.rightCols(info.actuatedDofNum) = Jv.rightCols(info.actuatedDofNum);

  return {dfdx, dfdu};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getCentroidalMomentumMatrix() const -> const matrix6x_t& {
  const Data& data = pinocchioInterfacePtr_->getData();
  return data.Ag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPositionComToContactPointInWorldFrame(size_t contactIndex) const -> vector3_t {
  const Data& data = pinocchioInterfacePtr_->getData();
  return (data.oMf[centroidalModelInfo_.endEffectorFrameIndices[contactIndex]].translation() - data.com[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getTranslationalJacobianComToContactPointInWorldFrame(size_t contactIndex) const
    -> matrix3x_t {
  const Model& model = pinocchioInterfacePtr_->getModel();
  // TODO: Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  Data data = pinocchioInterfacePtr_->getData();

  matrix6x_t jacobianWorldToContactPointInWorldFrame;
  jacobianWorldToContactPointInWorldFrame.setZero(6, centroidalModelInfo_.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, centroidalModelInfo_.endEffectorFrameIndices[contactIndex], pinocchio::LOCAL_WORLD_ALIGNED,
                              jacobianWorldToContactPointInWorldFrame);
  matrix3x_t J_com = getCentroidalMomentumMatrix().template topRows<3>() / centroidalModelInfo_.robotMass;
  if (centroidalModelInfo_.centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    J_com.rightCols(centroidalModelInfo_.actuatedDofNum).setZero();
  }
  return (jacobianWorldToContactPointInWorldFrame.template topRows<3>() - J_com);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::normalizedCentroidalMomentumRate(const vector_t& input) const -> vector6_t {
  const auto& info = centroidalModelInfo_;
  const vector3_t gravityVector = vector3_t(SCALAR(0.0), SCALAR(0.0), SCALAR(-9.81));
  vector6_t normalizedCentroidalMomentumRate;
  normalizedCentroidalMomentumRate << gravityVector, vector3_t::Zero();

  auto getForce = [&](size_t index) { return getContactForce(input, index, info); };
  auto getTorque = [&](size_t index) { return getContactTorque(input, index, info); };

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto contactForceInWorldFrame = getForce(i);
    normalizedCentroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    normalizedCentroidalMomentumRate.template tail<3>() += getPositionComToContactPointInWorldFrame(i).cross(contactForceInWorldFrame);
  }

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const auto contactForceInWorldFrame = getForce(i);
    const auto contactTorqueInWorldFrame = getTorque(i);
    normalizedCentroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    normalizedCentroidalMomentumRate.template tail<3>() +=
        getPositionComToContactPointInWorldFrame(i).cross(contactForceInWorldFrame) + contactTorqueInWorldFrame;
  }

  normalizedCentroidalMomentumRate = normalizedCentroidalMomentumRate / info.robotMass;

  return normalizedCentroidalMomentumRate;
}

// explicit template instantiation
template class ocs2::CentroidalModelPinocchioMapping<ocs2::scalar_t>;
template class ocs2::CentroidalModelPinocchioMapping<ocs2::ad_scalar_t>;

}  // namespace ocs2
