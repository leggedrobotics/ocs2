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
CentroidalModelPinocchioMapping<SCALAR>::CentroidalModelPinocchioMapping(size_t stateDim, size_t inputDim,
                                                                         const CentroidalModelType& centroidalModelType,
                                                                         const vector_t& qPinocchioNominal,
                                                                         const std::vector<std::string>& threeDofContactNames,
                                                                         const std::vector<std::string>& sixDofContactNames,
                                                                         const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface)
    : stateDim_(stateDim), inputDim_(inputDim), pinocchioInterface_(pinocchioInterface) {
  const Model& model = pinocchioInterface_.getModel();
  centroidalModelInfo_.robotMass = pinocchio::computeTotalMass(model);
  centroidalModelInfo_.centroidalModelType = centroidalModelType;
  centroidalModelInfo_.qPinocchioNominal = qPinocchioNominal;
  centroidalModelInfo_.numThreeDofContacts = threeDofContactNames.size();
  centroidalModelInfo_.numSixDofContacts = sixDofContactNames.size();
  for (const auto& name : threeDofContactNames) {
    centroidalModelInfo_.endEffectorFrameIndices.push_back(model.getBodyId(name));
  }
  for (const auto& name : sixDofContactNames) {
    centroidalModelInfo_.endEffectorFrameIndices.push_back(model.getBodyId(name));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  assert(stateDim_ == state.rows());
  const Model& model = pinocchioInterface_.getModel();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  return state.segment(6, GENERALIZED_VEL_NUM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
  assert(stateDim_ == state.rows());
  const Model& model = pinocchioInterface_.getModel();
  const Data& data = pinocchioInterface_.getData();
  const CentroidalModelInfo& info = centroidalModelInfo_;
  const size_t GENERALIZED_VEL_NUM = model.nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;

  // It is assumed that pinocchio::computeCentroidalMap(model, data, q) or pinocchio::computeCentroidalMap(model, data, qNominal) is called
  // first
  const matrix6x_t A = getCentroidalMomentumMatrix();
  const matrix6_t Ab = A.template leftCols<6>();
  const matrix6x_t Aj = A.rightCols(ACTUATED_DOF_NUM);
  const matrix6_t Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(Ab);

  vector_t vPinocchio(GENERALIZED_VEL_NUM);
  switch (info.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      vPinocchio.template head<6>() = Ab_inv * (info.robotMass * state.template head<6>() - Aj * input.tail(ACTUATED_DOF_NUM));
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      vPinocchio.template head<6>() = Ab_inv * (info.robotMass * state.template head<6>());
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
      break;
    }
  }

  vPinocchio.tail(ACTUATED_DOF_NUM) = input.tail(ACTUATED_DOF_NUM);

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  assert(stateDim_ == state.rows());
  const Model& model = pinocchioInterface_.getModel();
  const Data& data = pinocchioInterface_.getData();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;

  matrix_t dfdx = matrix_t::Zero(Jq.rows(), stateDim_);
  dfdx.middleCols(6, GENERALIZED_VEL_NUM) = Jq;

  matrix_t dfdu = matrix_t::Zero(Jv.rows(), inputDim_);
  dfdu.rightCols(ACTUATED_DOF_NUM) = Jv.rightCols(ACTUATED_DOF_NUM);

  return {dfdx, dfdu};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getCentroidalMomentumMatrix() const -> matrix6x_t {
  const Data& data = pinocchioInterface_.getData();
  return data.Ag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getPositionComToContactPointInWorldFrame(size_t contactIndex) const -> vector3_t {
  const Data& data = pinocchioInterface_.getData();
  return (data.oMf[centroidalModelInfo_.endEffectorFrameIndices[contactIndex]].translation() - data.com[0]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::getTranslationalJacobianComToContactPointInWorldFrame(size_t contactIndex) const
    -> matrix3x_t {
  const Model& model = pinocchioInterface_.getModel();
  // Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  Data data = pinocchioInterface_.getData();
  const size_t GENERALIZED_VEL_NUM = model.nv;

  matrix6x_t jacobianWorldToContactPointInWorldFrame;
  jacobianWorldToContactPointInWorldFrame.setZero(6, GENERALIZED_VEL_NUM);
  pinocchio::getFrameJacobian(model, data, centroidalModelInfo_.endEffectorFrameIndices[contactIndex], pinocchio::LOCAL_WORLD_ALIGNED,
                              jacobianWorldToContactPointInWorldFrame);
  const matrix3x_t J_com = getCentroidalMomentumMatrix().template topRows<3>() / centroidalModelInfo_.robotMass;
  return (jacobianWorldToContactPointInWorldFrame.template topRows<3>() - J_com);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioMapping<SCALAR>::normalizedCentroidalMomentumRate(const vector_t& input) const -> vector6_t {
  const auto& info = centroidalModelInfo_;
  const vector3_t gravityVector = vector3_t(SCALAR(0.0), SCALAR(0.0), SCALAR(-9.81));
  vector3_t contactForceInWorldFrame;
  vector3_t contactTorqueInWorldFrame;
  vector6_t normalizedCentroidalMomentumRate;
  normalizedCentroidalMomentumRate.setZero();
  normalizedCentroidalMomentumRate.template head<3>() = gravityVector;

  std::cerr << info.endEffectorFrameIndices.front() << "\n";

  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    contactForceInWorldFrame = input.template segment<3>(3 * i);
    normalizedCentroidalMomentumRate.template head<3>() += contactForceInWorldFrame;
    normalizedCentroidalMomentumRate.template tail<3>() += getPositionComToContactPointInWorldFrame(i).cross(contactForceInWorldFrame);
  }

  for (size_t i = info.numThreeDofContacts; i < info.numThreeDofContacts + info.numSixDofContacts; i++) {
    const size_t inputIdx = 3 * info.numThreeDofContacts + 6 * (i - info.numThreeDofContacts);
    contactForceInWorldFrame = input.template segment<3>(inputIdx);
    contactTorqueInWorldFrame = input.template segment<3>(inputIdx + 3);
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
