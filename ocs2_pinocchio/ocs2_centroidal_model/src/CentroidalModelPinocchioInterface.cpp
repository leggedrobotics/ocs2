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

#include "ocs2_centroidal_model/CentroidalModelPinocchioInterface.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
CentroidalModelPinocchioInterface<SCALAR>::CentroidalModelPinocchioInterface(const CentroidalModelType& centroidalModelType,
                                                                             const vector_t& qPinocchioNominal,
                                                                             const std::vector<std::string>& threeDofContactNames,
                                                                             const std::vector<std::string>& sixDofContactNames,
                                                                             const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface)
    : pinocchioInterface_(pinocchioInterface) {
  initializeCentroidalModelInfo(centroidalModelType, qPinocchioNominal, threeDofContactNames, sixDofContactNames);
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void CentroidalModelPinocchioInterface<SCALAR>::initializeCentroidalModelInfo(const CentroidalModelType& centroidalModelType,
                                                                              const vector_t& qPinocchioNominal,
                                                                              const std::vector<std::string>& threeDofContactNames,
                                                                              const std::vector<std::string>& sixDofContactNames) {
  centroidalModelInfo_.centroidalModelType = centroidalModelType;
  centroidalModelInfo_.numThreeDofContacts = threeDofContactNames.size();
  centroidalModelInfo_.numSixDofContacts = sixDofContactNames.size();
  for (const auto& name : threeDofContactNames) {
    centroidalModelInfo_.endEffectorFrameIndices.push_back(getRobotModel().getBodyId(name));
  }
  for (const auto& name : sixDofContactNames) {
    centroidalModelInfo_.endEffectorFrameIndices.push_back(getRobotModel().getBodyId(name));
  }

  const size_t GENERALIZED_VEL_NUM = getRobotModel().nv;
  centroidalModelInfo_.qPinocchio.setZero(GENERALIZED_VEL_NUM);
  centroidalModelInfo_.vPinocchio.setZero(GENERALIZED_VEL_NUM);
  centroidalModelInfo_.mass = pinocchio::computeTotalMass(getRobotModel());
  centroidalModelInfo_.r_com.setZero();
  centroidalModelInfo_.J_com.setZero(3, GENERALIZED_VEL_NUM);
  centroidalModelInfo_.A.setZero(6, GENERALIZED_VEL_NUM);
  centroidalModelInfo_.Ab.setZero();
  centroidalModelInfo_.Aj.setZero(6, GENERALIZED_VEL_NUM - 6);
  centroidalModelInfo_.Ab_inv.setZero();

  // make sure the nominal base frame is aligned with the world frame
  centroidalModelInfo_.qPinocchio_nominal.setZero(GENERALIZED_VEL_NUM);
  centroidalModelInfo_.qPinocchio_nominal.tail(GENERALIZED_VEL_NUM - 6) = qPinocchioNominal.tail(GENERALIZED_VEL_NUM - 6);
  centroidalModelInfo_.I_com_nominal.setZero();
  centroidalModelInfo_.r_com_base_nominal.setZero();

  if (centroidalModelInfo_.centroidalModelType == CentroidalModelType::SingleRigidBodyDynamics) {
    const vector_t vPinocchioNominal = vector_t::Zero(GENERALIZED_VEL_NUM);
    centroidalModelInfo_.A = pinocchio::ccrba(getRobotModel(), getRobotData(), centroidalModelInfo_.qPinocchio_nominal, vPinocchioNominal);
    centroidalModelInfo_.A.template rightCols(GENERALIZED_VEL_NUM - 6).setZero();
    centroidalModelInfo_.Ab = centroidalModelInfo_.A.template leftCols<6>();
    centroidalModelInfo_.I_com_nominal = getRobotData().Ig.inertia().matrix();
    centroidalModelInfo_.r_com_base_nominal = centroidalModelInfo_.qPinocchio_nominal.template head<3>() - getRobotData().com[0];
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void CentroidalModelPinocchioInterface<SCALAR>::updatePinocchioJointPositions(const vector_t& state) {
  const size_t GENERALIZED_VEL_NUM = getRobotModel().nv;
  centroidalModelInfo_.qPinocchio = state.template segment(6, GENERALIZED_VEL_NUM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void CentroidalModelPinocchioInterface<SCALAR>::updatePinocchioJointVelocities(const vector_t& state, const vector_t& input) {
  const size_t GENERALIZED_VEL_NUM = getRobotModel().nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;

  switch (centroidalModelInfo_.centroidalModelType) {
    case CentroidalModelType::FullCentroidalDynamics: {
      centroidalModelInfo_.A = pinocchio::computeCentroidalMap(getRobotModel(), getRobotData(), centroidalModelInfo_.qPinocchio);
      centroidalModelInfo_.r_com = getRobotData().com[0];
      centroidalModelInfo_.J_com = centroidalModelInfo_.A.template topRows<3>() / centroidalModelInfo_.mass;
      centroidalModelInfo_.Ab = centroidalModelInfo_.A.template leftCols<6>();
      centroidalModelInfo_.Aj = centroidalModelInfo_.A.template rightCols(ACTUATED_DOF_NUM);
      centroidalModelInfo_.Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(centroidalModelInfo_.Ab);
      centroidalModelInfo_.vPinocchio.template head<6>() =
          centroidalModelInfo_.Ab_inv *
          (centroidalModelInfo_.mass * state.template head<6>() - centroidalModelInfo_.Aj * input.template tail(ACTUATED_DOF_NUM));
      centroidalModelInfo_.vPinocchio.template tail(ACTUATED_DOF_NUM) = input.template tail(ACTUATED_DOF_NUM);
      break;
    }
    case CentroidalModelType::SingleRigidBodyDynamics: {
      const vector3_t eulerAnglesZyx = centroidalModelInfo_.qPinocchio.template segment<3>(3);
      const matrix3_t T_zyx = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocities(eulerAnglesZyx);
      const matrix3_t R_w_b = getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
      const vector3_t o_r_com_base = R_w_b * centroidalModelInfo_.r_com_base_nominal;
      const matrix3_t S_com_base = skewSymmetricMatrix(o_r_com_base);
      centroidalModelInfo_.Ab.template topRightCorner<3, 3>() = centroidalModelInfo_.mass * S_com_base * T_zyx;
      const matrix3_t mat1 = R_w_b * centroidalModelInfo_.I_com_nominal;
      const matrix3_t mat2 = R_w_b.transpose() * T_zyx;
      centroidalModelInfo_.Ab.template bottomRightCorner<3, 3>() = mat1 * mat2;
      centroidalModelInfo_.Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(centroidalModelInfo_.Ab);
      centroidalModelInfo_.A.template leftCols<6>() = centroidalModelInfo_.Ab;
      centroidalModelInfo_.r_com = centroidalModelInfo_.qPinocchio.template head<3>() - o_r_com_base;
      centroidalModelInfo_.J_com = centroidalModelInfo_.A.template topRows<3>() / centroidalModelInfo_.mass;
      centroidalModelInfo_.vPinocchio.template head<6>() =
          centroidalModelInfo_.Ab_inv * (centroidalModelInfo_.mass * state.template head<6>());
      centroidalModelInfo_.vPinocchio.template tail(ACTUATED_DOF_NUM) = input.template tail(ACTUATED_DOF_NUM);
      break;
    }
    default: {
      throw std::runtime_error("The chosen centroidal model type is not supported.");
      break;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioInterface<SCALAR>::positionWorldToContactPointInWorldFrame(size_t contactIndex) const -> vector3_t {
  return getRobotData().oMf[centroidalModelInfo_.endEffectorFrameIndices[contactIndex]].translation();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioInterface<SCALAR>::positionComToContactPointInWorldFrame(size_t contactIndex) const -> vector3_t {
  return positionWorldToContactPointInWorldFrame(contactIndex) - centroidalModelInfo_.r_com;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioInterface<SCALAR>::translationalJacobianWorldToContactPointInWorldFrame(size_t contactIndex) -> matrix3x_t {
  const size_t GENERALIZED_VEL_NUM = getRobotModel().nv;
  matrix6x_t jacobianWorldToContactPointInWorldFrame;
  jacobianWorldToContactPointInWorldFrame.setZero(6, GENERALIZED_VEL_NUM);
  pinocchio::getFrameJacobian(getRobotModel(), getRobotData(), centroidalModelInfo_.endEffectorFrameIndices[contactIndex],
                              pinocchio::LOCAL_WORLD_ALIGNED, jacobianWorldToContactPointInWorldFrame);
  return jacobianWorldToContactPointInWorldFrame.template topRows<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioInterface<SCALAR>::translationalJacobianComToContactPointInWorldFrame(size_t contactIndex) -> matrix3x_t {
  return translationalJacobianWorldToContactPointInWorldFrame(contactIndex) - centroidalModelInfo_.J_com;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioInterface<SCALAR>::linearVelocityWorldToContactPointInWorldFrame(size_t contactIndex) -> vector3_t {
  return translationalJacobianWorldToContactPointInWorldFrame(contactIndex) * centroidalModelInfo_.vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto CentroidalModelPinocchioInterface<SCALAR>::normalizedCentroidalMomentumRate(const vector_t& input) const -> vector6_t {
  const Eigen::Matrix<SCALAR, 3, 1> gravityVector{SCALAR(0.0), SCALAR(0.0), SCALAR(-9.81)};
  vector3_t contactForceInWorldFrame;
  vector3_t contactTorqueInWorldFrame;
  vector6_t normalizedCentroidalMomentumRate;
  normalizedCentroidalMomentumRate.setZero();
  normalizedCentroidalMomentumRate.template head<3>() = gravityVector;

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    contactForceInWorldFrame = input.template segment<3>(3 * i);
    normalizedCentroidalMomentumRate.template head<3>() += 1.0 / centroidalModelInfo_.mass * contactForceInWorldFrame;
    normalizedCentroidalMomentumRate.template tail<3>() +=
        1.0 / centroidalModelInfo_.mass * (positionComToContactPointInWorldFrame(i).cross(contactForceInWorldFrame));
  }

  for (size_t i = centroidalModelInfo_.numThreeDofContacts;
       i < centroidalModelInfo_.numThreeDofContacts + centroidalModelInfo_.numSixDofContacts; i++) {
    const size_t inputIdx = 3 * centroidalModelInfo_.numThreeDofContacts + 6 * (i - centroidalModelInfo_.numThreeDofContacts);
    contactForceInWorldFrame = input.template segment<3>(inputIdx);
    contactTorqueInWorldFrame = input.template segment<3>(inputIdx + 3);
    normalizedCentroidalMomentumRate.template head<3>() += 1.0 / centroidalModelInfo_.mass * contactForceInWorldFrame;
    normalizedCentroidalMomentumRate.template tail<3>() +=
        1.0 / centroidalModelInfo_.mass *
        ((positionComToContactPointInWorldFrame(i).cross(contactForceInWorldFrame)) + contactTorqueInWorldFrame);
  }

  return normalizedCentroidalMomentumRate;
}

// explicit template instantiation
template class ocs2::CentroidalModelPinocchioInterface<ocs2::scalar_t>;
template class ocs2::CentroidalModelPinocchioInterface<ocs2::ad_scalar_t>;

}  // namespace ocs2
