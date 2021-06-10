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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "ocs2_centroidal_model/SingleRigidBodyPinocchioMapping.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
SingleRigidBodyPinocchioMapping<SCALAR>::SingleRigidBodyPinocchioMapping(size_t stateDim, size_t inputDim,
                                                                         const vector_t& qPinocchioNominal,
                                                                         const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface)
    : stateDim_(stateDim), inputDim_(inputDim), pinocchioInterface_(pinocchioInterface) {
  const Model& model = pinocchioInterface_.getModel();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  robotMass_ = pinocchio::computeTotalMass(model);

  // make sure the nominal base frame is aligned with the world frame
  qPinocchioNominal_.setZero(GENERALIZED_VEL_NUM);
  qPinocchioNominal_.tail(GENERALIZED_VEL_NUM - 6) = qPinocchioNominal.tail(GENERALIZED_VEL_NUM - 6);
  I_com_nominal_.setZero();
  r_com_base_nominal_.setZero();
  const vector_t vPinocchioNominal = vector_t::Zero(GENERALIZED_VEL_NUM);

  // Create copy since pinocchio::ccrba alters data
  Data data = pinocchioInterface_.getData();
  pinocchio::ccrba(model, data, qPinocchioNominal_, vPinocchioNominal);
  I_com_nominal_ = data.Ig.inertia().matrix();
  r_com_base_nominal_ = qPinocchioNominal_.template head<3>() - data.com[0];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SingleRigidBodyPinocchioMapping<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  assert(stateDim_ == state.rows());
  const Model& model = pinocchioInterface_.getModel();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  return state.segment(6, GENERALIZED_VEL_NUM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SingleRigidBodyPinocchioMapping<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const -> vector_t {
  assert(stateDim_ == state.rows());
  const Model& model = pinocchioInterface_.getModel();
  const Data& data = pinocchioInterface_.getData();
  const size_t GENERALIZED_VEL_NUM = model.nv;
  const size_t ACTUATED_DOF_NUM = GENERALIZED_VEL_NUM - 6;

  const matrix6_t Ab = getCentroidalMomentumMatrix(state).template leftCols<6>();
  const matrix6_t Ab_inv = getFloatingBaseCentroidalMomentumMatrixInverse(Ab);

  vector_t vPinocchio(GENERALIZED_VEL_NUM);
  vPinocchio.template head<6>() = Ab_inv * (robotMass_ * state.template head<6>());
  vPinocchio.tail(ACTUATED_DOF_NUM) = input.tail(ACTUATED_DOF_NUM);

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SingleRigidBodyPinocchioMapping<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
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
auto SingleRigidBodyPinocchioMapping<SCALAR>::getCentroidalMomentumMatrix(const vector_t& state) const -> matrix6x_t {
  const Model& model = pinocchioInterface_.getModel();
  const size_t GENERALIZED_VEL_NUM = model.nv;

  const auto qPinocchio = getPinocchioJointPosition(state);
  const vector3_t eulerAnglesZyx = qPinocchio.template segment<3>(3);
  const matrix3_t T_zyx = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocities(eulerAnglesZyx);
  const matrix3_t R_w_b = getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
  const vector3_t o_r_com_base = R_w_b * r_com_base_nominal_;
  const matrix3_t S_com_base = skewSymmetricMatrix(o_r_com_base);

  matrix6_t Ab;
  Ab.template topLeftCorner<3, 3>() = robotMass_ * matrix3_t::Identity();
  Ab.template bottomLeftCorner<3, 3>() = matrix3_t::Zero();
  Ab.template topRightCorner<3, 3>() = robotMass_ * S_com_base * T_zyx;
  const matrix3_t mat1 = R_w_b * I_com_nominal_;
  const matrix3_t mat2 = R_w_b.transpose() * T_zyx;
  Ab.template bottomRightCorner<3, 3>() = mat1 * mat2;

  matrix6x_t A = matrix6x_t::Zero(GENERALIZED_VEL_NUM);
  A.template leftCols<6>() = Ab;
  return A;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SingleRigidBodyPinocchioMapping<SCALAR>::getComPositionInWorld(const vector_t& state) const -> vector3_t {
  const auto qPinocchio = getPinocchioJointPosition(state);
  const vector3_t eulerAnglesZyx = qPinocchio.template segment<3>(3);
  const matrix3_t R_w_b = getRotationMatrixFromZyxEulerAngles(eulerAnglesZyx);
  const vector3_t o_r_com_base = R_w_b * r_com_base_nominal_;
  return (qPinocchio.template head<3>() - o_r_com_base);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto SingleRigidBodyPinocchioMapping<SCALAR>::getComJacobianInWorld(const vector_t& state) const -> matrix3x_t {
  const matrix3x_t J_com = getCentroidalMomentumMatrix(state).template topRows<3>() / robotMass_;
  return J_com;
}

// explicit template instantiation
template class ocs2::SingleRigidBodyPinocchioMapping<ocs2::scalar_t>;
template class ocs2::SingleRigidBodyPinocchioMapping<ocs2::ad_scalar_t>;

}  // namespace ocs2
