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

#include "ocs2_centroidal_model/utils.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <unordered_map>

namespace ocs2 {

/**
 * Centroidal Dynamics:
 * State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
 * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
 * Pinocchio Joint Positions: qPinocchio = [ base_position, base_orientation_zyx, joint_positions ]'
 * Pinocchio Joint Velocities: vPinocchio = [ base_linear_velocity, base_orientation_zyx_derivatives, joint_velocities ]'
 */

template <typename SCALAR>
class CentroidalModelPinocchioInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  using vector6_t = Eigen::Matrix<SCALAR, 6, 1>;
  using vector3_t = Eigen::Matrix<SCALAR, 3, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR, 3, Eigen::Dynamic>;
  using matrix6x_t = Eigen::Matrix<SCALAR, 6, Eigen::Dynamic>;
  using matrix3_t = Eigen::Matrix<SCALAR, 3, 3>;
  using matrix6_t = Eigen::Matrix<SCALAR, 6, 6>;

  using Model = pinocchio::ModelTpl<SCALAR>;
  using Data = typename Model::Data;

  enum class CentroidalModelType { FullCentroidalDynamics, SingleRigidBodyDynamics };

  struct CentroidalModelInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CentroidalModelType centroidalModelType;      // full centroidal dynamics OR single rigid body dynamics (SRBD)
    size_t numThreeDofContacts;                   // 3DOF contacts, force only
    size_t numSixDofContacts;                     // 6DOF contacts, force and torque
    std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF contacts, 6DOF contacts]
    vector_t qPinocchio;                          // pinocchio joint positions deduced from the centroidal dynamics
    vector_t vPinocchio;                          // pinocchio joint velocities deduced from the centroidal dynamics
    SCALAR mass;                                  // total robot mass
    vector3_t r_com;                              // position of center-of-mass in world frame
    matrix_t J_com;                               // jacobian mapping joint velocities to CoM linear velocities
    matrix_t A;                                   // full Centroidal Momentum Matrix (CMM)
    matrix6_t Ab;                                 // base part of CMM
    matrix_t Aj;                                  // joint part of CMM
    matrix6_t Ab_inv;                             // inverse of Ab
    vector_t qPinocchio_nominal;                  // nominal robot configuration used in the SRBD model
    matrix3_t I_com_nominal;                      // nominal robot centroidal inertia used in the SRBD model
    vector3_t r_com_base_nominal;                 // nominal CoM to base position used in the SRBD model
  };

  CentroidalModelPinocchioInterface(const CentroidalModelType& centroidalModelType, const vector_t& qPinocchioNominal,
                                    const std::vector<std::string>& threeDofContactNames,
                                    const std::vector<std::string>& sixDofContactNames,
                                    const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface)
      : pinocchioInterface_(pinocchioInterface) {
    initializeCentroidalModelInfo(centroidalModelType, qPinocchioNominal, threeDofContactNames, sixDofContactNames);
  };

  void initializeCentroidalModelInfo(const CentroidalModelType& centroidalModelType, const vector_t& qPinocchioNominal,
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
      centroidalModelInfo_.A =
          pinocchio::ccrba(getRobotModel(), getRobotData(), centroidalModelInfo_.qPinocchio_nominal, vPinocchioNominal);
      centroidalModelInfo_.A.template rightCols(GENERALIZED_VEL_NUM - 6).setZero();
      centroidalModelInfo_.Ab = centroidalModelInfo_.A.template leftCols<6>();
      centroidalModelInfo_.I_com_nominal = getRobotData().Ig.inertia().matrix();
      centroidalModelInfo_.r_com_base_nominal = centroidalModelInfo_.qPinocchio_nominal.template head<3>() - getRobotData().com[0];
    }
  }

  ~CentroidalModelPinocchioInterface() = default;

  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  const Model& getRobotModel() const { return pinocchioInterface_.getModel(); }
  Data& getRobotData() { return pinocchioInterface_.getData(); }
  const Data& getRobotData() const { return pinocchioInterface_.getData(); }

  /**
   * Updates the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state variables
   *
   * @param [in] state: system state vector
   */
  void updatePinocchioJointPositions(const vector_t& state) {
    const size_t GENERALIZED_VEL_NUM = getRobotModel().nv;
    centroidalModelInfo_.qPinocchio = state.template segment(6, GENERALIZED_VEL_NUM);
  }

  /**
   * Updates the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input variables
   * @remark: This function internally calls pinocchio::forwardKinematics + pinocchio::computeJointJacobians (only for the
   * FullCentroidalDynamics case) and updates the centroidalModelInfo.
   *
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   *
   * @warning: The function updatePinocchioJointPositions should have been called before.
   */
  void updatePinocchioJointVelocities(const vector_t& state, const vector_t& input) {
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
        vector3_t o_r_com_base = R_w_b * centroidalModelInfo_.r_com_base_nominal;
        matrix3_t S_com_base = skewSymmetricMatrix(o_r_com_base);
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

  /**
   * Computes the world to contact point position in world frame
   * This method assumes that pinocchio::ForwardKinematics has been called, followed by pinocchio::updateFramePlacements
   *
   * @param [in] contactIndex: index of the contact point
   * @return: position of the contact point expressed in world frame
   *
   * @warning: The function pinocchio::framesForwardKinematics (i.e., pinocchio::forwardKinematics followed by
   * pinocchio::updateFramePlacements) should have been called before.
   */
  vector3_t positionWorldToContactPointInWorldFrame(size_t contactIndex) const {
    return getRobotData().oMf[centroidalModelInfo_.endEffectorFrameIndices[contactIndex]].translation();
  }

  /**
   * Computes the CoM to contact point position in world frame
   *
   * @param [in] contactIndex: index of the contact point
   * @return: position of the contact point w.r.t CoM expressed in world frame
   *
   * @warning: The functions updatePinocchioJointVelocities and pinocchio::framesForwardKinematics should have been called before.
   */
  vector3_t positionComToContactPointInWorldFrame(size_t contactIndex) const {
    return positionWorldToContactPointInWorldFrame(contactIndex) - centroidalModelInfo_.r_com;
  }

  /**
   * Computes the world to contact point translational Jacobian in world frame
   *
   * @param [in] contactIndex: index of the contact point
   * @return: contact point translational Jacobian in world frame
   *
   * @warning: The function pinocchio::computeJointJacobians followed by pinocchio::framesForwardKinematics should have been called before.
   */
  matrix3x_t translationalJacobianWorldToContactPointInWorldFrame(size_t contactIndex) {
    const size_t GENERALIZED_VEL_NUM = getRobotModel().nv;
    matrix6x_t jacobianWorldToContactPointInWorldFrame;
    jacobianWorldToContactPointInWorldFrame.setZero(6, GENERALIZED_VEL_NUM);
    pinocchio::getFrameJacobian(getRobotModel(), getRobotData(), centroidalModelInfo_.endEffectorFrameIndices[contactIndex],
                                pinocchio::LOCAL_WORLD_ALIGNED, jacobianWorldToContactPointInWorldFrame);
    return jacobianWorldToContactPointInWorldFrame.template topRows<3>();
  }

  /**
   * Computes the CoM to contact point translational Jacobian in world frame
   *
   * @param [in] contactIndex: index of the contact point
   * @return: CoM to contact point translational Jacobian expressed in world frame
   *
   * @warning: The functions updatePinocchioJointVelocities, pinocchio::computeJointJacobians and pinocchio::framesForwardKinematics
   * should have been called before.
   */
  matrix3x_t translationalJacobianComToContactPointInWorldFrame(size_t contactIndex) {
    return translationalJacobianWorldToContactPointInWorldFrame(contactIndex) - centroidalModelInfo_.J_com;
  }

  /**
   * Computes the absolute linear velocity of the contact point in world frame
   *
   * @param [in] contactIndex: index of the contact point
   * @return: absolute linear velocity of the contact point expressed in world frame
   *
   * @warning: The functions updatePinocchioJointVelocities, pinocchio::computeJointJacobians and pinocchio::framesForwardKinematics
   * should have been called before.
   */
  vector3_t linearVelocityWorldToContactPointInWorldFrame(size_t contactIndex) {
    return translationalJacobianWorldToContactPointInWorldFrame(contactIndex) * centroidalModelInfo_.vPinocchio;
  }

  /**
   * Computes the derivative of the normalized centroidal momentum (linear + angular) expressed in the centroidal frame
   *
   * @param [in] input: system input vector
   * @return: time derivative of normalized centroidal momentum
   *
   * @warning: The functions updatePinocchioJointVelocities and pinocchio::framesForwardKinematics should have been called before.
   */
  vector6_t normalizedCentroidalMomentumRate(const vector_t& input) const {
    const Eigen::Matrix<scalar_t, 3, 1> gravityVector = Eigen::Matrix<scalar_t, 3, 1>{0.0, 0.0, -9.81};
    vector3_t contactForceInWorldFrame;
    vector3_t contactTorqueInWorldFrame;
    vector6_t normalizedCentroidalMomentumRate;
    normalizedCentroidalMomentumRate.setZero();
    normalizedCentroidalMomentumRate.template head<3>() = gravityVector.cast<SCALAR>();

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

 private:
  PinocchioInterfaceTpl<SCALAR> pinocchioInterface_;
  CentroidalModelInfo centroidalModelInfo_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class CentroidalModelPinocchioInterface<scalar_t>;
extern template class CentroidalModelPinocchioInterface<ad_scalar_t>;

}  // namespace ocs2
