/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_centroidal_model/CentroidalModelInfo.h"

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2 {

/**
 * Centroidal Dynamics:
 *
 * State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
 * @remark: The linear and angular momenta are expressed with respect to
 * the centroidal frame (a frame centered at the CoM and aligned with the inertial frame)
 *
 * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
 * @remark: Contact forces and wrenches are expressed with respect to the inertial frame
 *
 *
 * Pinocchio Joint Positions: qPinocchio = [ base_position, base_orientation_zyx, joint_positions ]'
 * @remark: Base position is expressed with respect to the inertial frame
 *
 * Pinocchio Joint Velocities: vPinocchio = [ base_linear_velocity, base_orientation_zyx_derivatives, joint_velocities ]'
 * @remark: Base linear velocity is expressed with respect to the inertial frame
 */

template <typename SCALAR>
class CentroidalModelPinocchioMapping final : public PinocchioStateInputMapping<SCALAR> {
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

  using CentroidalModelInfo = CentroidalModelInfoTpl<SCALAR>;

  explicit CentroidalModelPinocchioMapping(const CentroidalModelInfo& centroidalModelInfo)
      : pinocchioInterfacePtr_(nullptr), centroidalModelInfo_(centroidalModelInfo) {}

  ~CentroidalModelPinocchioMapping() override = default;
  CentroidalModelPinocchioMapping<SCALAR>* clone() const override { return new CentroidalModelPinocchioMapping<SCALAR>(*this); }

  /** Sets the pinocchio interface for caching
   * @note The pinocchio interface must be set before calling the getters.
   * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
   */
  void setPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) override {
    pinocchioInterfacePtr_ = &pinocchioInterface;
  }

  /**
   * Computes the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state variables
   *
   * @param [in] state: system state vector
   * @return pinocchio joint positions, which are also the robot's generalized positions
   * with a ZYX-Euler angle parameterization of the base orientation
   */
  vector_t getPinocchioJointPosition(const vector_t& state) const override;

  /**
   * Computes the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input variables
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return pinocchio joint velocities, which are also the time derivatives of the pinocchio joint positions
   *
   * @warning: The function updateCentroidalDynamics(interface, info, q) should have been called first.
   */
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  /**
   * Maps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
   * @param [in] state: system state vector
   * @param [in] Jq: jacobian with respect to pinocchio joint positions
   * @param [in] Jv: jacobian with respect to pinocchio joint velocities
   * @return a pair {dfdx, dfdu} containing the jacobians with respect to the system state and input
   *
   * @warning: The function updateCentroidalDynamicsDerivatives(interface, info, q, v) should have been called first
   */
  // TODO: Add Jacobian with respect to generalized accelerations as argument to get a full implicit dependence on the inputs
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  /** Returns a structure containing robot-specific information needed for the centroidal dynamics computations
   *
   * @return centroidalModelInfo_
   */
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }

  /** Returns the centroidal momentum matrix from the pinocchioInterface data
   * @return centroidal momentum matrix from data.Ag
   *
   * @warning: The function updateCentroidalDynamics(interface, info, q) should have been called first.
   */
  const matrix6x_t& getCentroidalMomentumMatrix() const;

  /**
   * Computes the CoM to contact point position in world frame
   *
   * @param [in] contactIndex: index of the contact point
   * @return: position of the contact point w.r.t CoM expressed in world frame
   *
   * @warning: The function updateCentroidalDynamics(interface, info, q) should have been called first.
   */
  vector3_t getPositionComToContactPointInWorldFrame(size_t contactIndex) const;

  /**
   * Computes the CoM to contact point translational Jacobian in world frame
   *
   * @param [in] contactIndex: index of the contact point
   * @return: CoM to contact point translational Jacobian expressed in world frame
   *
   * @warning: The function updateCentroidalDynamics(interface, info, q) should have been called first,
   * followed by pinocchio::computeJointJacobians(model, data, q) and updateFramePlacements(model, data)
   */
  matrix3x_t getTranslationalJacobianComToContactPointInWorldFrame(size_t contactIndex) const;

  /**
   * Computes the derivative of the normalized centroidal momentum (linear + angular) expressed in the centroidal frame
   *
   * @param [in] input: system input vector
   * @return: time derivative of normalized centroidal momentum
   *
   * @warning: The function updateCentroidalDynamics(interface, info, q) should have been called first.
   */
  vector6_t getNormalizedCentroidalMomentumRate(const vector_t& input) const;

 private:
  /** Copy Constructor */
  CentroidalModelPinocchioMapping(const CentroidalModelPinocchioMapping& rhs)
      : pinocchioInterfacePtr_(nullptr), centroidalModelInfo_(rhs.centroidalModelInfo_) {}

  const PinocchioInterfaceTpl<SCALAR>* pinocchioInterfacePtr_;
  const CentroidalModelInfoTpl<SCALAR> centroidalModelInfo_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class CentroidalModelPinocchioMapping<scalar_t>;
extern template class CentroidalModelPinocchioMapping<ad_scalar_t>;
}  // namespace ocs2
