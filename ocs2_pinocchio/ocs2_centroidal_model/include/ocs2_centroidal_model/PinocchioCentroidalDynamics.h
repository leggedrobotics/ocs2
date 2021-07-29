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

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

namespace ocs2 {

/**
 * Centroidal Dynamics:
 *
 * State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
 * @remark: The linear and angular momenta are expressed with respect to the centroidal frame (a frame centered at
 * the CoM and aligned with the inertial frame).
 *
 * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
 * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
 */
class PinocchioCentroidalDynamics final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
  using Matrix3x = Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>;
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

  /**
   * Constructor
   * @param [in] CentroidalModelInfo : The centroidal model information.
   */
  explicit PinocchioCentroidalDynamics(CentroidalModelInfo info);

  /** Copy Constructor */
  PinocchioCentroidalDynamics(const PinocchioCentroidalDynamics& rhs);

  /** Set the pinocchio interface for caching.
   * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
   * @note The pinocchio interface must be set before calling the getters.
   */
  void setPinocchioInterface(const PinocchioInterface& pinocchioInterface) {
    pinocchioInterfacePtr_ = &pinocchioInterface;
    mapping_.setPinocchioInterface(pinocchioInterface);
  }

  /**
   * Computes system flow map x_dot = f(x, u)
   *
   * @param time: time
   * @param state: system state vector
   * @param input: system input vector
   * @return system flow map x_dot = f(x, u)
   *
   * @note requires pinocchioInterface to be updated with:
   *       ocs2::updateCentroidalDynamics(interface, info, q)
   */
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input);

  /**
   * Computes first order approximation of the system flow map x_dot = f(x, u)
   *
   * @param time: time
   * @param state: system state vector
   * @param input: system input vector
   * @return linear approximation of system flow map x_dot = f(x, u)
   *
   * @note requires pinocchioInterface to be updated with:
   *       ocs2::updateCentroidalDynamicsDerivatives(interface, info, q, v)
   */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input);

 private:
  /**
   * Computes the gradients of the normalized centroidal momentum rate (linear + angular) expressed in the centroidal frame
   *
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return: time derivative of normalized centroidal momentum (required for the linear approximation)
   */
  void computeNormalizedCentroidalMomentumRateGradients(const vector_t& state, const vector_t& input);

  const PinocchioInterface* pinocchioInterfacePtr_;
  CentroidalModelPinocchioMapping mapping_;

  // partial derivatives of the system dynamics
  Matrix3x normalizedLinearMomentumRateDerivativeQ_;
  Matrix3x normalizedAngularMomentumRateDerivativeQ_;
  Matrix3x normalizedLinearMomentumRateDerivativeInput_;
  Matrix3x normalizedAngularMomentumRateDerivativeInput_;
};
}  // namespace ocs2
