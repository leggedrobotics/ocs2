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

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2 {

class CentroidalModelRbdConversions {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

  /** Constructor
   *
   * @param pinocchioInterface: predefined pinocchio interface for a robot
   * @param mapping: maps centroidal model states and inputs to pinocchio generalized coordinates and velocities,
   * which are needed for pinocchio functions and algorithms
   */
  CentroidalModelRbdConversions(PinocchioInterface& pinocchioInterface, CentroidalModelPinocchioMapping<scalar_t>& mapping);

  ~CentroidalModelRbdConversions() = default;

  /**
   * Computes the floating-base generalized positions, velocities, and accelerations.
   *
   * @param [in] state: ocs2 state vector
   * @param [in] input: ocs2 input vector
   * @param [in] jointAccelerations: actuated joints accelerations
   * @param [out] basePose: [base position, base orientation (EulerAngles-ZYX)] expressed in the world frame
   * @param [out] baseVelocity: [base linear velocity, base angular velocity] expressed in the world frame
   * @param [out] baseAcceleration: [base linear acceleration, base angular acceleration] expressed in the world frame
   */
  void computeBaseKinematicsFromCentroidalModel(const vector_t& state, const vector_t& input, const vector_t& jointAccelerations,
                                                Vector6& basePose, Vector6& baseVelocity, Vector6& baseAcceleration);

  /**
   * Computes the ocs2 centroidal model state from the rigid body dynamics model state.
   * @note: In rbdState, orientations precede positions and angular velocities precede linear velocities
   * @note: The inputted rbdState contains non-local base velocities (expressed in world frame)
   *
   * @param [in] rbdState: rigid body dynamics model state [base pose, joint positions, base twist, joint velocities]
   * @param [out] state: ocs2 state vector
   */
  void computeCentroidalStateFromRbdModel(const vector_t& rbdState, vector_t& state);

  /**
   * Computes the rigid body dynamics model state from the ocs2 centroidal model state
   * @note: The outputted rbdState contains non-local base velocities (expressed in world frame)
   *
   * @param [in] state: ocs2 state vector
   * @param [in] input: ocs2 input vector
   * @param [in] jointAccelerations: actuated joints accelerations
   * @param [out] rbdState: rigid body dynamics model state [base pose, joint positions, base twist, joint velocities]
   */
  void computeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input, const vector_t& jointAccelerations,
                                          vector_t& rbdState);

 private:
  PinocchioInterface* pinocchioInterfacePtr_;
  CentroidalModelPinocchioMapping<scalar_t>* mappingPtr_;

  Matrix6 Adot_;
  Vector6 qbaseDdot_;
  Vector6 centroidalMomentum_;
  Vector3 derivativeEulerAnglesZyx_;
  Vector3 baseAngularVelocityInWorld_;
  Vector3 baseAngularAccelerationInWorld_;
};

}  // namespace ocs2