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

class CentroidalModelRbdConversions final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

  /**
   * Constructor
   * @param [in] pinocchioInterface: predefined pinocchio interface for a robot
   * @param [in] CentroidalModelInfo: The centroidal model information.
   */
  CentroidalModelRbdConversions(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info);

  /**
   * Computes the floating-base generalized positions, velocities, and accelerations.
   *
   * @param [in] state: ocs2 switched-model state vector
   * @param [in] input: ocs2 switched-model input vector
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
   * @note: The input rbdState contains non-local base velocities (expressed in world frame)
   *
   * @param [in] rbdState: rigid body dynamics model state [base pose, joint positions, base twist, joint velocities]
   * @return ocs2 switched-model state vector
   */
  vector_t computeCentroidalStateFromRbdModel(const vector_t& rbdState);

  /**
   * Computes the rigid body dynamics model state from the ocs2 centroidal model state
   * @note: The output rbdState contains non-local base velocities (expressed in world frame)
   *
   * @param [in] state: ocs2 switched-model state vector
   * @param [in] input: ocs2 switched-model input vector
   * @return rigid body dynamics model state [base pose, joint positions, base twist, joint velocities]
   */
  vector_t computeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input);

  /**
   * Computes the rigid body dynamics model torque from the ocs2 centroidal model input
   * @note: Calls computeRbdTorqueFromCentroidalModelPD() with zero gains, so only returning the feedforward torque
   *
   * @param [in] state: ocs2 switched-model state vector
   * @param [in] input: ocs2 switched-model input vector
   * @param [in] jointAccelerations: actuated joints accelerations
   * @return rigid body dynamics model torque [base wrench, joint torques]
   */
  vector_t computeRbdTorqueFromCentroidalModel(const vector_t& state, const vector_t& input, const vector_t& jointAccelerations);

  /**
   * Computes the rigid body dynamics model torque from the ocs2 centroidal model input and adds PD feedback
   * @note: Calls computeRbdTorqueFromCentroidalModelPD() with gains loaded from settings
   *
   * @param [in] desiredState: desired ocs2 switched-model state vector
   * @param [in] desiredInput: desired ocs2 switched-model input vector
   * @param [in] desiredJointAccelerations: desired joint accelerations
   * @param [in] measuredRbdState: measured rigid body dynamics model state (required for PD control)
   * @return rigid body dynamics model torque [base wrench, joint torques]
   */
  vector_t computeRbdTorqueFromCentroidalModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                 const vector_t& desiredJointAccelerations, const vector_t& measuredRbdState);

  /**
   * Computes the rigid body dynamics model torque from the ocs2 centroidal model input and adds PD feedback
   * @note: PD controller is added on the acceleration level
   *
   * @param [in] desiredState: desired ocs2 switched-model state vector
   * @param [in] desiredInput: desired ocs2 switched-model input vector
   * @param [in] desiredJointAccelerations: desired joint accelerations
   * @param [in] measuredRbdState: measured rigid body dynamics model state (required for PD control)
   * @param [in] pGains: proportional gains for [base, joint] tracking
   * @param [in] dGains: derivative gains for [base, joint] tracking
   * @return rigid body dynamics model torque [base wrench, joint torques]
   */
  vector_t computeRbdTorqueFromCentroidalModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                 const vector_t& desiredJointAccelerations, const vector_t& measuredRbdState,
                                                 const vector_t& pGains, const vector_t& dGains);

  /**
   * Load settings from a configuration file
   *
   * @param [in] fileName: File name which contains the configuration data
   * @param [in] fieldName: Field name which contains the configuration data
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not
   */
  void loadSettings(const std::string& fileName, const std::string& fieldName, bool verbose = true);

 private:
  CentroidalModelRbdConversions(const CentroidalModelRbdConversions& other) = default;
  CentroidalModelRbdConversions& operator=(const CentroidalModelRbdConversions& rhs) = default;

  PinocchioInterface pinocchioInterface_;
  CentroidalModelPinocchioMapping mapping_;

  // settings
  vector_t pGains_;
  vector_t dGains_;
};

}  // namespace ocs2
