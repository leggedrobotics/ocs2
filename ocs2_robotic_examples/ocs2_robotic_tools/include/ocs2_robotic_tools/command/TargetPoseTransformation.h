/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>

namespace ocs2 {

/**
 * This class transforms the 6Dof goal representation in between "Target Pose" representation and
 * "Desired State" representation.
 *
 * Target Pose:
 *
 * targetPoseDisplacementVelocity(0): Roll \\
 * targetPoseDisplacementVelocity(1): Pitch \\
 * targetPoseDisplacementVelocity(2): Yaw \\
 *
 * targetPoseDisplacementVelocity(3): X \\
 * targetPoseDisplacementVelocity(4): Y \\
 * targetPoseDisplacementVelocity(5): Z \\
 *
 * targetPoseDisplacementVelocity(6): \omega_X \\
 * targetPoseDisplacementVelocity(7): \omega_Y \\
 * targetPoseDisplacementVelocity(8): \omega_Z \\
 *
 * targetPoseDisplacementVelocity(9):  v_X \\
 * targetPoseDisplacementVelocity(10): v_Y \\
 * targetPoseDisplacementVelocity(11): v_Z \\
 *
 *
 * Desired State:
 *
 * dState = costDesiredTrajectories.desiredStateTrajectory().at(0)
 *
 * dState(0): orientation.w \\
 * dState(1): orientation.x \\
 * dState(2): orientation.y \\
 * dState(3): orientation.z \\
 *
 * dState(4): position.x \\
 * dState(5): position.y \\
 * dState(6): position.z \\
 *
 * dState(7): angular_velocity.x \\
 * dState(8): angular_velocity.y \\
 * dState(9): angular_velocity.y \\
 *
 * dState(10): linear_velocity.x \\
 * dState(11): linear_velocity.y \\
 * dState(12): linear_velocity.z \\
 *
 */
template <typename SCALAR_T>
class TargetPoseTransformation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum DIM : size_t {
    POSE_DIM_ = 6,
  };

  using scalar_t = SCALAR_T;
  using scalar_array_t = std::vector<scalar_t>;
  using pose_vector_t = Eigen::Matrix<scalar_t, POSE_DIM_, 1>;
  using dynamic_vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;

  /**
   * Constructor
   */
  TargetPoseTransformation() = default;

  /**
   * Destructor
   */
  ~TargetPoseTransformation() = default;

  /**
   * Transforms the cost function's desired state to the target pose displacement.
   *
   * @param [in] desiredState: The cost function's desired state.
   * @param [out] targetPoseDisplacement: The target pose displacement.
   * @param [out] targetVelocity: The target velocity.
   */
  static void toTargetPoseDisplacement(const dynamic_vector_t& desiredState, pose_vector_t& targetPoseDisplacement,
                                       pose_vector_t& targetVelocity) {
    Eigen::Quaternion<scalar_t> qxyz(desiredState(0), desiredState(1), desiredState(2), desiredState(3));

    targetPoseDisplacement.template head<3>() = qxyz.toRotationMatrix().eulerAngles(0, 1, 2);
    targetPoseDisplacement(3) = desiredState(4);
    targetPoseDisplacement(4) = desiredState(5);
    targetPoseDisplacement(5) = desiredState(6);

    // if velocities are included
    if (desiredState.size() > POSE_DIM_ + 1) {
      targetVelocity(0) = desiredState(7);
      targetVelocity(1) = desiredState(8);
      targetVelocity(2) = desiredState(9);
      targetVelocity(3) = desiredState(10);
      targetVelocity(4) = desiredState(11);
      targetVelocity(5) = desiredState(12);

    } else {
      targetVelocity.setZero();
    }
  }

  /**
   * Transforms the target pose displacement to the cost function's desired state.
   *
   * @param [in] targetPoseDisplacementVelocity: The target base pose displacement and base local velocities.
   * @param [out] desiredState: The cost function's desired state.
   */
  static void toCostDesiredState(const scalar_array_t& targetPoseDisplacementVelocity, dynamic_vector_t& desiredState) {
    if (targetPoseDisplacementVelocity.size() < 12) {
      throw std::runtime_error("target command should have at least 12 elements.");
    }

    auto deg2rad = [](const scalar_t& deg) { return (deg * M_PI / 180.0); };

    desiredState.resize(targetPoseDisplacementVelocity.size() + 1);

    // Orientation
    Eigen::Quaternion<scalar_t> qxyz =
        Eigen::AngleAxis<scalar_t>(deg2rad(targetPoseDisplacementVelocity[0]), Eigen::Vector3d::UnitX()) *  // roll
        Eigen::AngleAxis<scalar_t>(deg2rad(targetPoseDisplacementVelocity[1]), Eigen::Vector3d::UnitY()) *  // pitch
        Eigen::AngleAxis<scalar_t>(deg2rad(targetPoseDisplacementVelocity[2]), Eigen::Vector3d::UnitZ());   // yaw

    desiredState(0) = qxyz.w();
    desiredState(1) = qxyz.x();
    desiredState(2) = qxyz.y();
    desiredState(3) = qxyz.z();

    // Position
    desiredState(4) = targetPoseDisplacementVelocity[3];
    desiredState(5) = targetPoseDisplacementVelocity[4];
    desiredState(6) = targetPoseDisplacementVelocity[5];

    // if velocities are included
    if (targetPoseDisplacementVelocity.size() > POSE_DIM_) {
      // Angular velocity
      desiredState(7) = targetPoseDisplacementVelocity[6];
      desiredState(8) = targetPoseDisplacementVelocity[7];
      desiredState(9) = targetPoseDisplacementVelocity[8];

      // Linear velocity
      desiredState(10) = targetPoseDisplacementVelocity[9];
      desiredState(11) = targetPoseDisplacementVelocity[10];
      desiredState(12) = targetPoseDisplacementVelocity[11];
    }
  }

 private:
};

}  // namespace ocs2
