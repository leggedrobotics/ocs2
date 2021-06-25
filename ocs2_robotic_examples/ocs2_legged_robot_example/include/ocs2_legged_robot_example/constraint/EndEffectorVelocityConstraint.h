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

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_legged_robot_example/common/definitions.h>

namespace ocs2 {
namespace legged_robot {

struct EndEffectorVelocityConstraintSettings {
  // Constraints are  A * x + b
  matrix_t A;
  vector_t b;
};

class EndEffectorVelocityConstraint final : public ocs2::StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::StateInputConstraint;
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  explicit EndEffectorVelocityConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const size_t numConstraints);

  ~EndEffectorVelocityConstraint() override = default;
  EndEffectorVelocityConstraint* clone() const override;

  void configure(const EndEffectorVelocityConstraintSettings& settings);

  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input) const override;

  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

 private:
  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

  const size_t numConstraints_;

  EndEffectorVelocityConstraintSettings settings_;
};
}  // namespace legged_robot
}  // namespace ocs2
