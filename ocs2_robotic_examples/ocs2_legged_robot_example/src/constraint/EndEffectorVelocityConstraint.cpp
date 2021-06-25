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

#include <ocs2_legged_robot_example/constraint/EndEffectorVelocityConstraint.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorVelocityConstraint::EndEffectorVelocityConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                             const size_t numConstraints)
    : BASE(ConstraintOrder::Linear), endEffectorKinematicsPtr_(endEffectorKinematics.clone()), numConstraints_(numConstraints) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorVelocityConstraint::EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs)
    : BASE(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      numConstraints_(rhs.numConstraints_),
      settings_(rhs.settings_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorVelocityConstraint* EndEffectorVelocityConstraint::clone() const {
  return new EndEffectorVelocityConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorVelocityConstraint::configure(const EndEffectorVelocityConstraintSettings& settings) {
  assert(settings.A.rows() == numConstraints_);
  assert(settings.A.cols() == 6);
  assert(settings.b.rows() == numConstraints_);
  settings_ = settings;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorVelocityConstraint::getNumConstraints(scalar_t time) const {
  return numConstraints_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorVelocityConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  // Compute constraints
  const vector3_t eeVelocityWorld = endEffectorKinematicsPtr_->getVelocity(state, input)[0];
  const vector3_t eePositionWorld = endEffectorKinematicsPtr_->getPosition(state)[0];

  return settings_.A * (vector_t(6) << eeVelocityWorld, eePositionWorld).finished() + settings_.b;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorVelocityConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input) const {
  // Convert to output format
  VectorFunctionLinearApproximation linearApproximation(getNumConstraints(time), state.size(), input.size());
  const auto velocityLinearApproximation = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input)[0];
  const auto positionLinearApproximation = endEffectorKinematicsPtr_->getPositionLinearApproximation(state)[0];

  linearApproximation.f =
      settings_.A * (vector_t(6) << velocityLinearApproximation.f, positionLinearApproximation.f).finished() + settings_.b;
  linearApproximation.dfdx =
      settings_.A * (matrix_t(6, state.size()) << velocityLinearApproximation.dfdx, positionLinearApproximation.dfdx).finished();
  linearApproximation.dfdu =
      settings_.A * (matrix_t(6, input.size()) << velocityLinearApproximation.dfdu, matrix_t::Zero(3, input.size())).finished();

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
