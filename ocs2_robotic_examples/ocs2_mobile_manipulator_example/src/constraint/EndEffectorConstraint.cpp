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

#include <ocs2_mobile_manipulator_example/constraint/EndEffectorConstraint.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const ocs2::EndEffectorKinematics<scalar_t>& endEffectorKinematics)
    : eeDesiredPosition_(vector3_t::Zero()),
      eeDesiredOrientation_(1.0, 0.0, 0.0, 0.0),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()) {
  assert(endEffectorKinematics.getIds().size() == 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const EndEffectorConstraint& rhs)
    : ocs2::StateConstraint(rhs),
      eeDesiredPosition_(vector3_t::Zero()),
      eeDesiredOrientation_(1.0, 0.0, 0.0, 0.0),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const {
  return 6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorConstraint::getValue(scalar_t time, const vector_t& state) const {
  vector_t constraint(6);
  constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state)[0] - eeDesiredPosition_;
  constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(state, {eeDesiredOrientation_})[0];
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(scalar_t time, const vector_t& state) const {
  auto constraintApproximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state)[0];
  constraintApproximation.f.head<3>() = eePosition.f - eeDesiredPosition_;
  constraintApproximation.dfdx.topRows<3>() = eePosition.dfdx;

  const auto eeOrientationError = endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {eeDesiredOrientation_})[0];
  constraintApproximation.f.tail<3>() = eeOrientationError.f;
  constraintApproximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;

  return constraintApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorConstraint::setDesiredPose(const vector3_t& position, const quaternion_t& orientation) {
  eeDesiredPosition_ = position;
  eeDesiredOrientation_ = orientation;
}

}  // namespace mobile_manipulator
