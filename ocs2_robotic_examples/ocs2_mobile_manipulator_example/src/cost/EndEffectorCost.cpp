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

#include <ocs2_mobile_manipulator_example/cost/EndEffectorCost.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorCost::EndEffectorCost(const ocs2::EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                 const ocs2::PenaltyFunctionBase& penalty)
    : endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      constraintPenalty_(3 * endEffectorKinematics.getIds().size(), std::unique_ptr<ocs2::PenaltyFunctionBase>(penalty.clone())) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorCost::EndEffectorCost(const EndEffectorCost& rhs)
    : endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()), constraintPenalty_(rhs.constraintPenalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t EndEffectorCost::cost(scalar_t time, const vector_t& state, const vector_t& input) {
  vector_t eeDesiredPosition;
  Eigen::Quaternion<scalar_t> eeDesiredOrientation;
  std::tie(eeDesiredPosition, eeDesiredOrientation) = interpolateReference(time);

  const vector_t eePosition = endEffectorKinematicsPtr_->getPositions(state)[0];
  return constraintPenalty_.getValue(eePosition - eeDesiredPosition);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t EndEffectorCost::finalCost(scalar_t time, const vector_t& state) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation EndEffectorCost::costQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input) {
  vector_t eeDesiredPosition;
  Eigen::Quaternion<scalar_t> eeDesiredOrientation;
  std::tie(eeDesiredPosition, eeDesiredOrientation) = interpolateReference(time);

  auto eePosLin = endEffectorKinematicsPtr_->getPositionsLinearApproximation(state)[0];
  eePosLin.f -= eeDesiredPosition;

  return constraintPenalty_.getQuadraticApproximation(eePosLin);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation EndEffectorCost::finalCostQuadraticApproximation(scalar_t time, const vector_t& state) {
  return ScalarFunctionQuadraticApproximation::Zero(state.rows(), 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, Eigen::Quaternion<scalar_t>> EndEffectorCost::interpolateReference(scalar_t time) const {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorCost] costDesiredTrajectoriesPtr_ is not set.");
  }

  std::pair<vector_t, Eigen::Quaternion<scalar_t>> reference;

  const auto& desiredTimeTrajectory = costDesiredTrajectoriesPtr_->desiredTimeTrajectory();
  const auto& desiredStateTrajectory = costDesiredTrajectoriesPtr_->desiredStateTrajectory();

  auto it = std::lower_bound(desiredTimeTrajectory.begin(), desiredTimeTrajectory.end(), time);
  int timeAIdx = it - desiredTimeTrajectory.begin() - 1;
  if (timeAIdx == -1) {
    reference.first = desiredStateTrajectory[0].head<3>();
    reference.second.coeffs() = desiredStateTrajectory[0].tail<4>();
  } else if (timeAIdx == desiredTimeTrajectory.size() - 1) {
    reference.first = desiredStateTrajectory[timeAIdx].head<3>();
    reference.second.coeffs() = desiredStateTrajectory[timeAIdx].tail<4>();
  } else {
    // interpolation
    const scalar_t tau = (time - desiredTimeTrajectory[timeAIdx]) / (desiredTimeTrajectory[timeAIdx + 1] - desiredTimeTrajectory[timeAIdx]);
    const Eigen::Quaternion<scalar_t> quatA(desiredStateTrajectory[timeAIdx].tail<4>());
    const Eigen::Quaternion<scalar_t> quatB(desiredStateTrajectory[timeAIdx + 1].tail<4>());

    reference.first = (1 - tau) * desiredStateTrajectory[timeAIdx].head<3>() + tau * desiredStateTrajectory[timeAIdx + 1].head<3>();
    reference.second = quatA.slerp(tau, quatB);
  }

  return reference;
}

}  // namespace mobile_manipulator
