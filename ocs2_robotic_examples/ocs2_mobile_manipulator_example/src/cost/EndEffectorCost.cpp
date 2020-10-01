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

#include <cppad/cg/support/cppadcg_eigen.hpp>

#include <ocs2_mobile_manipulator_example/cost/EndEffectorCost.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace mobile_manipulator {

EndEffectorCost::EndEffectorCost(const ocs2::PinocchioInterfaceCppAd& pinocchioInterface, matrix_t Q, matrix_t R, matrix_t Qf,
                                 std::string endEffectorName)
    : ocs2::QuadraticGaussNewtonCostBaseAD(STATE_DIM, INPUT_DIM),
      Q_(std::move(Q)),
      R_(std::move(R)),
      Qf_(std::move(Qf)),
      endEffectorName_(std::move(endEffectorName)) {
  pinocchioInterface_.reset(new ocs2::PinocchioInterfaceCppAd(pinocchioInterface));
}

EndEffectorCost::EndEffectorCost(const EndEffectorCost& rhs)
    : ocs2::QuadraticGaussNewtonCostBaseAD(rhs), Q_(rhs.Q_), R_(rhs.R_), Qf_(rhs.Qf_), endEffectorName_(rhs.endEffectorName_) {
  pinocchioInterface_.reset(new ocs2::PinocchioInterfaceCppAd(*rhs.pinocchioInterface_));
}

auto EndEffectorCost::intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                               const ad_vector_t& parameters) const -> ad_vector_t {
  const ad_vector_t eeDesiredPosition(parameters.head<3>());
  Eigen::Quaternion<ad_scalar_t> eeDesiredOrientation;
  eeDesiredOrientation.coeffs() = parameters.tail(4);
  const auto eePose = pinocchioInterface_->getBodyPoseInWorldFrame(endEffectorName_, state);

  ad_vector_t error(6);
  error.head(3) = eePose.position - eeDesiredPosition;                                 // position error
  error.tail(3) = ocs2::quaternionDistance(eePose.orientation, eeDesiredOrientation);  // orientation error

  ad_vector_t cost(INPUT_DIM + 6);
  cost.head(INPUT_DIM) = R_.array().sqrt().matrix() * input;
  cost.tail(6) = Q_.array().sqrt().matrix() * error;
  return cost;
}

auto EndEffectorCost::finalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const -> ad_vector_t {
  const ad_vector_t eeDesiredPosition(parameters.head(3));
  Eigen::Quaternion<ad_scalar_t> eeDesiredOrientation;
  eeDesiredOrientation.coeffs() = parameters.tail(4);
  const auto eePose = pinocchioInterface_->getBodyPoseInWorldFrame(endEffectorName_, state);

  ad_vector_t error(6);
  error.head(3) = eePose.position - eeDesiredPosition;                                 // position error
  error.tail(3) = ocs2::quaternionDistance(eePose.orientation, eeDesiredOrientation);  // orientation error

  return Qf_.array().sqrt().matrix() * error;
}

/* Author: Johannes Pankert */
vector_t EndEffectorCost::interpolateReference(scalar_t time) const {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorCost] costDesiredTrajectoriesPtr_ is not set.");
  }

  vector_t reference(7);
  const auto& desiredTimeTrajectory = costDesiredTrajectoriesPtr_->desiredTimeTrajectory();
  const auto& desiredStateTrajectory = costDesiredTrajectoriesPtr_->desiredStateTrajectory();

  auto it = std::lower_bound(desiredTimeTrajectory.begin(), desiredTimeTrajectory.end(), time);
  int timeAIdx = it - desiredTimeTrajectory.begin() - 1;
  if (timeAIdx == -1) {
    reference = desiredStateTrajectory[0];

  } else if (timeAIdx == desiredTimeTrajectory.size() - 1) {
    reference = desiredStateTrajectory[timeAIdx];

  } else {
    // interpolation
    scalar_t tau = (time - desiredTimeTrajectory[timeAIdx]) / (desiredTimeTrajectory[timeAIdx + 1] - desiredTimeTrajectory[timeAIdx]);
    const Eigen::Quaterniond quatA(desiredStateTrajectory[timeAIdx].tail<4>());
    const Eigen::Quaterniond quatB(desiredStateTrajectory[timeAIdx + 1].tail<4>());

    reference.tail<4>() = quatA.slerp(tau, quatB).coeffs();
    reference.head<3>() = (1 - tau) * desiredStateTrajectory[timeAIdx].head<3>() + tau * desiredStateTrajectory[timeAIdx + 1].head<3>();
  }

  return reference;
}

size_t EndEffectorCost::getNumIntermediateParameters() const {
  return 7;
}

vector_t EndEffectorCost::getIntermediateParameters(scalar_t time) const {
  return interpolateReference(time);
}

size_t EndEffectorCost::getNumFinalParameters() const {
  return 7;
}

vector_t EndEffectorCost::getFinalParameters(scalar_t time) const {
  return interpolateReference(time);
}

}  // namespace mobile_manipulator
