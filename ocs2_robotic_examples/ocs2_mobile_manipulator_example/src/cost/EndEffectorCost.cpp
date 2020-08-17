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

namespace mobile_manipulator {

EndEffectorCost::EndEffectorCost(const PinocchioInterface<ad_scalar_t>& pinocchioInterface, matrix_t Q, matrix_t R, matrix_t Qf)
    : ocs2::QuadraticGaussNewtonCostBaseAD(STATE_DIM, INPUT_DIM), Q_(std::move(Q)), R_(std::move(R)), Qf_(std::move(Qf)) {
  pinocchioInterface_.reset(new PinocchioInterface<ad_scalar_t>(pinocchioInterface));
}

ad_vector_t EndEffectorCost::intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                      const ad_vector_t& parameters) const {
  ad_vector_t cost(INPUT_DIM + 3);
  const ad_vector_t eePosDesired = parameters.tail(3);
  const auto eePosition = pinocchioInterface_->getBodyPoseInWorldFrame("WRIST_2", state).position;
  const ad_vector_t err = eePosition - eePosDesired;
  cost.head(INPUT_DIM) = R_.array().sqrt().matrix() * input;
  cost.tail(3) = Q_.array().sqrt().matrix() * err;
  return cost;
}

ad_vector_t EndEffectorCost::finalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  ad_vector_t cost(3);
  ad_vector_t eePosDesired = parameters.tail(3);
  const auto eePosition = pinocchioInterface_->getBodyPoseInWorldFrame("WRIST_2", state).position;
  const ad_vector_t err = eePosition - eePosDesired;
  cost.tail(3) = Qf_.array().sqrt().matrix() * err;
  return cost;
}

size_t EndEffectorCost::getNumIntermediateParameters() const {
  return 6;
}

vector_t EndEffectorCost::getIntermediateParameters(scalar_t time) const {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorCost] costDesiredTrajectoriesPtr_ is not set.");
  }
  return this->costDesiredTrajectoriesPtr_->getDesiredState(time);
}

size_t EndEffectorCost::getNumFinalParameters() const {
  return 6;
}

vector_t EndEffectorCost::getFinalParameters(scalar_t time) const {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorCost] costDesiredTrajectoriesPtr_ is not set.");
  }
  return this->costDesiredTrajectoriesPtr_->getDesiredState(time);
}

}  // namespace mobile_manipulator
