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

#include <ocs2_legged_robot_example/cost/LeggedRobotStateInputQuadraticCost.h>

namespace ocs2 {
namespace legged_robot {

LeggedRobotStateInputQuadraticCost::LeggedRobotStateInputQuadraticCost(
    std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr, std::string taskFile, matrix_t Q, matrix_t R)
    : BASE(std::move(Q), std::move(R)), modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), taskFile_(std::move(taskFile)) {}

LeggedRobotStateInputQuadraticCost::LeggedRobotStateInputQuadraticCost(const LeggedRobotStateInputQuadraticCost& rhs)
    : BASE(rhs), modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_), taskFile_(rhs.taskFile_) {}

LeggedRobotStateInputQuadraticCost* LeggedRobotStateInputQuadraticCost::clone() const {
  return new LeggedRobotStateInputQuadraticCost(*this);
}

std::pair<vector_t, vector_t> LeggedRobotStateInputQuadraticCost::getStateInputDeviation(
    scalar_t time, const vector_t& state, const vector_t& input, const CostDesiredTrajectories& desiredTrajectory) const {
  // Get stance configuration
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(time);

  const vector_t xNominal = desiredTrajectory.getDesiredState(time).tail(STATE_DIM_ + 1).head(STATE_DIM_);
  vector_t uNominal = desiredTrajectory.getDesiredInput(time).head(INPUT_DIM_);

  // Distribute total mass equally over active stance legs.
  const scalar_t totalWeight = ROBOT_TOTAL_MASS_ * 9.81;
  size_t numStanceLegs(0);

  for (size_t i = 0; i < FOOT_CONTACTS_NUM_; i++) {
    if (contactFlags[i]) {
      ++numStanceLegs;
    }
  }

  if (numStanceLegs > 0) {
    for (size_t i = 0; i < FOOT_CONTACTS_NUM_; i++) {
      if (contactFlags[i]) {
        uNominal(3 * i + 2) = totalWeight / numStanceLegs;
      }
    }
  }

  return {state - xNominal, input - uNominal};
}

}  // namespace legged_robot
}  // namespace ocs2
