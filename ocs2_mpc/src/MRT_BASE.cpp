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

#include "ocs2_mpc/MRT_BASE.h"

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_BASE::MRT_BASE()
    : currentPrimalSolution_(new PrimalSolution),
      primalSolutionBuffer_(new PrimalSolution),
      currentCommand_(new CommandData),
      commandBuffer_(new CommandData) {
  reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::reset() {
  std::lock_guard<std::mutex> lock(policyBufferMutex_);

  policyReceivedEver_ = false;
  newPolicyInBuffer_ = false;

  policyUpdated_ = false;
  policyUpdatedBuffer_ = false;

  partitioningTimesUpdate(0.0, partitioningTimes_);
  partitioningTimesUpdate(0.0, partitioningTimesBuffer_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PrimalSolution& MRT_BASE::getPolicy() const {
  if (policyUpdated_) {
    return *currentPrimalSolution_;
  } else {
    throw std::runtime_error("[MRT_BASE::getPolicy] The policy was never received or not updated");
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::initRollout(const RolloutBase* rolloutPtr) {
  rolloutPtr_.reset(rolloutPtr->clone());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::evaluatePolicy(scalar_t currentTime, const vector_t& currentState, vector_t& mpcState, vector_t& mpcInput, size_t& mode) {
  if (currentTime > currentPrimalSolution_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: " << std::to_string(currentTime) << ">"
              << std::to_string(currentPrimalSolution_->timeTrajectory_.back()) << "\n";
  }

  mpcInput = currentPrimalSolution_->controllerPtr_->computeInput(currentTime, currentState);
  LinearInterpolation::interpolate(currentTime, mpcState, &currentPrimalSolution_->timeTrajectory_,
                                   &currentPrimalSolution_->stateTrajectory_);

  mode = currentPrimalSolution_->modeSchedule_.modeAtTime(currentTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::rolloutPolicy(scalar_t currentTime, const vector_t& currentState, const scalar_t& timeStep, vector_t& mpcState,
                             vector_t& mpcInput, size_t& mode) {
  if (currentTime > currentPrimalSolution_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: " << std::to_string(currentTime) << ">"
              << std::to_string(currentPrimalSolution_->timeTrajectory_.back()) << "\n";
  }

  if (!rolloutPtr_) {
    throw std::runtime_error("MRT_ROS_interface: rolloutPtr is not initialized, call initRollout first.");
  }

  const size_t activePartitionIndex = 0;  // there is only one partition.
  scalar_t finalTime = currentTime + timeStep;
  scalar_array_t timeTrajectory;
  size_array_t postEventIndicesStock;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;

  // perform a rollout
  if (policyUpdated_) {
    rolloutPtr_->run(currentTime, currentState, finalTime, currentPrimalSolution_->controllerPtr_.get(),
                     currentPrimalSolution_->modeSchedule_.eventTimes, timeTrajectory, postEventIndicesStock, stateTrajectory,
                     inputTrajectory);
  } else {
    throw std::runtime_error("MRT_ROS_interface: policy should be updated before rollout.");
  }

  mpcState = stateTrajectory.back();
  mpcInput = inputTrajectory.back();

  mode = currentPrimalSolution_->modeSchedule_.modeAtTime(finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MRT_BASE::updatePolicy() {
  std::lock_guard<std::mutex> lock(policyBufferMutex_);

  if (!policyUpdatedBuffer_ || !newPolicyInBuffer_) {
    return false;
  }
  newPolicyInBuffer_ = false;  // make sure we don't swap in the old policy again

  // update the current policy from buffer
  policyUpdated_ = policyUpdatedBuffer_;
  currentCommand_.swap(commandBuffer_);
  currentPrimalSolution_.swap(primalSolutionBuffer_);
  partitioningTimes_.swap(partitioningTimesBuffer_);

  modifyPolicy(*currentCommand_, *currentPrimalSolution_);

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::partitioningTimesUpdate(scalar_t time, scalar_array_t& partitioningTimes) const {
  partitioningTimes.resize(2);
  partitioningTimes[0] = (policyReceivedEver_) ? initPlanObservation_.time : time;
  partitioningTimes[1] = std::numeric_limits<scalar_t>::max();
}

}  // namespace ocs2
