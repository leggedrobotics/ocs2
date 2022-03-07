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
MRT_BASE::MRT_BASE() {
  reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::reset() {
  std::lock_guard<std::mutex> lock(bufferMutex_);

  policyReceivedEver_ = false;
  newPolicyInBuffer_ = false;
  mrtTrylockWarningCount_ = 0;

  activeCommandPtr_.reset();
  bufferCommandPtr_.reset();
  activePrimalSolutionPtr_.reset();
  bufferPrimalSolutionPtr_.reset();
  activePerformanceIndicesPtr_.reset();
  bufferPerformanceIndicesPtr_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const CommandData& MRT_BASE::getCommand() const {
  if (activeCommandPtr_ != nullptr) {
    return *activeCommandPtr_;
  } else {
    throw std::runtime_error("[MRT_BASE::getCommand] updatePolicy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PrimalSolution& MRT_BASE::getPolicy() const {
  if (activePrimalSolutionPtr_ != nullptr) {
    return *activePrimalSolutionPtr_;
  } else {
    throw std::runtime_error("[MRT_BASE::getPolicy] updatePolicy() should be called first!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PerformanceIndex& MRT_BASE::getPerformanceIndices() const {
  if (activePerformanceIndicesPtr_ != nullptr) {
    return *activePerformanceIndicesPtr_;
  } else {
    throw std::runtime_error("[MRT_BASE::getPerformanceIndices] updatePolicy() should be called first!");
  }
}

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
  if (activePrimalSolutionPtr_ == nullptr) {
    throw std::runtime_error("[MRT_BASE::evaluatePolicy] updatePolicy() should be called first!");
  }

  if (currentTime > activePrimalSolutionPtr_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: " << std::to_string(currentTime) << ">"
              << std::to_string(activePrimalSolutionPtr_->timeTrajectory_.back()) << "\n";
  }

  mpcInput = activePrimalSolutionPtr_->controllerPtr_->computeInput(currentTime, currentState);
  mpcState =
      LinearInterpolation::interpolate(currentTime, activePrimalSolutionPtr_->timeTrajectory_, activePrimalSolutionPtr_->stateTrajectory_);

  mode = activePrimalSolutionPtr_->modeSchedule_.modeAtTime(currentTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::rolloutPolicy(scalar_t currentTime, const vector_t& currentState, const scalar_t& timeStep, vector_t& mpcState,
                             vector_t& mpcInput, size_t& mode) {
  if (rolloutPtr_ == nullptr) {
    throw std::runtime_error("[MRT_BASE::rolloutPolicy] rollout class is not set! Use initRollout() to initialize it!");
  }

  if (activePrimalSolutionPtr_ == nullptr) {
    throw std::runtime_error("[MRT_BASE::rolloutPolicy] updatePolicy() should be called first!");
  }

  if (currentTime > activePrimalSolutionPtr_->timeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: " << std::to_string(currentTime) << ">"
              << std::to_string(activePrimalSolutionPtr_->timeTrajectory_.back()) << "\n";
  }

  // perform a rollout
  scalar_array_t timeTrajectory;
  size_array_t postEventIndicesStock;
  vector_array_t stateTrajectory, inputTrajectory;
  const scalar_t finalTime = currentTime + timeStep;
  rolloutPtr_->run(currentTime, currentState, finalTime, activePrimalSolutionPtr_->controllerPtr_.get(),
                   activePrimalSolutionPtr_->modeSchedule_, timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

  mpcState = stateTrajectory.back();
  mpcInput = inputTrajectory.back();

  mode = activePrimalSolutionPtr_->modeSchedule_.modeAtTime(finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MRT_BASE::updatePolicy() {
  std::unique_lock<std::mutex> lock(bufferMutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    mrtTrylockWarningCount_ = 0;
    if (newPolicyInBuffer_) {
      // update the active solution from buffer
      activeCommandPtr_.swap(bufferCommandPtr_);
      activePrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
      activePerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
      newPolicyInBuffer_ = false;  // make sure we don't swap in the old policy again

      modifyActiveSolution(*activeCommandPtr_, *activePrimalSolutionPtr_);
      return true;
    } else {
      return false;  // No policy update: the buffer contains nothing new.
    }
  } else {
    ++mrtTrylockWarningCount_;
    if (mrtTrylockWarningCount_ > mrtTrylockWarningThreshold_) {
      std::cerr << "[MRT_BASE::updatePolicy] failed to lock the policyBufferMutex for " << mrtTrylockWarningCount_
                << " consecutive times.\n";
    }
    return false;  // No policy update: the lock could not be acquired.
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::moveToBuffer(std::unique_ptr<CommandData> commandDataPtr, std::unique_ptr<PrimalSolution> primalSolutionPtr,
                            std::unique_ptr<PerformanceIndex> performanceIndicesPtr) {
  if (commandDataPtr == nullptr) {
    throw std::runtime_error("[MRT_BASE::moveToBuffer] commandDataPtr cannot be a null pointer!");
  }

  if (primalSolutionPtr == nullptr) {
    throw std::runtime_error("[MRT_BASE::moveToBuffer] primalSolutionPtr cannot be a null pointer!");
  }

  if (performanceIndicesPtr == nullptr) {
    throw std::runtime_error("[MRT_BASE::moveToBuffer] performanceIndicesPtr cannot be a null pointer!");
  }

  std::lock_guard<std::mutex> lk(bufferMutex_);
  // use swap such that the old objects are destroyed after releasing the lock.
  bufferCommandPtr_.swap(commandDataPtr);
  bufferPrimalSolutionPtr_.swap(primalSolutionPtr);
  bufferPerformanceIndicesPtr_.swap(performanceIndicesPtr);

  // allow user to modify the buffer
  modifyBufferedSolution(*bufferCommandPtr_, *bufferPrimalSolutionPtr_);

  newPolicyInBuffer_ = true;
  policyReceivedEver_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::modifyActiveSolution(const CommandData& command, PrimalSolution& primalSolution) {
  for (auto& mrtObserver : observerPtrArray_) {
    if (mrtObserver != nullptr) {
      mrtObserver->modifyActiveSolution(command, primalSolution);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_BASE::modifyBufferedSolution(const CommandData& commandBuffer, PrimalSolution& primalSolutionBuffer) {
  for (auto& mrtObserver : observerPtrArray_) {
    if (mrtObserver != nullptr) {
      mrtObserver->modifyBufferedSolution(commandBuffer, primalSolutionBuffer);
    }
  }
}

}  // namespace ocs2
