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

#include <ocs2_comm_interfaces/ocs2_interfaces/MRT_BASE.h>

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MRT_BASE<STATE_DIM, INPUT_DIM>::MRT_BASE(std::shared_ptr<HybridLogicRules> logicRules)
    : mpcLinInterpolateState_(&mpcTimeTrajectory_, &mpcStateTrajectory_) {
  reset();
  if (!logicRules) {
    logicRules.reset(new NullLogicRules());
  }
  logicMachinePtr_.reset(new HybridLogicRulesMachine(std::move(logicRules)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_BASE<STATE_DIM, INPUT_DIM>::reset() {
  std::lock_guard<std::mutex> lock(policyBufferMutex_);

  policyReceivedEver_ = false;
  newPolicyInBuffer_ = false;

  policyUpdated_ = false;
  policyUpdatedBuffer_ = false;

  mpcController_.reset();
  mpcLinInterpolateState_.setZero();

  eventTimes_.clear();
  eventTimesBuffer_.clear();
  subsystemsSequence_.clear();
  subsystemsSequenceBuffer_.clear();
  partitioningTimesUpdate(0.0, partitioningTimes_);
  partitioningTimesUpdate(0.0, partitioningTimesBuffer_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const CostDesiredTrajectories<typename MRT_BASE<STATE_DIM, INPUT_DIM>::scalar_t>&
MRT_BASE<STATE_DIM, INPUT_DIM>::mpcCostDesiredTrajectories() const {
  return mpcCostDesiredTrajectories_;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_BASE<STATE_DIM, INPUT_DIM>::initRollout(const ControlledSystemBase<STATE_DIM, INPUT_DIM>& controlSystemBase,
                                                 const Rollout_Settings& rolloutSettings) {
  rolloutPtr_.reset(new TimeTriggeredRollout<STATE_DIM, INPUT_DIM>(controlSystemBase, rolloutSettings, "mrt"));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_BASE<STATE_DIM, INPUT_DIM>::evaluatePolicy(scalar_t currentTime, const state_vector_t& currentState, state_vector_t& mpcState,
                                                    input_vector_t& mpcInput, size_t& subsystem) {
  if (currentTime > mpcTimeTrajectory_.back()) {
    std::cerr << "The requested currentTime is greater than the received plan: " + std::to_string(currentTime) + ">" +
                    std::to_string(mpcTimeTrajectory_.back()) << std::endl;
  }

  mpcInput = mpcController_->computeInput(currentTime, currentState);
  mpcLinInterpolateState_.interpolate(currentTime, mpcState);

  size_t index = findActiveSubsystemFnc_(currentTime);
  subsystem = logicMachinePtr_->getLogicRulesPtr()->subsystemsSequence().at(index);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_BASE<STATE_DIM, INPUT_DIM>::rolloutPolicy(scalar_t currentTime, const state_vector_t& currentState, const scalar_t& timeStep,
                                                   state_vector_t& mpcState, input_vector_t& mpcInput, size_t& subsystem) {
  if (currentTime > mpcTimeTrajectory_.back()) {
	  std::cerr << "The requested currentTime is greater than the received plan: " + std::to_string(currentTime) + ">" +
                    std::to_string(mpcTimeTrajectory_.back()) << std::endl;
  }

  if (!rolloutPtr_) {
    throw std::runtime_error("MRT_ROS_interface: rolloutPtr is not initialized, call initRollout first.");
  }

  const size_t activePartitionIndex = 0;  // there is only one partition.
  scalar_t finalTime = currentTime + timeStep;
  scalar_array_t timeTrajectory;
  size_array_t eventsPastTheEndIndeces;
  state_vector_array_t stateTrajectory;
  input_vector_array_t inputTrajectory;

  // perform a rollout
  if (policyUpdated_ == true) {
    rolloutPtr_->run(activePartitionIndex, currentTime, currentState, finalTime, mpcController_.get(), *logicMachinePtr_, timeTrajectory,
                     eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);
  } else {
    throw std::runtime_error("MRT_ROS_interface: policy should be updated before rollout.");
  }

  mpcState = stateTrajectory.back();
  mpcInput = inputTrajectory.back();

  size_t index = findActiveSubsystemFnc_(finalTime);
  subsystem = logicMachinePtr_->getLogicRulesPtr()->subsystemsSequence().at(index);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool MRT_BASE<STATE_DIM, INPUT_DIM>::updatePolicy() {
  std::lock_guard<std::mutex> lock(policyBufferMutex_);

  if (!policyUpdatedBuffer_ or !newPolicyInBuffer_) {
    return false;
  }

  newPolicyInBuffer_ = false;  // make sure we don't swap in the old policy again

  // update the current policy from buffer
  policyUpdated_ = policyUpdatedBuffer_;
  mpcInitObservation_.swap(mpcInitObservationBuffer_);
  mpcCostDesiredTrajectories_.swap(mpcCostDesiredTrajectoriesBuffer_);
  mpcTimeTrajectory_.swap(mpcTimeTrajectoryBuffer_);
  mpcStateTrajectory_.swap(mpcStateTrajectoryBuffer_);
  mpcLinInterpolateState_.setData(&mpcTimeTrajectory_, &mpcStateTrajectory_);
  mpcController_.swap(mpcControllerBuffer_);

  // check whether logic rules needs to be updated
  bool logicUpdated = false;
  if (subsystemsSequence_ != subsystemsSequenceBuffer_) {
    subsystemsSequence_.swap(subsystemsSequenceBuffer_);
    logicUpdated = true;
  }
  if (eventTimes_ != eventTimesBuffer_) {
    eventTimes_.swap(eventTimesBuffer_);
    logicUpdated = true;
  }
  if (partitioningTimes_ != partitioningTimesBuffer_) {
    partitioningTimes_.swap(partitioningTimesBuffer_);
    logicUpdated = true;
  }

  // update logic rules
  if (logicUpdated) {
    // set mode sequence
    logicMachinePtr_->getLogicRulesPtr()->setModeSequence(subsystemsSequence_, eventTimes_);
    // Tell logicMachine that logicRules are modified
    logicMachinePtr_->logicRulesUpdated();
    // update logicMachine
    logicMachinePtr_->updateLogicRules(partitioningTimes_);

    // function for finding active subsystem
    const size_t partitionIndex = 0;  // we assume only one partition
    findActiveSubsystemFnc_ = std::move(logicMachinePtr_->getHandleToFindActiveEventCounter(partitionIndex));
  }

  modifyPolicy(logicUpdated, *mpcController_, mpcTimeTrajectory_, mpcStateTrajectory_, eventTimes_, subsystemsSequence_);

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_BASE<STATE_DIM, INPUT_DIM>::partitioningTimesUpdate(scalar_t time, scalar_array_t& partitioningTimes) const {
  partitioningTimes.resize(2);
  partitioningTimes[0] = (policyReceivedEver_) ? initPlanObservation_.time() : time;
  partitioningTimes[1] = std::numeric_limits<scalar_t>::max();
}

}  // namespace ocs2
