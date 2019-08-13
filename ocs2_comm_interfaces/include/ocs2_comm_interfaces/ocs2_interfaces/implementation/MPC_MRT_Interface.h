
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::MPC_MRT_Interface(mpc_t& mpc, std::shared_ptr<HybridLogicRules> logicRules)
    : Base(std::move(logicRules)), mpc_(mpc), numMpcIterations_(0) {}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  numMpcIterations_ = 0;
  mpc_.reset();
  setTargetTrajectories(initCostDesiredTrajectories);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories) {
  if (mpc_.settings().debugPrint_) {
    std::cerr << "### The target position is updated to" << std::endl;
    targetTrajectories.display();
  }
  mpc_.getSolverPtr()->setCostDesiredTrajectories(targetTrajectories);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setCurrentObservation(const system_observation_t& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setModeSequence(const mode_sequence_template_t& modeSequenceTemplate) {
  mpc_.setNewLogicRulesTemplate(modeSequenceTemplate);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::advanceMpc() {
  std::chrono::time_point<std::chrono::steady_clock> startTime, finishTime;
  if (mpc_.settings().debugPrint_) {
    startTime = std::chrono::steady_clock::now();
  }

  system_observation_t mpcInitObservation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    mpcInitObservation = currentObservation_;
  }

  mpc_.run(mpcInitObservation.time(), mpcInitObservation.state());
  fillMpcOutputBuffers(std::move(mpcInitObservation));

  // Incrementing numIterations must happen after fillMpcOutputBuffers
  numMpcIterations_++;

  // measure the delay
  if (mpc_.settings().debugPrint_) {
    finishTime = std::chrono::steady_clock::now();
    auto currentDelay = std::chrono::duration<scalar_t, std::milli>(finishTime - startTime).count();
    meanDelay_ += (currentDelay - meanDelay_) / numMpcIterations_;
    maxDelay_ = std::max(maxDelay_, currentDelay);

    std::cerr << std::endl;
    std::cerr << "### Average duration of MPC optimization is: " << meanDelay_ << " [ms]." << std::endl;
    std::cerr << "### Maximum duration of MPC optimization is: " << maxDelay_ << " [ms]." << std::endl;
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::fillMpcOutputBuffers(system_observation_t mpcInitObservation) {
  // obtain pointers to MPC-internal variables
  const std::vector<scalar_array_t>* timeTrajectoriesPtr;
  const state_vector_array2_t* stateTrajectoriesPtr;
  const input_vector_array2_t* inputTrajectoriesPtr;
  mpc_.getOptimizedTrajectoriesPtr(timeTrajectoriesPtr, stateTrajectoriesPtr, inputTrajectoriesPtr);

  // update buffers, i.e., copy MPC results into our buffers
  std::lock_guard<std::mutex> policyBufferLock(this->policyBufferMutex_);
  auto& timeBuffer = this->policyBuffer_->mpcTimeTrajectory_;
  auto& stateBuffer = this->policyBuffer_->mpcStateTrajectory_;
  auto& controlBuffer = this->policyBuffer_->mpcController_;
  auto& eventBuffer = this->policyBuffer_->eventTimes_;
  auto& subsystemBuffer = this->policyBuffer_->subsystemsSequence_;
  auto& initObservationBuffer = this->commandBuffer_->mpcInitObservation_;
  auto& costDesiredBuffer = this->commandBuffer_->mpcCostDesiredTrajectories_;
  auto& inputBuffer = mpcInputTrajectoryBuffer_;

  this->policyUpdatedBuffer_ = true;

  initObservationBuffer = std::move(mpcInitObservation);

  int N = 0;
  for (int i = 0; i < timeTrajectoriesPtr->size(); i++) {
    N += (*timeTrajectoriesPtr)[i].size();
  }

  timeBuffer.clear();
  timeBuffer.reserve(N);
  stateBuffer.clear();
  stateBuffer.reserve(N);
  inputBuffer.clear();
  inputBuffer.reserve(N);
  for (int i = 0; i < timeTrajectoriesPtr->size(); i++) {
    timeBuffer.insert(std::end(timeBuffer), std::begin((*timeTrajectoriesPtr)[i]), std::end((*timeTrajectoriesPtr)[i]));
    stateBuffer.insert(std::end(stateBuffer), std::begin((*stateTrajectoriesPtr)[i]), std::end((*stateTrajectoriesPtr)[i]));
    inputBuffer.insert(std::end(inputBuffer), std::begin((*inputTrajectoriesPtr)[i]), std::end((*inputTrajectoriesPtr)[i]));
  }

  auto controllerPtrs = mpc_.getOptimizedControllerPtr();
  if (mpc_.settings().useFeedbackPolicy_) {
    // concatenate controller stock into a single controller
    controlBuffer.reset();
    for (auto controllerPtr : *controllerPtrs) {
      if (controllerPtr->empty()) {
        continue;  // some time partitions may be unused
      }

      if (controlBuffer) {
        controlBuffer->concatenate(controllerPtr);
      } else {
        controlBuffer.reset(controllerPtr->clone());
      }
    }
  } else {
    controlBuffer.reset(new FeedforwardController<STATE_DIM, INPUT_DIM>(timeBuffer, inputBuffer));
  }

  eventBuffer = mpc_.getLogicRulesPtr()->eventTimes();
  subsystemBuffer = mpc_.getLogicRulesPtr()->subsystemsSequence();
  costDesiredBuffer = mpc_.getSolverPtr()->getCostDesiredTrajectories();
  this->partitioningTimesUpdate(initObservationBuffer.time(), this->partitioningTimesBuffer_);

  // allow user to modify the buffer
  this->modifyBufferPolicy(*this->commandBuffer_, *this->policyBuffer_);

  // Flags to be set last:
  this->newPolicyInBuffer_ = true;
  this->policyReceivedEver_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getLinearFeedbackGain(scalar_t time, input_state_matrix_t& K) {
  if (!mpc_.settings().useFeedbackPolicy_) {
    throw std::runtime_error("Feedback gains only available with useFeedbackPolicy setting");
  }
  auto controller = dynamic_cast<LinearController<STATE_DIM, INPUT_DIM>*>(this->currentPolicy_->mpcController_.get());
  if (!controller) {
    throw std::runtime_error("Feedback gains only available with linear controller");
  }
  controller->getFeedbackGain(time, K);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state,
                                                                              state_vector_t& Vx) {
  mpc_.getSolverPtr()->getValueFunctionStateDerivative(time, state, Vx);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
bool MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::updatePolicyImpl() {
  if (Base::updatePolicyImpl()) {
    // additionally update variables only present in this child class
    mpcInputTrajectory_.swap(mpcInputTrajectoryBuffer_);
    mpcLinInterpolateInput_.setData(&this->currentPolicy_->mpcTimeTrajectory_, &mpcInputTrajectory_);
    return true;
  }
  return false;
}

}  // namespace ocs2
