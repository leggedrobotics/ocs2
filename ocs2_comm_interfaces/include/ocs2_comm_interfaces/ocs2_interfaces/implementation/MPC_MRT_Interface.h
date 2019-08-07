
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>
#include <ocs2_core/control/FeedforwardController.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::MPC_MRT_Interface(mpc_t* mpc, std::shared_ptr<HybridLogicRules> logicRules)
    : Base(std::move(logicRules)), mpcPtr_(std::move(mpc)), numMpcIterations_(0) {}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  numMpcIterations_ = 0;
  mpcPtr_->reset();
  setTargetTrajectories(initCostDesiredTrajectories);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories) {
  if (mpcPtr_->settings().debugPrint_) {
    std::cerr << "### The target position is updated to" << std::endl;
    targetTrajectories.display();
  }
  mpcPtr_->getSolverPtr()->setCostDesiredTrajectories(targetTrajectories);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setCurrentObservation(const system_observation_t& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setModeSequence(const mode_sequence_template_t& modeSequenceTemplate) {
  mpcPtr_->setNewLogicRulesTemplate(modeSequenceTemplate);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::advanceMpc() {
  std::chrono::time_point<std::chrono::steady_clock> startTime, finishTime;
  if (mpcPtr_->settings().debugPrint_) {
    startTime = std::chrono::steady_clock::now();
  }

  system_observation_t mpcInitObservation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    mpcInitObservation = currentObservation_;
  }

  mpcPtr_->run(mpcInitObservation.time(), mpcInitObservation.state());
  fillMpcOutputBuffers(std::move(mpcInitObservation));

  // Incrementing numIterations must happen after fillMpcOutputBuffers
  numMpcIterations_++;

  // measure the delay
  if (mpcPtr_->settings().debugPrint_) {
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
  const controller_ptr_array_t* controllerPtrs(nullptr);
  const std::vector<scalar_array_t>* timeTrajectoriesPtr(nullptr);
  const state_vector_array2_t* stateTrajectoriesPtr(nullptr);
  const input_vector_array2_t* inputTrajectoriesPtr(nullptr);
  const cost_desired_trajectories_t* solverCostDesiredTrajectoriesPtr(nullptr);
  const scalar_array_t* eventTimesPtr(nullptr);
  const size_array_t* subsystemsSequencePtr(nullptr);

  // obtain pointers to MPC-internal variables
  controllerPtrs = mpcPtr_->getOptimizedControllerPtr();
  mpcPtr_->getOptimizedTrajectoriesPtr(timeTrajectoriesPtr, stateTrajectoriesPtr, inputTrajectoriesPtr);
  mpcPtr_->getCostDesiredTrajectoriesPtr(solverCostDesiredTrajectoriesPtr);
  eventTimesPtr = &mpcPtr_->getLogicRulesPtr()->eventTimes();
  subsystemsSequencePtr = &mpcPtr_->getLogicRulesPtr()->subsystemsSequence();

  // update buffers, i.e., copy MPC results into our buffers
  std::lock_guard<std::mutex> policyBufferLock(this->policyBufferMutex_);

  this->policyUpdatedBuffer_ = true;

  this->mpcInitObservationBuffer_ = std::move(mpcInitObservation);

  int N = 0;
  for (int i = 0; i < timeTrajectoriesPtr->size(); i++) {
    N += (*timeTrajectoriesPtr)[i].size();
  }
  this->mpcTimeTrajectoryBuffer_.clear();
  this->mpcTimeTrajectoryBuffer_.reserve(N);
  this->mpcStateTrajectoryBuffer_.clear();
  this->mpcStateTrajectoryBuffer_.reserve(N);
  mpcInputTrajectoryBuffer_.clear();
  mpcInputTrajectoryBuffer_.reserve(N);
  for (int i = 0; i < timeTrajectoriesPtr->size(); i++) {
    this->mpcTimeTrajectoryBuffer_.insert(std::end(this->mpcTimeTrajectoryBuffer_), std::begin((*timeTrajectoriesPtr)[i]),
                                          std::end((*timeTrajectoriesPtr)[i]));
    this->mpcStateTrajectoryBuffer_.insert(std::end(this->mpcStateTrajectoryBuffer_), std::begin((*stateTrajectoriesPtr)[i]),
                                           std::end((*stateTrajectoriesPtr)[i]));
    mpcInputTrajectoryBuffer_.insert(std::end(mpcInputTrajectoryBuffer_), std::begin((*inputTrajectoriesPtr)[i]),
                                     std::end((*inputTrajectoriesPtr)[i]));
  }

  if (mpcPtr_->settings().useFeedbackPolicy_) {
    // concatenate controller stock into a single controller
    this->mpcControllerBuffer_.reset();
    for (auto controllerPtr : *controllerPtrs) {
      if (controllerPtr->empty()) {
        continue;  // some time partitions may be unused
      }

      if (this->mpcControllerBuffer_) {
        this->mpcControllerBuffer_->concatenate(controllerPtr);
      } else {
        this->mpcControllerBuffer_.reset(controllerPtr->clone());
      }
    }
  } else {
    this->mpcControllerBuffer_.reset(
        new FeedforwardController<STATE_DIM, INPUT_DIM>(this->mpcTimeTrajectoryBuffer_, mpcInputTrajectoryBuffer_));
  }

  this->eventTimesBuffer_ = *eventTimesPtr;
  this->partitioningTimesUpdate(this->mpcInitObservationBuffer_.time(), this->partitioningTimesBuffer_);
  this->subsystemsSequenceBuffer_ = *subsystemsSequencePtr;
  this->mpcCostDesiredTrajectoriesBuffer_ = *solverCostDesiredTrajectoriesPtr;

  // allow user to modify the buffer
  this->modifyBufferPolicy(this->mpcInitObservationBuffer_, *this->mpcControllerBuffer_, this->mpcTimeTrajectoryBuffer_,
                           this->mpcStateTrajectoryBuffer_, this->eventTimesBuffer_, this->subsystemsSequenceBuffer_);

  // Flags to be set last:
  this->newPolicyInBuffer_ = true;
  this->policyReceivedEver_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
bool MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::updatePolicy() {
  if (Base::updatePolicy()) {
    // additionally update variables only present in this child class
    std::lock_guard<std::mutex> lock(this->policyBufferMutex_);
    mpcInputTrajectory_.swap(mpcInputTrajectoryBuffer_);
    mpcLinInterpolateInput_.setData(&this->mpcTimeTrajectory_, &mpcInputTrajectory_);
    return true;
  }
  return false;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::state_vector_array_t&
MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getMpcStateTrajectory() const {
  return this->mpcStateTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::input_vector_array_t&
MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getMpcInputTrajectory() const {
  return mpcInputTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::scalar_array_t& MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getMpcTimeTrajectory()
    const {
  return this->mpcTimeTrajectory_;
}

}  // namespace ocs2
