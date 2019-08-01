
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_Interface<STATE_DIM, INPUT_DIM>::MPC_Interface(mpc_t& mpc, std::shared_ptr<HybridLogicRules> logicRules)
    : Base(logicRules), mpcPtr_(&mpc), mpcSettings_(mpc.settings()), numMpcIterations_(0) {}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  numMpcIterations_ = 0;
  mpcPtr_->reset();
  setTargetTrajectories(initCostDesiredTrajectories);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories) {
  if (mpcSettings_.debugPrint_) {
    std::cerr << "### The target position is updated to" << std::endl;
    targetTrajectories.display();
  }
  mpcPtr_->getSolverPtr()->setCostDesiredTrajectories(targetTrajectories);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::setCurrentObservation(const system_observation_t& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::setModeSequence(const mode_sequence_template_t& modeSequenceTemplate) {
  mpcPtr_->setNewLogicRulesTemplate(modeSequenceTemplate);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::advanceMpc() {
  std::chrono::time_point<std::chrono::steady_clock> startTime, finishTime;
  if (mpcSettings_.debugPrint_) {
    startTime = std::chrono::steady_clock::now();
  }

  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    mpcPtr_->run(currentObservation_.time(), currentObservation_.state());
  }

  fillMpcOutputBuffers();
  // Incrementing numIterations must happen after fillMpcOutputBuffers
  numMpcIterations_++;

  // measure the delay
  if (mpcSettings_.debugPrint_) {
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
void MPC_Interface<STATE_DIM, INPUT_DIM>::fillMpcOutputBuffers() {
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
  std::lock_guard<std::mutex> lock(this->policyBufferMutex_);

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

  // FIXME(jcarius) concatenate controllers into a single controller
  // TODO(johannes) we might want to instantiate a feedforward controller if requested in the settings (see mrt ros interface)
  this->mpcControllerBuffer_.clear();
  this->mpcControllerBuffer_.reserve(controllerPtrs->size());
  for (auto controllerPtr : *controllerPtrs) {
    if (controllerPtr->empty()) {
      continue;
    }
    switch (controllerPtr->getType()) {
      case ControllerType::FEEDFORWARD: {
        using ctrl_t = FeedforwardController<STATE_DIM, INPUT_DIM>;
        this->mpcControllerBuffer_.emplace_back(std::unique_ptr<controller_t>(new ctrl_t(*static_cast<ctrl_t*>(controllerPtr))));
        break;
      }
      case ControllerType::LINEAR: {
        using ctrl_t = LinearController<STATE_DIM, INPUT_DIM>;
        this->mpcControllerBuffer_.emplace_back(std::unique_ptr<controller_t>(new ctrl_t(*static_cast<ctrl_t*>(controllerPtr))));
        break;
      }
      default: {
        ROS_WARN("MPC_Interface::fillMpcOutputBuffers: No controller copied into buffer.");
        this->mpcControllerBuffer_.push_back(nullptr);
      }
    }
  }

  this->eventTimesBuffer_ = *eventTimesPtr;
  this->subsystemsSequenceBuffer_ = *subsystemsSequencePtr;
  this->mpcCostDesiredTrajectoriesBuffer_ = *solverCostDesiredTrajectoriesPtr;

  this->newPolicyInBuffer_ = true;
  this->policyReceivedEver_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
bool MPC_Interface<STATE_DIM, INPUT_DIM>::updatePolicy() {
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
const typename MPC_Interface<STATE_DIM, INPUT_DIM>::state_vector_array_t& MPC_Interface<STATE_DIM, INPUT_DIM>::getMpcStateTrajectory()
    const {
  return this->mpcStateTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_Interface<STATE_DIM, INPUT_DIM>::input_vector_array_t& MPC_Interface<STATE_DIM, INPUT_DIM>::getMpcInputTrajectory()
    const {
  return mpcInputTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_Interface<STATE_DIM, INPUT_DIM>::scalar_array_t& MPC_Interface<STATE_DIM, INPUT_DIM>::getMpcTimeTrajectory() const {
  return this->mpcTimeTrajectory_;
}

}  // namespace ocs2
