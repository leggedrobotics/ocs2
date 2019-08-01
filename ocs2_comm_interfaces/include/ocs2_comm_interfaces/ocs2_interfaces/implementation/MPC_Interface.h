//
// Created by johannes on 01.04.19.
//

#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_Interface<STATE_DIM, INPUT_DIM>::MPC_Interface(mpc_t& mpc, std::shared_ptr<HybridLogicRules> logicRules)
    : mpcPtr_(&mpc), mpcSettings_(mpc.settings()), observationUpdated_(false) {
  // reset variables
  reset();

  if (!logicRules) {
    logicRules.reset(new NullLogicRules());
  }
  logicMachine_.reset(new HybridLogicRulesMachine(std::move(logicRules)));
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::reset() {
  numIterations_ = 0;
  observationUpdated_ = false;
  mpcOutputBufferUpdated_ = false;
  logicUpdated_ = false;

  // MPC inputs
  currentObservation_ = system_observation_t();

  // MPC outputs:
  mpcSubsystemsSequenceBuffer_.clear();
  mpcEventTimesBuffer_.clear();
  mpcTimeTrajectoryBuffer_.clear();
  mpcStateTrajectoryBuffer_.clear();
  mpcInputTrajectoryBuffer_.clear();
  mpcControllersBuffer_.clear();
  mpcSolverCostDesiredTrajectoriesBuffer_.clear();

  // variables for the tracking controller:
  eventTimes_.clear();
  subsystemsSequence_.clear();
  partitioningTimes_.clear();

  mpcControllers_.clear();
  mpcTimeTrajectory_.clear();
  mpcStateTrajectory_.clear();
  mpcInputTrajectory_.clear();
  mpcCostDesiredTrajectories_.clear();

  mpcPtr_->reset();
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
  observationUpdated_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::setModeSequence(const mode_sequence_template_t& modeSequenceTemplate) {
  mpcPtr_->setNewLogicRulesTemplate(modeSequenceTemplate);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::advanceMpc() {
  if (mpcSettings_.debugPrint_) startTimePoint_ = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    // run MPC
    mpcPtr_->run(currentObservation_.time(), currentObservation_.state());
    // allow for new observations:
    observationUpdated_ = false;
  }

  fillMpcOutputBuffers();
  // Incrementing numIterations must happen after fillMpcOutputBuffers
  numIterations_++;

  // measure the delay
  if (mpcSettings_.debugPrint_) {
    finalTimePoint_ = std::chrono::steady_clock::now();
    currentDelay_ = std::chrono::duration<scalar_t, std::milli>(finalTimePoint_ - startTimePoint_).count();
    meanDelay_ += (currentDelay_ - meanDelay_) / numIterations_;
    maxDelay_ = std::max(maxDelay_, currentDelay_);
  }

  // display
  if (mpcSettings_.debugPrint_) {
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
  const scalar_array_t* eventTimesPtr(nullptr);
  const size_array_t* subsystemsSequencePtr(nullptr);
  const cost_desired_trajectories_t* solverCostDesiredTrajectoriesPtr(nullptr);

  // get a pointer to the optimized controllers
  controllerPtrs = mpcPtr_->getOptimizedControllerPtr();
  // get a pointer to the optimized trajectories
  mpcPtr_->getOptimizedTrajectoriesPtr(timeTrajectoriesPtr, stateTrajectoriesPtr, inputTrajectoriesPtr);

  // TODO(johannes) Is this necessary? Has the trajectory been modified by the solver?
  mpcPtr_->getCostDesiredTrajectoriesPtr(solverCostDesiredTrajectoriesPtr);

  // get a pointer to event times and motion sequence
  eventTimesPtr = &mpcPtr_->getLogicRulesPtr()->eventTimes();
  subsystemsSequencePtr = &mpcPtr_->getLogicRulesPtr()->subsystemsSequence();

  std::lock_guard<std::mutex> lock(mpcBufferMutex);

  // update buffers, i.e., copy MPC results into our buffers
  int N = 0;
  for (int i = 0; i < timeTrajectoriesPtr->size(); i++) {
    N += (*timeTrajectoriesPtr)[i].size();
  }
  mpcTimeTrajectoryBuffer_.clear();
  mpcTimeTrajectoryBuffer_.reserve(N);
  mpcStateTrajectoryBuffer_.clear();
  mpcStateTrajectoryBuffer_.reserve(N);
  mpcInputTrajectoryBuffer_.clear();
  mpcInputTrajectoryBuffer_.reserve(N);
  for (int i = 0; i < timeTrajectoriesPtr->size(); i++) {
    mpcTimeTrajectoryBuffer_.insert(std::end(mpcTimeTrajectoryBuffer_), std::begin((*timeTrajectoriesPtr)[i]),
                                    std::end((*timeTrajectoriesPtr)[i]));
    mpcStateTrajectoryBuffer_.insert(std::end(mpcStateTrajectoryBuffer_), std::begin((*stateTrajectoriesPtr)[i]),
                                     std::end((*stateTrajectoriesPtr)[i]));
    mpcInputTrajectoryBuffer_.insert(std::end(mpcInputTrajectoryBuffer_), std::begin((*inputTrajectoriesPtr)[i]),
                                     std::end((*inputTrajectoriesPtr)[i]));
  }

  // TODO(jcarius) concatenate controllers into a single controller
  // TODO(johannes) we might want to instantiate a feedforward controller if requested in the settings (see mrt ros interface)
  mpcControllersBuffer_.clear();
  mpcControllersBuffer_.reserve(controllerPtrs->size());
  for (auto controllerPtr : *controllerPtrs) {
    if (controllerPtr->empty()) {
      continue;
    }
    switch (controllerPtr->getType()) {
      case ControllerType::FEEDFORWARD: {
        using ctrl_t = FeedforwardController<STATE_DIM, INPUT_DIM>;
        mpcControllersBuffer_.emplace_back(std::unique_ptr<controller_t>(new ctrl_t(*static_cast<ctrl_t*>(controllerPtr))));
        break;
      }
      case ControllerType::LINEAR: {
        using ctrl_t = LinearController<STATE_DIM, INPUT_DIM>;
        mpcControllersBuffer_.emplace_back(std::unique_ptr<controller_t>(new ctrl_t(*static_cast<ctrl_t*>(controllerPtr))));
        break;
      }
      default: {
        ROS_WARN("MPC_Interface::fillMpcOutputBuffers: No controller copied into buffer.");
        mpcControllersBuffer_.push_back(nullptr);
      }
    }
  }

  mpcEventTimesBuffer_ = *eventTimesPtr;
  mpcSubsystemsSequenceBuffer_ = *subsystemsSequencePtr;
  mpcSolverCostDesiredTrajectoriesBuffer_ = *solverCostDesiredTrajectoriesPtr;
  mpcOutputBufferUpdated_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
bool MPC_Interface<STATE_DIM, INPUT_DIM>::updatePolicy() {
  std::lock_guard<std::mutex> lock(mpcBufferMutex);

  if (mpcOutputBufferUpdated_) {
    mpcOutputBufferUpdated_ = false;
  } else {
    return false;
  }

  mpcCostDesiredTrajectories_.swap(mpcSolverCostDesiredTrajectoriesBuffer_);

  if (subsystemsSequence_ != mpcSubsystemsSequenceBuffer_) {
    subsystemsSequence_.swap(mpcSubsystemsSequenceBuffer_);
    logicUpdated_ = true;
  }
  if (eventTimes_ != mpcEventTimesBuffer_) {
    eventTimes_.swap(mpcEventTimesBuffer_);
    logicUpdated_ = true;
  }
  partitioningTimesUpdate(partitioningTimes_);

  if (logicUpdated_ == true) {
    // set mode sequence
    logicMachine_->getLogicRulesPtr()->setModeSequence(subsystemsSequence_, eventTimes_);
    // Tell logicMachine that logicRules are modified
    logicMachine_->logicRulesUpdated();
    // update logicMachine
    logicMachine_->updateLogicRules(partitioningTimes_);

    // function for finding active subsystem
    const size_t partitionIndex = 0;  // we assume only one partition
    findActiveSubsystemFnc_ = std::move(logicMachine_->getHandleToFindActiveEventCounter(partitionIndex));

    logicUpdated_ = false;
  }

  mpcTimeTrajectory_.swap(mpcTimeTrajectoryBuffer_);
  mpcStateTrajectory_.swap(mpcStateTrajectoryBuffer_);
  mpcInputTrajectory_.swap(mpcInputTrajectoryBuffer_);

  mpcLinInterpolateState_.setData(&mpcTimeTrajectory_, &mpcStateTrajectory_);
  mpcLinInterpolateInput_.setData(&mpcTimeTrajectory_, &mpcInputTrajectory_);

  mpcControllers_.swap(mpcControllersBuffer_);

  return true;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::evaluatePolicy(const scalar_t& time, const state_vector_t& currentState, state_vector_t& mpcState,
                                                         input_vector_t& mpcInput, size_t& subsystem) {
  updatePolicy();

  if (time > mpcTimeTrajectory_.back()) {
    ROS_WARN_STREAM_THROTTLE(2, "The requested time is greater than the received plan: " + std::to_string(time) + ">" +
                                    std::to_string(mpcTimeTrajectory_.back()));
  }

  mpcLinInterpolateState_.interpolate(time, mpcState);

  size_t index = findActiveSubsystemFnc_(time);
  subsystem = logicMachine_->getLogicRulesPtr()->subsystemsSequence().at(index);

  // TODO(jcarius) is index correct here or should we use subsystem?
  mpcInput = mpcControllers_[index]->computeInput(time, currentState);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_Interface<STATE_DIM, INPUT_DIM>::partitioningTimesUpdate(scalar_array_t& partitioningTimes) const {
  // TODO: Is this correct?
  partitioningTimes.resize(2);
  partitioningTimes[0] = currentObservation_.time();
  partitioningTimes[1] = std::numeric_limits<scalar_t>::max();
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_Interface<STATE_DIM, INPUT_DIM>::state_vector_array_t& MPC_Interface<STATE_DIM, INPUT_DIM>::getMpcStateTrajectory() {
  updatePolicy();
  return mpcStateTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_Interface<STATE_DIM, INPUT_DIM>::input_vector_array_t& MPC_Interface<STATE_DIM, INPUT_DIM>::getMpcInputTrajectory() {
  updatePolicy();
  return mpcInputTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_Interface<STATE_DIM, INPUT_DIM>::scalar_array_t& MPC_Interface<STATE_DIM, INPUT_DIM>::getMpcTimeTrajectory() {
  updatePolicy();
  return mpcTimeTrajectory_;
}

}  // namespace ocs2
