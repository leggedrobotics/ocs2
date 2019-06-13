//
// Created by johannes on 01.04.19.
//

#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_Interface(mpc_t& mpc, const LOGIC_RULES_T& logicRules,
                                                                  const bool& useFeedforwardPolicy /*= true*/)
    : mpcPtr_(&mpc),
      mpcSettings_(mpc.settings()),
      desiredTrajectoriesUpdated_(false),
      modeSequenceUpdated_(false),
      observationUpdated_(false),
      logicMachine_(logicRules),
      useFeedforwardPolicy_(useFeedforwardPolicy) {
  // reset variables
  reset();
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset() {
  initialCall_ = true;
  numIterations_ = 0;
  desiredTrajectoriesUpdated_ = false;
  observationUpdated_ = false;
  mpcOutputBufferUpdated_ = false;
  logicUpdated_ = false;

  // MPC inputs
  costDesiredTrajectories_.clear();
  currentObservation_ = system_observation_t();

  // MPC outputs:
  mpcSubsystemsSequenceBuffer_.clear();
  mpcEventTimesBuffer_.clear();
  mpcTimeTrajectoryBuffer_.clear();
  mpcStateTrajectoryBuffer_.clear();
  mpcInputTrajectoryBuffer_.clear();
  mpcControllerBuffer_.clear();
  mpcSolverCostDesiredTrajectoriesBuffer_.clear();

  // variables for the tracking controller:
  eventTimes_.clear();
  subsystemsSequence_.clear();
  partitioningTimes_.clear();

  mpcController_.clear();
  mpcTimeTrajectory_.clear();
  mpcStateTrajectory_.clear();
  mpcInputTrajectory_.clear();
  mpcLinInterpolateState_.reset();
  mpcLinInterpolateInput_.reset();
  mpcLinInterpolateUff_.reset();
  mpcLinInterpolateK_.reset();
  mpcCostDesiredTrajectories_.clear();

  if (mpcPtr_ != nullptr) mpcPtr_->reset();
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories) {
  std::lock_guard<std::mutex> lock(desiredTrajectoryMutex_);
  costDesiredTrajectories_ = targetTrajectories;
  desiredTrajectoriesUpdated_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCurrentObservation(const system_observation_t& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
  observationUpdated_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setModeSequence(const mode_sequence_template_t& modeSequenceTemplate) {
  std::lock_guard<std::mutex> lock(modeSequenceMutex_);
  modeSequenceTemplate_ = modeSequenceTemplate;
  modeSequenceUpdated_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULSES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULSES_T>::advanceMpc() {
  if (mpcSettings_.debugPrint_) startTimePoint_ = std::chrono::steady_clock::now();

  // number of iterations
  numIterations_++;

  if (initialCall_ == true) {
    // reset the MPC solver since it is the beginning of the task
    mpcPtr_->reset();
  }

  updateModeSequence();
  updateDesiredTrajectory();

  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    // run SLQ-MPC
    mpcPtr_->run(currentObservation_.time(), currentObservation_.state());
    // allow for new observations:
    observationUpdated_ = false;
  }

  fillMpcOutputBuffers();

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
  if (initialCall_ == true) initialCall_ = false;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::fillMpcOutputBuffers() {
  const controller_array_t* controllersPtr(nullptr);
  const std::vector<scalar_array_t>* timeTrajectoriesPtr(nullptr);
  const state_vector_array2_t* stateTrajectoriesPtr(nullptr);
  const input_vector_array2_t* inputTrajectoriesPtr(nullptr);
  const scalar_array_t* eventTimesPtr(nullptr);
  ;
  const size_array_t* subsystemsSequencePtr(nullptr);
  ;
  const cost_desired_trajectories_t* solverCostDesiredTrajectoriesPtr(nullptr);
  ;

  // get a pointer to the optimized controller
  mpcPtr_->getOptimizedControllerPtr(controllersPtr);
  // get a pointer to the optimized trajectories
  mpcPtr_->getOptimizedTrajectoriesPtr(timeTrajectoriesPtr, stateTrajectoriesPtr, inputTrajectoriesPtr);

  // TODO: Is this necessary? Has the trajectory been modified by the solver?
  mpcPtr_->getCostDesiredTrajectoriesPtr(solverCostDesiredTrajectoriesPtr);

  // get a pointer to event times and motion sequence
  eventTimesPtr = &mpcPtr_->getLogicRulesPtr()->eventTimes();
  subsystemsSequencePtr = &mpcPtr_->getLogicRulesPtr()->subsystemsSequence();

  std::lock_guard<std::mutex> lock(mpcBufferMutex);

  // update buffers
  if (useFeedforwardPolicy_) {
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
  } else {
    int N = 0;
    for (int i = 0; i < controllersPtr->size(); i++) {
      N += (*controllersPtr)[i].time_.size();
    }
    mpcControllerBuffer_.clear();
    mpcControllerBuffer_.time_.reserve(N);
    mpcControllerBuffer_.k_.reserve(N);
    mpcControllerBuffer_.deltaUff_.reserve(N);
    mpcControllerBuffer_.uff_.reserve(N);

    for (int i = 0; i < controllersPtr->size(); i++) {
      const controller_t& currentController = (*controllersPtr)[i];
      mpcControllerBuffer_.time_.insert(std::end(mpcControllerBuffer_.time_), std::begin(currentController.time_),
                                        std::end(currentController.time_));
      mpcControllerBuffer_.k_.insert(std::end(mpcControllerBuffer_.k_), std::begin(currentController.k_), std::end(currentController.k_));
      mpcControllerBuffer_.deltaUff_.insert(std::end(mpcControllerBuffer_.deltaUff_), std::begin(currentController.deltaUff_),
                                            std::end(currentController.deltaUff_));
      mpcControllerBuffer_.uff_.insert(std::end(mpcControllerBuffer_.uff_), std::begin(currentController.uff_),
                                       std::end(currentController.uff_));
    }
  }

  mpcEventTimesBuffer_ = *eventTimesPtr;
  mpcSubsystemsSequenceBuffer_ = *subsystemsSequencePtr;
  mpcSolverCostDesiredTrajectoriesBuffer_ = *solverCostDesiredTrajectoriesPtr;
  mpcOutputBufferUpdated_ = true;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::updateDesiredTrajectory() {
  if (desiredTrajectoriesUpdated_ == true) {
    std::lock_guard<std::mutex> lock(desiredTrajectoryMutex_);
    // display
    if (mpcSettings_.debugPrint_) {
      std::cerr << "### The target position is updated at time " << std::setprecision(4) << currentObservation_.time() << " as "
                << std::endl;
      costDesiredTrajectories_.display();
    }
    // set CostDesiredTrajectories
    mpcPtr_->swapCostDesiredTrajectories(costDesiredTrajectories_);
    desiredTrajectoriesUpdated_ = false;
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::updateModeSequence() {
  if (modeSequenceUpdated_ == true) {
    std::lock_guard<std::mutex> lock(modeSequenceMutex_);

    std::cerr << "### The mode sequence is updated at time " << std::setprecision(4) << currentObservation_.time() << " as " << std::endl;
    modeSequenceTemplate_.display();

    mpcPtr_->setNewLogicRulesTemplate(modeSequenceTemplate_);

    modeSequenceUpdated_ = false;
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::updatePolicy() {

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
    logicMachine_.getLogicRulesPtr()->setModeSequence(subsystemsSequence_, eventTimes_);
    // Tell logicMachine that logicRules are modified
    logicMachine_.logicRulesUpdated();
    // update logicMachine
    logicMachine_.updateLogicRules(partitioningTimes_);

    // function for finding active subsystem
    const size_t partitionIndex = 0;  // we assume only one partition
    findActiveSubsystemFnc_ = std::move(logicMachine_.getHandleToFindActiveEventCounter(partitionIndex));

    logicUpdated_ = false;
  }

  if (useFeedforwardPolicy_ == true) {
    mpcTimeTrajectory_.swap(mpcTimeTrajectoryBuffer_);
    mpcStateTrajectory_.swap(mpcStateTrajectoryBuffer_);
    mpcInputTrajectory_.swap(mpcInputTrajectoryBuffer_);

    mpcLinInterpolateState_.reset();
    mpcLinInterpolateState_.setTimeStamp(&mpcTimeTrajectory_);
    mpcLinInterpolateState_.setData(&mpcStateTrajectory_);

    mpcLinInterpolateInput_.reset();
    mpcLinInterpolateInput_.setTimeStamp(&mpcTimeTrajectory_);
    mpcLinInterpolateInput_.setData(&mpcInputTrajectory_);

  } else {
    mpcController_.swap(mpcControllerBuffer_);

    mpcLinInterpolateUff_.reset();
    mpcLinInterpolateUff_.setTimeStamp(&mpcController_.time_);
    mpcLinInterpolateUff_.setData(&mpcController_.uff_);

    mpcLinInterpolateK_.reset();
    mpcLinInterpolateK_.setTimeStamp(&mpcController_.time_);
    mpcLinInterpolateK_.setData(&mpcController_.k_);
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::evaluateFeedforwardPolicy(const scalar_t& time, state_vector_t& mpcState,
                                                                                   input_vector_t& mpcInput, size_t& subsystem) {
  updatePolicy();

  if (useFeedforwardPolicy_ == false) throw std::runtime_error("The MRT is set to receive the feedforward policy.");

  if (time > mpcTimeTrajectory_.back())
    ROS_WARN_STREAM_THROTTLE(2, "The requested time is greater than the received plan: " + std::to_string(time) + ">" +
                                    std::to_string(mpcTimeTrajectory_.back()));

  mpcLinInterpolateState_.interpolate(time, mpcState);
  int greatestLessTimeStampIndex = mpcLinInterpolateState_.getGreatestLessTimeStampIndex();
  mpcLinInterpolateInput_.interpolate(time, mpcInput, greatestLessTimeStampIndex);

  size_t index = findActiveSubsystemFnc_(time);
  subsystem = logicMachine_.getLogicRulesPtr()->subsystemsSequence().at(index);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::evaluateFeedbackPolicy(const scalar_t& time, input_vector_t& mpcUff,
                                                                                input_state_matrix_t& mpcGain, size_t& subsystem) {
  updatePolicy();

  if (useFeedforwardPolicy_ == true) throw std::runtime_error("The MRT is set to receive the feedback policy.");

  if (time > mpcController_.time_.back())
    ROS_WARN_STREAM_THROTTLE(2, "The requested time is greater than the received plan: " + std::to_string(time) + ">" +
                                    std::to_string(mpcController_.time_.back()));

  mpcLinInterpolateUff_.interpolate(time, mpcUff);
  int greatestLessTimeStampIndex = mpcLinInterpolateUff_.getGreatestLessTimeStampIndex();
  mpcLinInterpolateK_.interpolate(time, mpcGain, greatestLessTimeStampIndex);

  size_t index = findActiveSubsystemFnc_(time);
  subsystem = logicMachine_.getLogicRulesPtr()->subsystemsSequence().at(index);
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::partitioningTimesUpdate(scalar_array_t& partitioningTimes) const {
  // TODO: Is this correct?
  partitioningTimes.resize(2);
  partitioningTimes[0] = currentObservation_.time();
  partitioningTimes[1] = std::numeric_limits<scalar_t>::max();
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_vector_array_t&
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getMpcStateTrajectory() {
  updatePolicy();
  return mpcStateTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_vector_array_t&
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getMpcInputTrajectory() {
  updatePolicy();
  return mpcInputTrajectory_;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getMpcTimeTrajectory() {
  updatePolicy();
  return mpcTimeTrajectory_;
}

}  // namespace ocs2
