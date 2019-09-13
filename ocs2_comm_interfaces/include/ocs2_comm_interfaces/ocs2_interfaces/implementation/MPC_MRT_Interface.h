
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::MPC_MRT_Interface(mpc_t& mpc, std::shared_ptr<HybridLogicRules> logicRules)
    : Base(std::move(logicRules)), mpc_(mpc), numMpcIterations_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  numMpcIterations_ = 0;
  mpc_.reset();
  setTargetTrajectories(initCostDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories) {
  if (mpc_.settings().debugPrint_) {
    std::cerr << "### The target position is updated to" << std::endl;
    targetTrajectories.display();
  }
  mpc_.getSolverPtr()->setCostDesiredTrajectories(targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setCurrentObservation(const system_observation_t& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::setModeSequence(const mode_sequence_template_t& modeSequenceTemplate) {
  mpc_.setNewLogicRulesTemplate(modeSequenceTemplate);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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
  fillMpcOutputBuffers(std::move(mpcInitObservation), mpc_, this->policyBuffer_.get(), this->commandBuffer_.get());

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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::fillMpcOutputBuffers(system_observation_t mpcInitObservation, mpc_t& mpc,
                                                                   policy_data_t* policyDataPtr, command_data_t* commandDataPtr) {
  // buffer policy mutex
  std::lock_guard<std::mutex> policyBufferLock(this->policyBufferMutex_);

  // policy
  const scalar_array2_t* timeTrajectoriesPtr = mpc.getOptimizedTimeTrajectoryPtr();
  const state_vector_array2_t* stateTrajectoriesPtr = mpc.getOptimizedStateTrajectoryPtr();
  const input_vector_array2_t* inputTrajectoriesPtr = mpc.getOptimizedInputTrajectoryPtr();
  if (mpc.settings().useFeedbackPolicy_) {
    policyDataPtr->fill(timeTrajectoriesPtr, stateTrajectoriesPtr, inputTrajectoriesPtr, mpc.getLogicRulesPtr()->eventTimes(),
                        mpc.getLogicRulesPtr()->subsystemsSequence(), mpc.getOptimizedControllersPtr());
  } else {
    policyDataPtr->fill(timeTrajectoriesPtr, stateTrajectoriesPtr, inputTrajectoriesPtr, mpc.getLogicRulesPtr()->eventTimes(),
                        mpc.getLogicRulesPtr()->subsystemsSequence());
  }

  // command
  commandDataPtr->fill(mpcInitObservation, mpc.getSolverPtr()->getCostDesiredTrajectories());

  // logic
  this->partitioningTimesUpdate(mpcInitObservation.time(), this->partitioningTimesBuffer_);

  // allow user to modify the buffer
  this->modifyBufferPolicy(*commandDataPtr, *policyDataPtr);

  // Flags to be set last:
  this->newPolicyInBuffer_ = true;
  this->policyReceivedEver_ = true;
  this->policyUpdatedBuffer_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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
typename MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::scalar_t MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getValueFunction(
    scalar_t time, const state_vector_t& state) {
  return mpc_.getSolverPtr()->getValueFunction(time, state);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state,
                                                                              state_vector_t& Vx) {
  mpc_.getSolverPtr()->getValueFunctionStateDerivative(time, state, Vx);
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state,
                                                                                dynamic_vector_t& nu) const {
  mpc_.getSolverPtr()->getStateInputConstraintLagrangian(time, state, nu);
}

}  // namespace ocs2
