
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::MPC_MRT_Interface(mpc_t& mpc, std::shared_ptr<HybridLogicRules> logicRules)
    : Base(std::move(logicRules)), mpc_(mpc) {
  // correcting solutionTimeWindow
  if (!mpc_.settings().recedingHorizon_) {
    mpc_.settings().solutionTimeWindow_ = -1;
  }

  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  mpc_.reset();
  mpcTimer_.reset();
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
  // measure the delay in running MPC
  mpcTimer_.startTimer();

  system_observation_t currentObservation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    currentObservation = currentObservation_;
  }

  bool controllerIsUpdated = mpc_.run(currentObservation.time(), currentObservation.state());
  if (!controllerIsUpdated) {
    return;
  }
  fillMpcOutputBuffers(currentObservation, mpc_);

  // measure the delay for sending ROS messages
  mpcTimer_.endTimer();

  // check MPC delay and solution window compatibility
  scalar_t timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time();
  }
  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) {
    std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!" << std::endl;
  }

  // measure the delay
  if (mpc_.settings().debugPrint_) {
    std::cerr << std::endl;
    std::cerr << "### MPC ROS runtime " << std::endl;
    std::cerr << "###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::fillMpcOutputBuffers(system_observation_t mpcInitObservation, const mpc_t& mpc) {
  // buffer policy mutex
  std::lock_guard<std::mutex> policyBufferLock(this->policyBufferMutex_);

  // get solution
  scalar_t startTime = mpcInitObservation.time();
  scalar_t finalTime = startTime + mpc.settings().solutionTimeWindow_;
  if (mpc.settings().solutionTimeWindow_ < 0) {
    finalTime = mpc.getSolverPtr()->getFinalTime();
  }
  mpc.getSolverPtr()->getPrimalSolution(finalTime, this->primalSolutionBuffer_.get());

  // command
  this->commandBuffer_->mpcInitObservation_ = std::move(mpcInitObservation);
  this->commandBuffer_->mpcCostDesiredTrajectories_ = mpc.getSolverPtr()->getCostDesiredTrajectories();

  // logic
  this->partitioningTimesUpdate(startTime, this->partitioningTimesBuffer_);

  // allow user to modify the buffer
  this->modifyBufferPolicy(*this->commandBuffer_, *this->primalSolutionBuffer_);

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
  auto controller = dynamic_cast<LinearController<STATE_DIM, INPUT_DIM>*>(this->currentPrimalSolution_->controllerPtr_.get());
  if (!controller) {
    throw std::runtime_error("Feedback gains only available with linear controller");
  }
  controller->getFeedbackGain(time, K);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::scalar_t MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getValueFunction(
    scalar_t time, const state_vector_t& state) {
  return mpc_.getSolverPtr()->getValueFunction(time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state,
                                                                              state_vector_t& Vx) {
  mpc_.getSolverPtr()->getValueFunctionStateDerivative(time, state, Vx);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_MRT_Interface<STATE_DIM, INPUT_DIM>::getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state,
                                                                                dynamic_vector_t& nu) const {
  mpc_.getSolverPtr()->getStateInputConstraintLagrangian(time, state, nu);
}

}  // namespace ocs2
