//
// Created by johannes on 01.04.19.
//

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_Interface(
    mpc_t& mpc,
    const std::string& robotName /*= "robot"*/)

    : mpcPtr_(&mpc)
    , mpcSettings_(mpc.settings())
    , robotName_(robotName)
    , desiredTrajectoriesUpdated_(false)
    , modeSequenceUpdated_(false)
    , timeTrajectoriesPtr_(nullptr)
    , stateTrajectoriesPtr_(nullptr)
    , inputTrajectoriesPtr_(nullptr)
    , controllersPtr_(nullptr)
    , eventTimesPtr_(nullptr)
    , subsystemsSequencePtr_(nullptr)
{
  // correcting rosMsgTimeWindow
  if (mpcSettings_.recedingHorizon_==false)
    mpcSettings_.rosMsgTimeWindow_ = 1e+6;

  // reset variables
  reset();
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset() {

  initialCall_ = true;
  numIterations_ = 0;

  if (mpcPtr_ != nullptr)
    mpcPtr_->reset();
}

template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setTargetTrajectories(
    const cost_desired_trajectories_t& targetTrajectories) {

  if (desiredTrajectoriesUpdated_ == false) {
    costDesiredTrajectories_ = targetTrajectories;
    desiredTrajectoriesUpdated_ = true;
  }
}

template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setModeSequence(
    const mode_sequence_template_t& modeSequenceTemplate) {

  if (modeSequenceUpdated_ == false) {
    modeSequenceTemplate_ = modeSequenceTemplate;
    modeSequenceUpdated_ = true;
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULSES_T>
void
MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULSES_T>::advance(){


  if (mpcSettings_.adaptiveRosMsgTimeWindow_==true || mpcSettings_.debugPrint_)
    startTimePoint_ = std::chrono::steady_clock::now();

  // number of iterations
  numIterations_++;

  if (initialCall_==true) {
    // reset the MPC solver since it is the beginning of the task
    mpcPtr_->reset();

    // display
    if (mpcSettings_.debugPrint_) {
      std::cerr << "### The target position is updated at time "
                << std::setprecision(4) << currentObservation_.time() << " as " << std::endl;
      defaultCostDesiredTrajectories_.display();
    }

    // set CostDesiredTrajectories
    mpcPtr_->swapCostDesiredTrajectories(defaultCostDesiredTrajectories_);
  }

  // update the mode sequence
  if(modeSequenceUpdated_==true) {

    // display
    std::cerr << "### The mode sequence is updated at time "
              << std::setprecision(4) << currentObservation_.time() << " as " << std::endl;
    modeSequenceTemplate_.display();

    // set CostDesiredTrajectories
    mpcPtr_->setNewLogicRulesTemplate(modeSequenceTemplate_);

    modeSequenceUpdated_ = false;

  } else if (mpcSettings_.recedingHorizon_==false) {
    return;
  }

  // update the desired trajectories
  if(desiredTrajectoriesUpdated_==true) {

    // display
    if (mpcSettings_.debugPrint_) {
      std::cerr << "### The target position is updated at time "
                << std::setprecision(4) << currentObservation_.time() << " as " << std::endl;
      costDesiredTrajectories_.display();
    }

    // set CostDesiredTrajectories
    mpcPtr_->swapCostDesiredTrajectories(costDesiredTrajectories_);

    desiredTrajectoriesUpdated_ = false;

  } else if (mpcSettings_.recedingHorizon_==false) {
    return;
  }

  // run SLQ-MPC
  bool controllerIsUpdated = mpcPtr_->run(
      currentObservation_.time(),
      currentObservation_.state());

  // get a pointer to the optimized controller
  mpcPtr_->getOptimizedControllerPtr(controllersPtr_);
  // get a pointer to the optimized trajectories
  mpcPtr_->getOptimizedTrajectoriesPtr(
      timeTrajectoriesPtr_,
      stateTrajectoriesPtr_,
      inputTrajectoriesPtr_);

  //TODO: Is this necessary? Has the trajectory been modified by the solver?
  // get a pointer to CostDesiredTrajectories
  mpcPtr_->getCostDesiredTrajectoriesPtr(solverCostDesiredTrajectoriesPtr_);

  // get a pointer to event times and motion sequence
  eventTimesPtr_ = &mpcPtr_->getLogicRulesPtr()->eventTimes();
  subsystemsSequencePtr_ = &mpcPtr_->getLogicRulesPtr()->subsystemsSequence();

  // measure the delay
  if(mpcSettings_.adaptiveRosMsgTimeWindow_==true || mpcSettings_.debugPrint_){
    finalTimePoint_ = std::chrono::steady_clock::now();
    currentDelay_ = std::chrono::duration<scalar_t, std::milli>(finalTimePoint_-startTimePoint_).count();
    meanDelay_ += (currentDelay_-meanDelay_) / numIterations_;
    maxDelay_   = std::max(maxDelay_, currentDelay_);
  }

  // measure the delay for sending ROS messages
  if(mpcSettings_.adaptiveRosMsgTimeWindow_==true)
    currentDelay_ = std::min(currentDelay_, meanDelay_*0.9);
  else
    currentDelay_ = 0.0;

  // display
  if(mpcSettings_.debugPrint_){
    std::cerr << std::endl;
    std::cerr << "### Average duration of MPC optimization is: " << meanDelay_ << " [ms]." << std::endl;
    std::cerr << "### Maximum duration of MPC optimization is: " << maxDelay_ << " [ms]." << std::endl;
  }
  // set the initialCall flag to false
  if (initialCall_==true)
    initialCall_ = false;
}

}
