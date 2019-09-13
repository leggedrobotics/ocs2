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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::MPC_ROS_Interface(
    mpc_t* mpcPtr, const std::string& robotName /*= "robot"*/,
    const task_listener_ptr_array_t& taskListenerArray /*= task_listener_ptr_array_t()*/)
    : taskListenerArray_(taskListenerArray) {
  set(mpcPtr, robotName);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::~MPC_ROS_Interface() {
  shutdownNode();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::set(mpc_t* mpcPtr, const std::string& robotName /*= "robot"*/) {
  if (!mpcPtr) {
    throw std::runtime_error("MPC pointer should be provided.");
  }

  mpcPtr_ = mpcPtr;
  mpcSettings_ = mpcPtr->settings();
  robotName_ = robotName;

  terminateThread_ = false;
  readyToPublish_ = false;

  initialCall_ = false;
  resetRequestedEver_ = false;

  // correcting rosMsgTimeWindow
  if (!mpcSettings_.recedingHorizon_) {
    mpcSettings_.rosMsgTimeWindow_ = 1e+6;
  }

  mpcTimer_.reset();
  currentDelay_ = 0.0;

  // Start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::sigintHandler(int sig) {
  ROS_INFO_STREAM("Shutting MPC node.");
  ::ros::shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::reset(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);

  if (mpcPtr_ != nullptr) {
    mpcPtr_->reset();
  }

  initialCall_ = true;
  resetRequestedEver_ = true;

  mpcPtr_->getSolverPtr()->setCostDesiredTrajectories(initCostDesiredTrajectories);

  mpcTimer_.reset();
  currentDelay_ = 0.0;

  terminateThread_ = false;
  readyToPublish_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res) {
  if (static_cast<bool>(req.reset)) {
    cost_desired_trajectories_t initCostDesiredTrajectories;
    RosMsgConversions<STATE_DIM, INPUT_DIM>::readTargetTrajectoriesMsg(req.targetTrajectories, initCostDesiredTrajectories);
    reset(initCostDesiredTrajectories);

    res.done = true;

    std::cerr << std::endl
              << "\n#####################################################"
              << "\n#####################################################"
              << "\n#################  MPC is reset.  ###################"
              << "\n#####################################################"
              << "\n#####################################################" << std::endl;

  } else {
    ROS_WARN_STREAM("Ineffective reset request.");
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::publishDummy() {
  ocs2_msgs::dummy msg;
  msg.ping = 1;
  dummyPublisher_.publish(msg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::publishPolicy(bool controllerIsUpdated, policy_data_t policyData,
                                                            command_data_t commandData) {
#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

  mpcPolicyMsg_.controllerIsUpdated = controllerIsUpdated;

  ros_msg_conversions_t::createObservationMsg(commandData.mpcInitObservation_, mpcPolicyMsg_.initObservation);
  ros_msg_conversions_t::createTargetTrajectoriesMsg(commandData.mpcCostDesiredTrajectories_, mpcPolicyMsg_.planTargetTrajectories);

  ros_msg_conversions_t::createModeSequenceMsg(policyData.eventTimes_, policyData.subsystemsSequence_, mpcPolicyMsg_.modeSequence);

  ControllerType controllerType = policyData.mpcController_->getType();

  // translate controllerType enum into message enum
  switch (controllerType) {
    case ControllerType::FEEDFORWARD: {
      mpcPolicyMsg_.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    }
    case ControllerType::LINEAR: {
      mpcPolicyMsg_.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    }
    default: {
      throw std::runtime_error("MPC_ROS_Interface: Unknown controller type.");
      break;
    }
  }

  // maximum length of the message
  size_t N = policyData.mpcTimeTrajectory_.size();

  mpcPolicyMsg_.timeTrajectory.clear();
  mpcPolicyMsg_.timeTrajectory.reserve(N);
  mpcPolicyMsg_.stateTrajectory.clear();
  mpcPolicyMsg_.stateTrajectory.reserve(N);
  mpcPolicyMsg_.data.clear();
  mpcPolicyMsg_.data.reserve(N);

  ocs2_msgs::mpc_state mpcState;
  mpcState.value.resize(STATE_DIM);
  ocs2_msgs::mpc_input mpcInput;
  mpcInput.value.resize(INPUT_DIM);

  // The message truncation time
  const scalar_t t0 = commandData.mpcInitObservation_.time() + currentDelay_ * 1e-3;
  const scalar_t tf = commandData.mpcInitObservation_.time() + mpcSettings_.rosMsgTimeWindow_ * 1e-3;
  if (tf < t0 + 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) {
    std::cerr << "WARNING: Message publishing time-horizon is shorter than the MPC delay!" << std::endl;
  }

  const scalar_array_t& timeTrajectory = policyData.mpcTimeTrajectory_;
  const state_vector_array_t& stateTrajectory = policyData.mpcStateTrajectory_;
  const input_vector_array_t& inputTrajectory = policyData.mpcInputTrajectory_;
  const controller_t* controller = policyData.mpcController_.get();

  scalar_array_t timeTrajectoryTruncated;
  std::vector<std::vector<float>*> policyMsgDataPointers;
  policyMsgDataPointers.reserve(N);

  for (size_t k = 0; k < N; k++) {  // loop through time
    // continue if elapsed time is smaller than computation time delay
    if (k < N - 1 && timeTrajectory[k + 1] < t0) {
      continue;
    }
    // break if the time exceed rosMsgTimeWindow
    if (k > 0 && timeTrajectory[k - 1] > tf) {
      break;
    }

    for (size_t j = 0; j < STATE_DIM; j++) {
      mpcState.value[j] = stateTrajectory[k](j);
    }
    for (size_t j = 0; j < INPUT_DIM; j++) {
      mpcInput.value[j] = inputTrajectory[k](j);
    }

    mpcPolicyMsg_.timeTrajectory.push_back(timeTrajectory[k]);
    mpcPolicyMsg_.stateTrajectory.push_back(mpcState);
    mpcPolicyMsg_.inputTrajectory.push_back(mpcInput);
    mpcPolicyMsg_.data.emplace_back(ocs2_msgs::controller_data());

    policyMsgDataPointers.push_back(&mpcPolicyMsg_.data.back().data);
    timeTrajectoryTruncated.push_back(timeTrajectory[k]);
  }  // end of k loop

  controller->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

#ifdef PUBLISH_THREAD
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();
#else
  mpcPolicyPublisher_.publish(mpcPolicyMsg_);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::publisherWorkerThread() {
  while (!terminateThread_) {
    std::unique_lock<std::mutex> lk(publisherMutex_);

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    if (terminateThread_) {
      break;
    }

    mpcPolicyMsgBuffer_ = std::move(mpcPolicyMsg_);

    readyToPublish_ = false;

    lk.unlock();
    msgReady_.notify_one();

    // publish the message
    mpcPolicyPublisher_.publish(mpcPolicyMsgBuffer_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::fillMpcOutputBuffers(system_observation_t mpcInitObservation, const mpc_t& mpc,
                                                                   policy_data_t* policyDataPtr, command_data_t* commandDataPtr) {
  // get solution
  mpc.getSolverPtr()->getSolutionPtr(policyDataPtr);

  // command
  commandDataPtr->fill(mpcInitObservation, mpc.getSolverPtr()->getCostDesiredTrajectories());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);

  if (!resetRequestedEver_.load()) {
    ROS_WARN_STREAM("MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service.");
    return;
  }

  // current time, state, input, and subsystem
  system_observation_t currentObservation;
  ros_msg_conversions_t::readObservationMsg(*msg, currentObservation);

  if (mpcSettings_.adaptiveRosMsgTimeWindow_ || mpcSettings_.debugPrint_) {
    mpcTimer_.startTimer();
  }

  if (initialCall_) {
    // after each reset, perform user defined operation if specialized
    initCall(currentObservation);
  }

  // update task listeners
  for (auto& taskListener : taskListenerArray_) {
    taskListener->update();
  }

  // run SLQ-MPC
  bool controllerIsUpdated = mpcPtr_->run(currentObservation.time(), currentObservation.state());
  policy_data_t policyData;
  command_data_t commandData;
  fillMpcOutputBuffers(currentObservation, *mpcPtr_, &policyData, &commandData);

  // measure the delay for sending ROS messages
  if (mpcSettings_.adaptiveRosMsgTimeWindow_ || mpcSettings_.debugPrint_) {
    mpcTimer_.endTimer();
  }

  // measure the delay for sending ROS messages
  if (mpcSettings_.adaptiveRosMsgTimeWindow_) {
    currentDelay_ = std::min(mpcTimer_.getLastIntervalInMilliseconds(), mpcTimer_.getAverageInMilliseconds() * 0.9);
  } else {
    currentDelay_ = 0.0;
  }

  // display
  if (mpcSettings_.debugPrint_) {
    std::cerr << std::endl;
    std::cerr << "### MPC ROS runtime " << std::endl;
    std::cerr << "###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

#ifdef PUBLISH_DUMMY

  // publish dummy for test
  publishDummy();

#else

  // publish optimized output
  publishPolicy(controllerIsUpdated, std::move(policyData), std::move(commandData));

#endif

  // set the initialCall flag to false
  initialCall_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcTargetTrajectoriesCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) {
  if (!mpcSettings_.recedingHorizon_) {
    throw std::runtime_error("Target trajectories can only be updated in receding horizon mode.");
  }

  cost_desired_trajectories_t costDesiredTrajectories;
  RosMsgConversions<STATE_DIM, INPUT_DIM>::readTargetTrajectoriesMsg(*msg, costDesiredTrajectories);

  if (mpcSettings_.debugPrint_) {
    std::cerr << "### The target position is updated to " << std::endl;
    costDesiredTrajectories.display();
  }

  mpcPtr_->getSolverPtr()->swapCostDesiredTrajectories(costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcModeSequenceCallback(const ocs2_msgs::mode_sequence::ConstPtr& msg) {
  mode_sequence_template_t modeSequenceTemplate;
  RosMsgConversions<STATE_DIM, INPUT_DIM>::readModeSequenceTemplateMsg(*msg, modeSequenceTemplate);
  mpcPtr_->setNewLogicRulesTemplate(modeSequenceTemplate);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::shutdownNode() {
#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("Shutting down workers ...");

  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) {
    publisherWorker_.join();
  }

  ROS_INFO_STREAM("All workers are shut down.");
#endif

  // shutdown publishers
  mpcPolicyPublisher_.shutdown();
  dummyPublisher_.shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::initializeNode(int argc, char* argv[]) {
  if (!nodeHandlerPtr_) {
    // display
    ROS_INFO_STREAM("MPC node is setting up ...");

    // setup ROS
    ::ros::init(argc, argv, robotName_ + "_mpc", ::ros::init_options::NoSigintHandler);
    signal(SIGINT, MPC_ROS_Interface::sigintHandler);

    // node handle
    nodeHandlerPtr_.reset(new ros::NodeHandle);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
std::shared_ptr<ros::NodeHandle>& MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::nodeHandlePtr() {
  return nodeHandlerPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::spin() {
  ROS_INFO_STREAM("Start spinning now ...");

  try {
    // Equivalent to ros::spin() + check if master is alive
    while (::ros::ok() && ::ros::master::check()) {
      ::ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }
  } catch (...) {
    // declaring that MPC is not updated anymore
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;
    mpcPolicyMsg.controllerIsUpdated = false;
    mpcPolicyPublisher_.publish(mpcPolicyMsg);
    throw;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::launchNodes(int argc, char* argv[]) {
  // initialize node
  initializeNode(argc, argv);

  // Observation subscriber
  mpcObservationSubscriber_ = nodeHandlerPtr_->subscribe(robotName_ + "_mpc_observation", 1, &MPC_ROS_Interface::mpcObservationCallback,
                                                         this, ::ros::TransportHints().udp());

  // Goal subscriber
  mpcTargetTrajectoriesSubscriber_ = nodeHandlerPtr_->subscribe(
      robotName_ + "_mpc_target", 1, &MPC_ROS_Interface::mpcTargetTrajectoriesCallback, this, ::ros::TransportHints().tcpNoDelay());

  // Logic rules template subscriber
  mpcModeSequenceSubscriber_ = nodeHandlerPtr_->subscribe(robotName_ + "_mpc_mode_sequence", 1, &MPC_ROS_Interface::mpcModeSequenceCallback,
                                                          this, ::ros::TransportHints().udp());

  // SLQ-MPC publisher
  mpcPolicyPublisher_ = nodeHandlerPtr_->advertise<ocs2_msgs::mpc_flattened_controller>(robotName_ + "_mpc_policy", 1, true);

  // dummy publisher
  dummyPublisher_ = nodeHandlerPtr_->advertise<ocs2_msgs::dummy>("ping", 1, true);

  // MPC reset service server
  mpcResetServiceServer_ = nodeHandlerPtr_->advertiseService(robotName_ + "_mpc_reset", &MPC_ROS_Interface::resetMpcCallback, this);

  // subscribe task listeners
  for (auto& taskListener : taskListenerArray_) {
    taskListener->subscribe(*nodeHandlerPtr_);
  }

  // display
#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("Publishing SLQ-MPC messages on a separate thread.");
#endif

  ROS_INFO_STREAM("MPC node is ready.");

  // spin
  spin();
}

}  // namespace ocs2
