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
MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::MPC_ROS_Interface(mpc_t& mpc, std::string robotName /*= "robot"*/)
    : mpc_(mpc),
      robotName_(std::move(robotName)),
      currentPrimalSolution_(new primal_solution_t()),
      primalSolutionBuffer_(new primal_solution_t()),
      currentCommand_(new command_data_t()),
      commandBuffer_(new command_data_t()) {
  set();
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
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::set() {
  terminateThread_ = false;
  readyToPublish_ = false;

  initialCall_ = false;
  resetRequestedEver_ = false;
  mpcTimer_.reset();

  // Start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::reset(const CostDesiredTrajectories& initCostDesiredTrajectories) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);

  mpc_.reset();

  initialCall_ = true;
  resetRequestedEver_ = true;

  mpc_.getSolverPtr()->setCostDesiredTrajectories(initCostDesiredTrajectories);
  costDesiredTrajectoriesBufferUpdated_ = false;

  mpcTimer_.reset();

  terminateThread_ = false;
  readyToPublish_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res) {
  if (static_cast<bool>(req.reset)) {
    CostDesiredTrajectories initCostDesiredTrajectories;
    ros_msg_conversions::readTargetTrajectoriesMsg(req.targetTrajectories, initCostDesiredTrajectories);
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
ocs2_msgs::mpc_flattened_controller MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::createMpcPolicyMsg(bool controllerIsUpdated,
                                                                                                const primal_solution_t& primalSolution,
                                                                                                const command_data_t& commandData) {
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;

  mpcPolicyMsg.controllerIsUpdated = controllerIsUpdated;

  ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_, mpcPolicyMsg.initObservation);
  ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcCostDesiredTrajectories_, mpcPolicyMsg.planTargetTrajectories);

  ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_, mpcPolicyMsg.modeSchedule);

  ControllerType controllerType = primalSolution.controllerPtr_->getType();

  // translate controllerType enum into message enum
  switch (controllerType) {
    case ControllerType::FEEDFORWARD: {
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    }
    case ControllerType::LINEAR: {
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    }
    default: {
      throw std::runtime_error("MPC_ROS_Interface: Unknown controller type.");
      break;
    }
  }

  // maximum length of the message
  size_t N = primalSolution.timeTrajectory_.size();

  mpcPolicyMsg.timeTrajectory.clear();
  mpcPolicyMsg.timeTrajectory.reserve(N);
  mpcPolicyMsg.stateTrajectory.clear();
  mpcPolicyMsg.stateTrajectory.reserve(N);
  mpcPolicyMsg.data.clear();
  mpcPolicyMsg.data.reserve(N);

  // time
  for (size_t k = 0; k < N; k++) {
    mpcPolicyMsg.timeTrajectory.emplace_back(primalSolution.timeTrajectory_[k]);
  }  // end of k loop

  // state
  for (size_t k = 0; k < N; k++) {
    ocs2_msgs::mpc_state mpcState;
    mpcState.value.resize(STATE_DIM);
    for (size_t j = 0; j < STATE_DIM; j++) {
      mpcState.value[j] = primalSolution.stateTrajectory_[k](j);
    }
    mpcPolicyMsg.stateTrajectory.emplace_back(mpcState);
  }  // end of k loop

  // input
  for (size_t k = 0; k < N; k++) {
    ocs2_msgs::mpc_input mpcInput;
    mpcInput.value.resize(INPUT_DIM);
    for (size_t j = 0; j < INPUT_DIM; j++) {
      mpcInput.value[j] = primalSolution.inputTrajectory_[k](j);
    }
    mpcPolicyMsg.inputTrajectory.emplace_back(mpcInput);
  }  // end of k loop

  // controller
  scalar_array_t timeTrajectoryTruncated;
  std::vector<std::vector<float>*> policyMsgDataPointers;
  policyMsgDataPointers.reserve(N);
  for (size_t k = 0; k < N; k++) {
    mpcPolicyMsg.data.emplace_back(ocs2_msgs::controller_data());

    policyMsgDataPointers.push_back(&mpcPolicyMsg.data.back().data);
    timeTrajectoryTruncated.push_back(primalSolution.timeTrajectory_[k]);
  }  // end of k loop
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

  return mpcPolicyMsg;
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

    {
      std::lock_guard<std::mutex> policyBufferLock(policyBufferMutex_);
      currentPrimalSolution_.swap(primalSolutionBuffer_);
      currentCommand_.swap(commandBuffer_);
    }

    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(true, *currentPrimalSolution_, *currentCommand_);

    // publish the message
    mpcPolicyPublisher_.publish(mpcPolicyMsg);

    readyToPublish_ = false;
    lk.unlock();
    msgReady_.notify_one();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::fillMpcOutputBuffers(system_observation_t mpcInitObservation) {
  // buffer policy mutex
  std::lock_guard<std::mutex> policyBufferLock(policyBufferMutex_);

  // get solution
  scalar_t finalTime = mpcInitObservation.time() + mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) {
    finalTime = mpc_.getSolverPtr()->getFinalTime();
  }
  mpc_.getSolverPtr()->getPrimalSolution(finalTime, primalSolutionBuffer_.get());

  // command
  commandBuffer_->mpcInitObservation_ = std::move(mpcInitObservation);
  commandBuffer_->mpcCostDesiredTrajectories_ = mpc_.getSolverPtr()->getCostDesiredTrajectories();
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
  ros_msg_conversions::readObservationMsg(*msg, currentObservation);

  // measure the delay in running MPC
  mpcTimer_.startTimer();

  // after each reset, perform user defined operation if specialized
  if (initialCall_) {
    initCall(currentObservation);
  }

  // Set latest cost desired trajectories
  if (costDesiredTrajectoriesBufferUpdated_) {
    std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
    mpc_.getSolverPtr()->swapCostDesiredTrajectories(costDesiredTrajectoriesBuffer_);
    costDesiredTrajectoriesBufferUpdated_ = false;

    if (mpc_.settings().debugPrint_) {
      std::cerr << "### The target position is updated to " << std::endl;
      mpc_.getSolverPtr()->getCostDesiredTrajectories().display();
    }
  }

  // run MPC
  bool controllerIsUpdated = mpc_.run(currentObservation.time(), currentObservation.state());
  if (!controllerIsUpdated) {
    return;
  }
  fillMpcOutputBuffers(currentObservation);

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

  // display
  if (mpc_.settings().debugPrint_) {
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

#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();

#else
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(true, *primalSolutionBuffer_, *commandBuffer_);
  mpcPolicyPublisher_.publish(mpcPolicyMsg);
#endif

#endif

  // set the initialCall flag to false
  initialCall_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcTargetTrajectoriesCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) {
  if (!mpc_.settings().recedingHorizon_) {
    throw std::runtime_error("Target trajectories can only be updated in receding horizon mode.");
  }

  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  ros_msg_conversions::readTargetTrajectoriesMsg(*msg, costDesiredTrajectoriesBuffer_);
  costDesiredTrajectoriesBufferUpdated_ = true;
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
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM>::launchNodes(ros::NodeHandle& nodeHandle) {
  ROS_INFO_STREAM("MPC node is setting up ...");

  // Observation subscriber
  mpcObservationSubscriber_ = nodeHandle.subscribe(robotName_ + "_mpc_observation", 1, &MPC_ROS_Interface::mpcObservationCallback, this,
                                                   ::ros::TransportHints().udp());

  // Goal subscriber
  mpcTargetTrajectoriesSubscriber_ = nodeHandle.subscribe(robotName_ + "_mpc_target", 1, &MPC_ROS_Interface::mpcTargetTrajectoriesCallback,
                                                          this, ::ros::TransportHints().tcpNoDelay());

  // MPC publisher
  mpcPolicyPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_flattened_controller>(robotName_ + "_mpc_policy", 1, true);

  // MPC reset service server
  mpcResetServiceServer_ = nodeHandle.advertiseService(robotName_ + "_mpc_reset", &MPC_ROS_Interface::resetMpcCallback, this);

  for (auto& module : synchronizedRosModules_) {
    module->subscribe(nodeHandle);
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
