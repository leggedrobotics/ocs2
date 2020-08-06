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

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_ROS_Interface::MPC_ROS_Interface(MPC_BASE& mpc, std::string robotName /*= "robot"*/)
    : mpc_(mpc),
      robotName_(std::move(robotName)),
      currentPrimalSolution_(new PrimalSolution()),
      primalSolutionBuffer_(new PrimalSolution()),
      currentCommand_(new CommandData()),
      commandBuffer_(new CommandData()) {
  set();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_ROS_Interface::~MPC_ROS_Interface() {
  shutdownNode();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::set() {
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
void MPC_ROS_Interface::reset(const CostDesiredTrajectories& initCostDesiredTrajectories) {
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
bool MPC_ROS_Interface::resetMpcCallback(ocs2_msgs::reset::Request& req, ocs2_msgs::reset::Response& res) {
  if (static_cast<bool>(req.reset)) {
    CostDesiredTrajectories initCostDesiredTrajectories;
    ros_msg_conversions::readTargetTrajectoriesMsg(req.targetTrajectories, initCostDesiredTrajectories);
    reset(initCostDesiredTrajectories);

    res.done = static_cast<uint8_t>(true);

    std::cerr << "\n#####################################################"
              << "\n#####################################################"
              << "\n#################  MPC is reset.  ###################"
              << "\n#####################################################"
              << "\n#####################################################\n";

  } else {
    ROS_WARN_STREAM("Ineffective reset request.");
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::mpc_flattened_controller MPC_ROS_Interface::createMpcPolicyMsg(bool controllerIsUpdated, const PrimalSolution& primalSolution,
                                                                          const CommandData& commandData) {
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;

  mpcPolicyMsg.controllerIsUpdated = static_cast<uint8_t>(controllerIsUpdated);

  ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_, mpcPolicyMsg.initObservation);
  ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcCostDesiredTrajectories_, mpcPolicyMsg.planTargetTrajectories);
  ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_, mpcPolicyMsg.modeSchedule);

  switch (primalSolution.controllerPtr_->getType()) {
    case ControllerType::FEEDFORWARD:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    case ControllerType::LINEAR:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    default:
      throw std::runtime_error("MPC_ROS_Interface::createMpcPolicyMsg: Unknown ControllerType");
  }

  // maximum length of the message
  const size_t N = primalSolution.timeTrajectory_.size();

  mpcPolicyMsg.timeTrajectory.clear();
  mpcPolicyMsg.timeTrajectory.reserve(N);
  mpcPolicyMsg.stateTrajectory.clear();
  mpcPolicyMsg.stateTrajectory.reserve(N);
  mpcPolicyMsg.data.clear();
  mpcPolicyMsg.data.reserve(N);

  // time
  for (auto t : primalSolution.timeTrajectory_) {
    mpcPolicyMsg.timeTrajectory.emplace_back(t);
  }  // end of k loop

  // state
  for (size_t k = 0; k < N; k++) {
    ocs2_msgs::mpc_state mpcState;
    mpcState.value.resize(primalSolution.stateTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++) {
      mpcState.value[j] = primalSolution.stateTrajectory_[k](j);
    }
    mpcPolicyMsg.stateTrajectory.emplace_back(mpcState);
  }  // end of k loop

  // input
  for (size_t k = 0; k < N; k++) {
    ocs2_msgs::mpc_input mpcInput;
    mpcInput.value.resize(primalSolution.inputTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++) {
      mpcInput.value[j] = primalSolution.inputTrajectory_[k](j);
    }
    mpcPolicyMsg.inputTrajectory.emplace_back(mpcInput);
  }  // end of k loop

  // controller
  scalar_array_t timeTrajectoryTruncated;
  std::vector<std::vector<float>*> policyMsgDataPointers;
  policyMsgDataPointers.reserve(N);
  for (auto t : primalSolution.timeTrajectory_) {
    mpcPolicyMsg.data.emplace_back(ocs2_msgs::controller_data());

    policyMsgDataPointers.push_back(&mpcPolicyMsg.data.back().data);
    timeTrajectoryTruncated.push_back(t);
  }  // end of k loop

  // serialize controller into data buffer
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

  return mpcPolicyMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::publisherWorkerThread() {
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
void MPC_ROS_Interface::fillMpcOutputBuffers(SystemObservation mpcInitObservation) {
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
void MPC_ROS_Interface::mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);

  if (!resetRequestedEver_.load()) {
    ROS_WARN_STREAM("MPC should be reset first. Either call MPC_ROS_Interface::reset() or use the reset service.");
    return;
  }

  // current time, state, input, and subsystem
  SystemObservation currentObservation;
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
      std::cerr << "### The target position is updated to \n";
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
    std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!\n";
  }

  // display
  if (mpc_.settings().debugPrint_) {
    std::cerr << '\n';
    std::cerr << "\n### MPC_ROS Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();

#else
  ocs2_msgs::mpc_flattened_controller mpcPolicyMsg = createMpcPolicyMsg(true, *primalSolutionBuffer_, *commandBuffer_);
  mpcPolicyPublisher_.publish(mpcPolicyMsg);
#endif

  // set the initialCall flag to false
  initialCall_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::mpcTargetTrajectoriesCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  ros_msg_conversions::readTargetTrajectoriesMsg(*msg, costDesiredTrajectoriesBuffer_);
  costDesiredTrajectoriesBufferUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::shutdownNode() {
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
void MPC_ROS_Interface::spin() {
  ROS_INFO_STREAM("Start spinning now ...");

#ifdef NDEBUG
  try {
#endif
    // Equivalent to ros::spin() + check if master is alive
    while (::ros::ok() && ::ros::master::check()) {
      ::ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }
#ifdef NDEBUG
  } catch (...) {
    // declaring that MPC is not updated anymore
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;
    mpcPolicyMsg.controllerIsUpdated = static_cast<uint8_t>(false);
    mpcPolicyPublisher_.publish(mpcPolicyMsg);
    throw;
  }
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::launchNodes(ros::NodeHandle& nodeHandle) {
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
