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
MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::MRT_ROS_Interface(std::string robotName /*= "robot"*/,
                                                           std::shared_ptr<HybridLogicRules> logicRules /*= nullptr*/) {
  if (!logicRules) {
    logicRules = std::shared_ptr<NullLogicRules>(new NullLogicRules());
  }
  set(std::move(robotName), std::move(logicRules));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::~MRT_ROS_Interface() {
  shutdownNodes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::set(std::string robotName, std::shared_ptr<HybridLogicRules> logicRules) {
  logicMachinePtr_ = logic_machine_ptr_t(new logic_machine_t(std::move(logicRules)));

  robotName_ = std::move(robotName);

  // reset variables
  reset();

  // Start thread for publishing
#ifdef PUBLISH_THREAD
  // Close old thread if it is already running
  shutdownPublisher();
  reset();

  publisherWorker_ = std::thread(&MRT_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::sigintHandler(int sig) {
  ROS_INFO_STREAM("Shutting MRT node.");
  ::ros::shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::reset() {
  std::lock(policyMutexBuffer_, policyMutex_);
  std::lock_guard<std::mutex> lkUpdate(policyMutexBuffer_, std::adopt_lock);
  std::lock_guard<std::mutex> lkRead(policyMutex_, std::adopt_lock);

  messageHash_ = std::numeric_limits<size_t>::max();
  messageHashBuffer_ = 0;

  logicUpdated_ = false;
  policyUpdated_ = false;
  policyUpdatedBuffer_ = false;
  policyReceivedEver_ = false;

  terminateThread_ = false;
  readyToPublish_ = false;

  mpcLinInterpolateState_.setZero();

  eventTimes_.clear();
  eventTimesBuffer_.clear();
  subsystemsSequence_.clear();
  subsystemsSequenceBuffer_.clear();
  partitioningTimesUpdate(0.0, partitioningTimes_);
  partitioningTimesUpdate(0.0, partitioningTimesBuffer_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::messageHashValue(const system_observation_t& observation) const {
  return (observation.time() * 1.0e+6);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::partitioningTimesUpdate(const scalar_t& time, scalar_array_t& partitioningTimes) const {
  partitioningTimes.resize(2);
  partitioningTimes[0] = (policyReceivedEver_.load() == true) ? initPlanObservation_.time() : time;
  partitioningTimes[1] = std::numeric_limits<scalar_t>::max();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  policyReceivedEver_ = false;

  ocs2_comm_interfaces::reset resetSrv;
  resetSrv.request.reset = true;

  RosMsgConversions<STATE_DIM, INPUT_DIM>::CreateTargetTrajectoriesMsg(initCostDesiredTrajectories, resetSrv.request.targetTrajectories);

  if (mpcResetServiceClient_.waitForExistence()) {
    mpcResetServiceClient_.call(resetSrv);
    ROS_INFO_STREAM("MPC node is reset.");
  } else {
    ROS_ERROR_STREAM("Failed to call service to reset MPC.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::publishDummy() {
  ocs2_comm_interfaces::dummy msg;
  msg.ping = 1;
  dummyPublisher_.publish(msg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::publishObservation(const system_observation_t& currentObservation) {
#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

  // create the message
  ros_msg_conversions_t::CreateObservationMsg(currentObservation, mpcObservationMsg_);

  // publish the current observation
#ifdef PUBLISH_THREAD
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();
#else
  mpcObservationPublisher_.publish(mpcObservationMsg_);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::publisherWorkerThread() {
  while (terminateThread_ == false) {
    std::unique_lock<std::mutex> lk(publisherMutex_);

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    if (terminateThread_ == true) {
      break;
    }

    mpcObservationMsgBuffer_ = std::move(mpcObservationMsg_);

    readyToPublish_ = false;

    lk.unlock();
    msgReady_.notify_one();

    // publish the message
    mpcObservationPublisher_.publish(mpcObservationMsgBuffer_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcPolicyCallback(const ocs2_comm_interfaces::mpc_flattened_controller::ConstPtr& msg) {
  //	std::cout << "\t Plan is received at time: " << msg->initObservation.time << std::endl;

  std::lock_guard<std::mutex> lk(policyMutexBuffer_);

  // if the policy is not updated
  if (!static_cast<bool>(msg->controllerIsUpdated)) {
    mpcInitObservationBuffer_ = system_observation_t();
    mpcCostDesiredTrajectoriesBuffer_.clear();
    policyUpdatedBuffer_ = false;
    eventTimesBuffer_.clear();
    subsystemsSequenceBuffer_.clear();
    mpcTimeTrajectoryBuffer_.clear();
    mpcStateTrajectoryBuffer_.clear();
    mpcControllerBufferPtr_.reset(nullptr);

    // It is important that the buffer message's hash get updated at the very last
    // since it will signal the updatePolicy method to swap buffer.
    // Although data is protected from racing however it will cause unnecessary delay
    messageHashBuffer_ = messageHashValue(mpcInitObservationBuffer_);

    return;
  }

  ros_msg_conversions_t::ReadObservationMsg(msg->initObservation, mpcInitObservationBuffer_);

  ros_msg_conversions_t::ReadTargetTrajectoriesMsg(msg->planTargetTrajectories, mpcCostDesiredTrajectoriesBuffer_);

  policyUpdatedBuffer_ = msg->controllerIsUpdated;

  ros_msg_conversions_t::ReadModeSequenceMsg(msg->modeSequence, eventTimesBuffer_, subsystemsSequenceBuffer_);

  const scalar_t partitionInitMargin = 1e-1;
  partitioningTimesUpdate(mpcInitObservationBuffer_.time() - partitionInitMargin, partitioningTimesBuffer_);

  const size_t N = msg->timeTrajectory.size();
  mpcTimeTrajectoryBuffer_.clear();
  mpcTimeTrajectoryBuffer_.reserve(N);
  mpcStateTrajectoryBuffer_.clear();
  mpcStateTrajectoryBuffer_.reserve(N);

  for (size_t i = 0; i < N; i++) {
    // time
    mpcTimeTrajectoryBuffer_.push_back(msg->timeTrajectory[i]);
    // state
    mpcStateTrajectoryBuffer_.push_back(
        Eigen::Map<const Eigen::Matrix<float, STATE_DIM, 1>>(msg->stateTrajectory[i].value.data(), STATE_DIM).template cast<scalar_t>());
  }  // end of i loop

  // instantiate the correct control
  switch (msg->controllerType) {
    case ocs2_comm_interfaces::mpc_flattened_controller::CONTROLLER_FEEDFORWARD: {
      using controller_t = FeedforwardController<STATE_DIM, INPUT_DIM>;
      mpcControllerBufferPtr_.reset(new controller_t());
      break;
    }
    case ocs2_comm_interfaces::mpc_flattened_controller::CONTROLLER_LINEAR: {
      using controller_t = LinearController<STATE_DIM, INPUT_DIM>;
      mpcControllerBufferPtr_.reset(new controller_t());
      break;
    }
    default:
      throw std::runtime_error("MRT_ROS_Interface::mpcPolicyCallback -- Unknown controllerType");
  }

  // check data size
  if (msg->data.size() != N) {
    throw std::runtime_error("Data has the wrong length");
  }

  std::vector<std::vector<float> const*> controllerDataPtrArray(N, nullptr);
  for (int i = 0; i < N; i++) {
    controllerDataPtrArray[i] = &(msg->data[i].data);
  }

  // load the message data into controller
  mpcControllerBufferPtr_->unFlatten(mpcTimeTrajectoryBuffer_, controllerDataPtrArray);

  // customized adjustment
  modifyBufferPolicy(mpcInitObservationBuffer_, *mpcControllerBufferPtr_, mpcTimeTrajectoryBuffer_, mpcStateTrajectoryBuffer_,
                     eventTimesBuffer_, subsystemsSequenceBuffer_);

  if (policyReceivedEver_.load() == false && policyUpdatedBuffer_ == true) {
    policyReceivedEver_ = true;
    initPlanObservation_ = mpcInitObservationBuffer_;
    initCall(initPlanObservation_);
  }

  // It is important that the buffer message's hash get updated at the very last
  // since it will signal the updatePolicy method to swap buffer.
  // Although data is protected from racing however it will cause unnecessary delay
  messageHashBuffer_ = messageHashValue(mpcInitObservationBuffer_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::updatePolicy() {
  std::lock(policyMutexBuffer_, policyMutex_);
  std::lock_guard<std::mutex> lkUpdate(policyMutexBuffer_, std::adopt_lock);
  std::lock_guard<std::mutex> lkRead(policyMutex_, std::adopt_lock);

  if (messageHash_ == messageHashBuffer_ || policyUpdatedBuffer_ == false) {
    return false;
  }

  // should not be swapped
  messageHash_ = messageHashBuffer_;

  mpcInitObservation_.swap(mpcInitObservationBuffer_);

  mpcCostDesiredTrajectories_.swap(mpcCostDesiredTrajectoriesBuffer_);

  policyUpdated_ = policyUpdatedBuffer_;

  // check whether logic rules needs to be updated
  logicUpdated_ = false;
  if (subsystemsSequence_ != subsystemsSequenceBuffer_) {
    subsystemsSequence_.swap(subsystemsSequenceBuffer_);
    logicUpdated_ = true;
  }
  if (eventTimes_ != eventTimesBuffer_) {
    eventTimes_.swap(eventTimesBuffer_);
    logicUpdated_ = true;
  }
  if (partitioningTimes_ != partitioningTimesBuffer_) {
    partitioningTimes_.swap(partitioningTimesBuffer_);
    logicUpdated_ = true;
  }

  // update logic rules
  if (logicUpdated_ == true) {
    // set mode sequence
    logicMachinePtr_->getLogicRulesPtr()->setModeSequence(subsystemsSequence_, eventTimes_);
    // Tell logicMachine that logicRules are modified
    logicMachinePtr_->logicRulesUpdated();
    // update logicMachine
    logicMachinePtr_->updateLogicRules(partitioningTimes_);

    // function for finding active subsystem
    const size_t partitionIndex = 0;  // we assume only one partition
    findActiveSubsystemFnc_ = std::move(logicMachinePtr_->getHandleToFindActiveEventCounter(partitionIndex));
  }

  mpcTimeTrajectory_.swap(mpcTimeTrajectoryBuffer_);
  mpcStateTrajectory_.swap(mpcStateTrajectoryBuffer_);
  mpcLinInterpolateState_.setData(&mpcTimeTrajectory_, &mpcStateTrajectory_);

  mpcControllerPtr_.swap(mpcControllerBufferPtr_);

  modifyPolicy(logicUpdated_, *mpcControllerPtr_, mpcTimeTrajectory_, mpcStateTrajectory_, eventTimes_, subsystemsSequence_);

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcIsTerminated() const {
  std::lock_guard<std::mutex> lk(policyMutex_);
  return !policyUpdated_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::initialPolicyReceived() const {
  return policyReceivedEver_.load();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::cost_desired_trajectories_t&
MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcCostDesiredTrajectories() const {
  std::lock_guard<std::mutex> lk(policyMutex_);
  return mpcCostDesiredTrajectories_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::initRollout(const controlled_system_base_t& controlSystemBase,
                                                          const Rollout_Settings& rolloutSettings) {
  rolloutPtr_.reset(new time_triggered_rollout_t(controlSystemBase, rolloutSettings, "mrt"));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::evaluatePolicy(const scalar_t& currentTime, const state_vector_t& currentState,
                                                             state_vector_t& mpcState, input_vector_t& mpcInput, size_t& subsystem) {
  if (currentTime > mpcTimeTrajectory_.back()) {
    ROS_WARN_STREAM("The requested currentTime is greater than the received plan: " + std::to_string(currentTime) + ">" +
                    std::to_string(mpcTimeTrajectory_.back()));
  }

  mpcInput = mpcControllerPtr_->computeInput(currentTime, currentState);
  mpcLinInterpolateState_.interpolate(currentTime, mpcState);

  size_t index = findActiveSubsystemFnc_(currentTime);
  subsystem = logicMachinePtr_->getLogicRulesPtr()->subsystemsSequence().at(index);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::rolloutPolicy(const scalar_t& currentTime, const state_vector_t& currentState,
                                                            const scalar_t& timeStep, state_vector_t& mpcState, input_vector_t& mpcInput,
                                                            size_t& subsystem) {
  if (currentTime > mpcTimeTrajectory_.back()) {
    ROS_WARN_STREAM("The requested currentTime is greater than the received plan: " + std::to_string(currentTime) + ">" +
                    std::to_string(mpcTimeTrajectory_.back()));
  }

  if (!rolloutPtr_) {
    throw std::runtime_error("MRT_ROS_interface: rolloutPtr is not initialized, call initRollout first.");
  }

  const size_t activePartitionIndex = 0;  // there is only one partition.
  scalar_t finalTime = currentTime + timeStep;
  scalar_array_t timeTrajectory;
  size_array_t eventsPastTheEndIndeces;
  state_vector_array_t stateTrajectory;
  input_vector_array_t inputTrajectory;

  // perform a rollout
  if (policyUpdated_ == true) {
    rolloutPtr_->run(activePartitionIndex, currentTime, currentState, finalTime, mpcControllerPtr_.get(), *logicMachinePtr_, timeTrajectory,
                     eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);
  } else {
    throw std::runtime_error("MRT_ROS_interface: policy should be updated before rollout.");
  }

  mpcState = stateTrajectory.back();
  mpcInput = inputTrajectory.back();

  size_t index = findActiveSubsystemFnc_(finalTime);
  subsystem = logicMachinePtr_->getLogicRulesPtr()->subsystemsSequence().at(index);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::shutdownNodes() {
#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("Shutting down workers ...");

  shutdownPublisher();

  ROS_INFO_STREAM("All workers are shut down.");
#endif

  // clean up callback queue
  mrtCallbackQueue_.clear();
  mpcPolicySubscriber_.shutdown();

  // shutdown publishers
  dummyPublisher_.shutdown();
  mpcObservationPublisher_.shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::shutdownPublisher() {
  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) {
    publisherWorker_.join();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
::ros::NodeHandlePtr& MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::nodeHandle() {
  return mrtRosNodeHandlePtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::spinMRT() {
  mrtCallbackQueue_.callOne();
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::launchNodes(int argc, char* argv[]) {
  reset();

  // display
  ROS_INFO_STREAM("MRT node is setting up ...");

  // setup ROS
  ::ros::init(argc, argv, robotName_ + "_mrt", ::ros::init_options::NoSigintHandler);
  signal(SIGINT, MRT_ROS_Interface::sigintHandler);

  mrtRosNodeHandlePtr_.reset(new ::ros::NodeHandle);
  mrtRosNodeHandlePtr_->setCallbackQueue(&mrtCallbackQueue_);

  // Observation publisher
  mpcObservationPublisher_ = mrtRosNodeHandlePtr_->advertise<ocs2_comm_interfaces::mpc_observation>(robotName_ + "_mpc_observation", 1);

  // SLQ-MPC subscriber
  mpcPolicySubscriber_ = mrtRosNodeHandlePtr_->subscribe(robotName_ + "_mpc_policy", 1, &MRT_ROS_Interface::mpcPolicyCallback, this,
                                                         ::ros::TransportHints().tcpNoDelay());

  // dummy publisher
  dummyPublisher_ = mrtRosNodeHandlePtr_->advertise<ocs2_comm_interfaces::dummy>("ping", 1, true);

  // MPC reset service client
  mpcResetServiceClient_ = mrtRosNodeHandlePtr_->serviceClient<ocs2_comm_interfaces::reset>(robotName_ + "_mpc_reset");

  // display
#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("Publishing MRT messages on a separate thread.");
#endif

  ROS_INFO_STREAM("MRT node is ready.");

  spinMRT();
}

}  // namespace ocs2
