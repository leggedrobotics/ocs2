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
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MRT_ROS_Interface(
    const LOGIC_RULES_T &logicRules,
    const bool &useFeedforwardPolicy /*= true*/,
    const std::string &robotName /*= "robot"*/)

    : logicMachinePtr_(new logic_machine_t(logicRules)),
      useFeedforwardPolicy_(useFeedforwardPolicy),
      robotName_(robotName) {
  // reset variables
  reset();

  // Start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MRT_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~MRT_ROS_Interface() {

  shutdownNodes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::set(
    const LOGIC_RULES_T &logicRules,
    const bool &useFeedforwardPolicy /*= true*/,
    const std::string &robotName /*= "robot"*/) {

  logicMachinePtr_ = logic_machine_ptr_t(new logic_machine_t(logicRules));

  useFeedforwardPolicy_ = useFeedforwardPolicy;

  robotName_ = robotName;

  // reset variables
  reset();

  // Start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MRT_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::sigintHandler(int sig) {

  ROS_INFO_STREAM("Shutting MRT node.");
  ::ros::shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset() {

  messageHash_ = std::numeric_limits<size_t>::max();
  messageHashBuffer_ = 0;

  logicUpdated_ = false;
  policyUpdated_ = false;
  policyUpdatedBuffer_ = false;
  policyReceivedEver_ = false;

  terminateThread_ = false;
  readyToPublish_ = false;

  feedforwardGeneratedWithRollout_ = false;

  mpcLinInterpolateState_.setZero();
  mpcLinInterpolateInput_.setZero();
  mpcLinInterpolateUff_.setZero();
  mpcLinInterpolateK_.setZero();

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
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::messageHashValue(
    const system_observation_t &observation) const {

  return observation.time() * 1.0e+6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::partitioningTimesUpdate(
    const scalar_t &time,
    scalar_array_t &partitioningTimes) const {

  partitioningTimes.resize(2);
  partitioningTimes[0] = (policyReceivedEver_ == true) ? initPlanObservation_.time() : time;
  partitioningTimes[1] = std::numeric_limits<scalar_t>::max();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::resetMpcNode() {

  policyReceivedEver_ = false;

  ocs2_comm_interfaces::reset resetSrv;
  resetSrv.request.reset = true;

  if (mpcResetServiceClient_.call(resetSrv)) {
    ROS_INFO_STREAM("Waiting for MPC node to reset ...");
    mpcResetServiceClient_.waitForExistence();
    ROS_INFO_STREAM("MPC node is reset.");

  } else {
    ROS_ERROR_STREAM("Failed to call service to reset MPC.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::publishDummy() {

  ocs2_comm_interfaces::dummy msg;
  msg.ping = 1;
  dummyPublisher_.publish(msg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::publishObservation(
    const system_observation_t &currentObservation) {

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
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::publisherWorkerThread() {

  while (terminateThread_ == false) {

    std::unique_lock<std::mutex> lk(publisherMutex_);

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    if (terminateThread_ == true) break;

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
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::mpcFeedforwardPolicyCallback(
    const ocs2_comm_interfaces::mpc_feedforward_policy::ConstPtr &msg) {

//	std::cout << "\t Plan is recieved at time: " << msg->initObservation.time << std::endl;

  std::unique_lock<std::mutex> lk(subscriberMutex_);

  ros_msg_conversions_t::ReadObservationMsg(msg->initObservation,
                                            mpcInitObservationBuffer_);

  ros_msg_conversions_t::ReadTargetTrajectoriesMsg(msg->planTargetTrajectories,
                                                   mpcCostDesiredTrajectoriesBuffer_);

  policyUpdatedBuffer_ = msg->controllerIsUpdated;

  ros_msg_conversions_t::ReadModeSequenceMsg(msg->modeSequence,
                                             eventTimesBuffer_, subsystemsSequenceBuffer_);

  // time partitioning
//	partitioningTimesBuffer_ = scalar_array_t {
//		msg->timeTrajectory.front(), msg->timeTrajectory.back()};
  partitioningTimesUpdate(msg->timeTrajectory.front(), partitioningTimesBuffer_);

  const size_t N = msg->timeTrajectory.size();
  mpcTimeTrajectoryBuffer_.clear();
  mpcTimeTrajectoryBuffer_.reserve(N);
  mpcStateTrajectoryBuffer_.clear();
  mpcStateTrajectoryBuffer_.reserve(N);
  mpcInputTrajectoryBuffer_.clear();
  mpcInputTrajectoryBuffer_.reserve(N);

  for (size_t i = 0; i < N; i++) {
    // time
    mpcTimeTrajectoryBuffer_.push_back(msg->timeTrajectory[i]);
    // state
    mpcStateTrajectoryBuffer_.push_back(Eigen::Map<const Eigen::Matrix<float, STATE_DIM, 1>>(
        msg->stateTrajectory[i].value.data(), STATE_DIM).template cast<scalar_t>());
    // input
    mpcInputTrajectoryBuffer_.push_back(Eigen::Map<const Eigen::Matrix<float, INPUT_DIM, 1>>(
        msg->inputTrajectory[i].value.data(), INPUT_DIM).template cast<scalar_t>());
  } // end of i loop

  // customized adjustment
  modifyBufferFeedforwardPolicy(mpcInitObservationBuffer_,
                                mpcTimeTrajectoryBuffer_, mpcStateTrajectoryBuffer_, mpcInputTrajectoryBuffer_,
                                eventTimesBuffer_, subsystemsSequenceBuffer_);

  if (policyReceivedEver_ == false && policyUpdatedBuffer_ == true) {
    policyReceivedEver_ = true;
    initPlanObservation_ = mpcInitObservationBuffer_;
    initCall(initPlanObservation_);
  }

  lk.unlock();

  // It is important that the buffer message's hash get updated at the very last
  // since it will signal the updatePolicy method to swap buffer.
  // Although data is protected from racing however it will cause unnecessary delay
  messageHashBuffer_ = messageHashValue(mpcInitObservationBuffer_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::mpcFeedbackPolicyCallback(
    const ocs2_comm_interfaces::mpc_feedback_policy::ConstPtr &msg) {

//	std::cout << "\t Plan is received at time: " << msg->initObservation.time << std::endl;

  std::unique_lock<std::mutex> lk(subscriberMutex_);

  ros_msg_conversions_t::ReadObservationMsg(msg->initObservation,
                                            mpcInitObservationBuffer_);

  ros_msg_conversions_t::ReadTargetTrajectoriesMsg(msg->planTargetTrajectories,
                                                   mpcCostDesiredTrajectoriesBuffer_);

  policyUpdatedBuffer_ = msg->controllerIsUpdated;

  ros_msg_conversions_t::ReadModeSequenceMsg(msg->modeSequence,
                                             eventTimesBuffer_, subsystemsSequenceBuffer_);

  // time partitioning
//	partitioningTimesBuffer_ = scalar_array_t {
//		msg->slqControllerTrajectory.front().timeStamp, msg->slqControllerTrajectory.back().timeStamp};
  partitioningTimesUpdate(msg->slqControllerTrajectory.front().timeStamp, partitioningTimesBuffer_);

  const size_t N = msg->slqControllerTrajectory.size();
  mpcControllerBuffer_.clear();
  mpcControllerBuffer_.time_.reserve(N);
  mpcControllerBuffer_.uff_.reserve(N);
  mpcControllerBuffer_.k_.reserve(N);

  input_vector_t uffTemp;
  input_state_matrix_t kTemp;
  for (size_t i = 0; i < N; i++) {

    uffTemp = Eigen::Map<const Eigen::Matrix<float, INPUT_DIM, 1>>(
        msg->slqControllerTrajectory[i].uff.value.data(), INPUT_DIM).template cast<scalar_t>();

    // covert from array to Eigen types
    for (size_t p = 0; p < INPUT_DIM; p++)
      for (size_t q = 0; q < STATE_DIM; q++)
        kTemp(p, q) = msg->slqControllerTrajectory[i].gainMatrix[p * STATE_DIM + q];

    mpcControllerBuffer_.time_.push_back(msg->slqControllerTrajectory[i].timeStamp);
    mpcControllerBuffer_.uff_.push_back(uffTemp);
    mpcControllerBuffer_.k_.push_back(kTemp);
  }  //end i loop

  // customized adjustment
  modifyBufferFeedbackPolicy(mpcInitObservationBuffer_,
                             mpcControllerBuffer_,
                             eventTimesBuffer_, subsystemsSequenceBuffer_);

  if (policyReceivedEver_ == false && policyUpdatedBuffer_ == true) {
    policyReceivedEver_ = true;
    initPlanObservation_ = mpcInitObservationBuffer_;
    initCall(initPlanObservation_);
  }

  lk.unlock();

  // It is important that the buffer message hash get updated the very last
  // since it will signal the updatePolicy method to swap buffer.
  // Although data is protected from racing however it will cause necessary delay
  messageHashBuffer_ = messageHashValue(mpcInitObservationBuffer_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::updatePolicy() {

  if (messageHash_ == messageHashBuffer_.load()) {
    return false;
  } else if (policyUpdatedBuffer_ == false) {
    return false;
  }

  std::unique_lock<std::mutex> lk(subscriberMutex_);

  // should not be swapped
  messageHash_ = messageHashBuffer_.load();

  mpcInitObservation_.swap(mpcInitObservationBuffer_);

  mpcCostDesiredTrajectories_.swap(mpcCostDesiredTrajectoriesBuffer_);

  policyUpdated_ = policyUpdatedBuffer_;

  logicUpdated_ = false;

  feedforwardGeneratedWithRollout_ = false;

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

  //
  if (logicUpdated_ == true) {
    // set mode sequence
    logicMachinePtr_->getLogicRulesPtr()->setModeSequence(subsystemsSequence_, eventTimes_);
    // Tell logicMachine that logicRules are modified
    logicMachinePtr_->logicRulesUpdated();
    // update logicMachine
    logicMachinePtr_->updateLogicRules(partitioningTimes_);

    // function for finding active subsystem
    const size_t partitionIndex = 0; // we assume only one partition
    findActiveSubsystemFnc_ = std::move(
        logicMachinePtr_->getHandleToFindActiveEventCounter(partitionIndex));
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

    loadModifiedFeedforwardPolicy(logicUpdated_, policyUpdated_,
                                  mpcTimeTrajectory_, mpcStateTrajectory_, mpcInputTrajectory_,
                                  eventTimes_, subsystemsSequence_);

  } else {
    mpcController_.swap(mpcControllerBuffer_);

    mpcLinInterpolateUff_.reset();
    mpcLinInterpolateUff_.setTimeStamp(&mpcController_.time_);
    mpcLinInterpolateUff_.setData(&mpcController_.uff_);

    mpcLinInterpolateK_.reset();
    mpcLinInterpolateK_.setTimeStamp(&mpcController_.time_);
    mpcLinInterpolateK_.setData(&mpcController_.k_);

    loadModifiedFeedbackPolicy(logicUpdated_, policyUpdated_,
                               mpcController_,
                               eventTimes_, subsystemsSequence_);
  }

  lk.unlock();

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::initialPolicyReceived() const {

  return policyReceivedEver_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::cost_desired_trajectories_t &
MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::mpcCostDesiredTrajectories() const {

  return mpcCostDesiredTrajectories_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::evaluateFeedforwardPolicy(
    const scalar_t &time,
    state_vector_t &mpcState,
    input_vector_t &mpcInput,
    size_t &subsystem) {

  if (!useFeedforwardPolicy_ && !feedforwardGeneratedWithRollout_)
    throw std::runtime_error("The MRT is set to receive the feedforward policy.");

  if (time > mpcTimeTrajectory_.back())
    ROS_WARN_STREAM("The requested time is greater than the received plan: "
                        + std::to_string(time) + ">" + std::to_string(mpcTimeTrajectory_.back()));

  mpcLinInterpolateState_.interpolate(time, mpcState);
  int greatestLessTimeStampIndex = mpcLinInterpolateState_.getGreatestLessTimeStampIndex();
  mpcLinInterpolateInput_.interpolate(time, mpcInput, greatestLessTimeStampIndex);

  size_t
  index = findActiveSubsystemFnc_(time);
  subsystem = logicMachinePtr_->getLogicRulesPtr()->subsystemsSequence().at(index);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::evaluateFeedbackPolicy(
    const scalar_t &time,
    input_vector_t &mpcUff,
    input_state_matrix_t &mpcGain,
    size_t &subsystem) {

  if (useFeedforwardPolicy_ == true)
    throw std::runtime_error("The MRT is set to receive the feedback policy.");

  if (time > mpcController_.time_.back())
    ROS_WARN_STREAM("The requested time is greater than the received plan: "
                        + std::to_string(time) + ">" + std::to_string(mpcController_.time_.back()));

  mpcLinInterpolateUff_.interpolate(time, mpcUff);
  int greatestLessTimeStampIndex = mpcLinInterpolateUff_.getGreatestLessTimeStampIndex();
  mpcLinInterpolateK_.interpolate(time, mpcGain, greatestLessTimeStampIndex);

  size_t
  index = findActiveSubsystemFnc_(time);
  subsystem = logicMachinePtr_->getLogicRulesPtr()->subsystemsSequence().at(index);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM,
                       INPUT_DIM,
                       LOGIC_RULES_T>::initRollout(controlled_system_base_ptr_t controlSystemBasePtr,
                                                   const Rollout_Settings &rolloutSettings) {

  rolloutPtr_ = rollout_base_ptr_t(new time_triggered_rollout_t(*controlSystemBasePtr, rolloutSettings, "mrt"));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rolloutFeedbackPolicy(scalar_t t0,
                                                                                   const state_vector_t &initState,
                                                                                   scalar_t rollout_time) {
  size_t
  activePartitionIndex = findActiveIntervalIndex(partitioningTimes_, t0, 0);

  scalar_t final_time = t0 + rollout_time;
  size_array_t eventsPastTheEndIndeces;

  // Perform rollout
  if (rolloutPtr_) {
    if (policyUpdated_) {
      rolloutPtr_->run(activePartitionIndex,
                       t0,
                       initState,
                       final_time,
                       mpcController_,
                       *logicMachinePtr_,
                       mpcTimeTrajectory_,
                       eventsPastTheEndIndeces,
                       mpcStateTrajectory_,
                       mpcInputTrajectory_);
    } else {
      throw std::runtime_error("MRT_ROS_interface: policy not updated before rollout");
    }
  } else {
    throw std::runtime_error("MRT_ROS_interface: rolloutPtr not initialized");
  }

  // Set rollout to be the mpc feedforward trajectory
  mpcLinInterpolateState_.reset();
  mpcLinInterpolateState_.setTimeStamp(&mpcTimeTrajectory_);
  mpcLinInterpolateState_.setData(&mpcStateTrajectory_);

  mpcLinInterpolateInput_.reset();
  mpcLinInterpolateInput_.setTimeStamp(&mpcTimeTrajectory_);
  mpcLinInterpolateInput_.setData(&mpcInputTrajectory_);

  loadModifiedFeedforwardPolicy(logicUpdated_, policyUpdated_,
                                mpcTimeTrajectory_, mpcStateTrajectory_, mpcInputTrajectory_,
                                eventTimes_, subsystemsSequence_);

  feedforwardGeneratedWithRollout_ = true;
}



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::shutdownNodes() {

#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("Shutting down workers ...");

  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable())
    publisherWorker_.join();

  ROS_INFO_STREAM("All workers are shut down.");
#endif

  // Clean up Callback queue
  mrtCallbackQueue_.clear();
  mpcFeedforwardPolicySubscriber_.shutdown();
  mpcFeedbackPolicySubscriber_.shutdown();

  // shutdown publishers
  dummyPublisher_.shutdown();
  mpcObservationPublisher_.shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
::ros::NodeHandlePtr &MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::nodeHandle() {

  return mrtRosNodeHandlePtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::launchNodes(int argc, char *argv[]) {

  reset();

  // display
  ROS_INFO_STREAM("MRT node is setting up ...");

  // setup ROS
  ::ros::init(argc, argv, robotName_ + "_mrt", ::ros::init_options::NoSigintHandler);
  signal(SIGINT, MRT_ROS_Interface::sigintHandler);

  mrtRosNodeHandlePtr_.reset(new ::ros::NodeHandle);
  mrtRosNodeHandlePtr_->setCallbackQueue(&mrtCallbackQueue_);

  // Observation publisher
  mpcObservationPublisher_ = mrtRosNodeHandlePtr_->advertise<ocs2_comm_interfaces::mpc_observation>(
      robotName_ + "_mpc_observation", 1);

  // SLQ-MPC subscriber
  if (useFeedforwardPolicy_ == true) {
    mpcFeedforwardPolicySubscriber_ = mrtRosNodeHandlePtr_->subscribe(
        robotName_ + "_mpc_ff_policy",
        1,
        &MRT_ROS_Interface::mpcFeedforwardPolicyCallback,
        this, ::ros::TransportHints().udp());
  } else {
    mpcFeedbackPolicySubscriber_ = mrtRosNodeHandlePtr_->subscribe(
        robotName_ + "_mpc_fb_policy",
        1,
        &MRT_ROS_Interface::mpcFeedbackPolicyCallback, this,
        ::ros::TransportHints().udp());
  }

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

template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::spinMRT() {
  mrtCallbackQueue_.callOne();
};

} // namespace ocs2

