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

#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/control/PiController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MRT_ROS_Interface(
    const LOGIC_RULES_T &logicRules,
    const std::string &robotName /*= "robot"*/)

    : logicMachinePtr_(new logic_machine_t(logicRules))
    , feedforwardGeneratedWithRollout_(false)
    , robotName_(robotName) {
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
    const std::string &robotName /*= "robot"*/) {

  logicMachinePtr_ = logic_machine_ptr_t(new logic_machine_t(logicRules));

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
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::mpcPolicyCallback(
    const ocs2_comm_interfaces::mpc_flattened_controller::ConstPtr &msg) {

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

  for (size_t i = 0; i < N; i++) {
    // time
    mpcTimeTrajectoryBuffer_.push_back(msg->timeTrajectory[i]);
    // state
    mpcStateTrajectoryBuffer_.push_back(Eigen::Map<const Eigen::Matrix<float, STATE_DIM, 1>>(
        msg->stateTrajectory[i].value.data(), STATE_DIM).template cast<scalar_t>() );
  } // end of i loop

    std::vector<scalar_array_t const *> controllerData(N, nullptr);
    if(msg->data.size() != N){
        throw std::runtime_error("Data has the wrong length");
    }
    for(int i=0; i<N; i++){
        controllerData[i] = &(msg->data[i].data);
  }

    switch (msg->controllerType) {
    case ocs2_comm_interfaces::mpc_flattened_controller::CONTROLLER_SLQ_FEEDFORWARD:
    {
        mpcControllerBuffer_.reset(); // no additional controller needed in this case
        break;
    }
    case ocs2_comm_interfaces::mpc_flattened_controller::CONTROLLER_SLQ_FEEDBACK:
    {
        using controller_t = LinearController<STATE_DIM, INPUT_DIM>;
        mpcControllerBuffer_ = std::unique_ptr<controller_t>(new controller_t());
        mpcControllerBuffer_->unFlatten(mpcTimeTrajectoryBuffer_, controllerData);
    }
    case ocs2_comm_interfaces::mpc_flattened_controller::CONTROLLER_PATH_INTEGRAL:
    {
        //TODO(jcarius) instantiate PiController here
        // mpcControllerBuffer_ = std::make_unique<PiController<STATE_DIM, INPUT_DIM>(...);
        throw std::runtime_error("not implemented");
    }
    default:
        throw std::runtime_error("MRT_ROS_Interface::mpcPolicyCallback -- Unknown controllerType");
    }


  // customized adjustment
  modifyBufferPolicy(mpcInitObservationBuffer_, mpcControllerBuffer_.get(),
      mpcTimeTrajectoryBuffer_, mpcStateTrajectoryBuffer_,
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

    mpcTimeTrajectory_.swap(mpcTimeTrajectoryBuffer_);
    mpcStateTrajectory_.swap(mpcStateTrajectoryBuffer_);

    mpcLinInterpolateState_.reset();
    mpcLinInterpolateState_.setTimeStamp(&mpcTimeTrajectory_);
    mpcLinInterpolateState_.setData(&mpcStateTrajectory_);

    mpcController_.swap(mpcControllerBuffer_);

    loadModifiedPolicy(logicUpdated_, policyUpdated_,
                       *(mpcController_.get()),
                       mpcTimeTrajectory_, mpcStateTrajectory_,
                       eventTimes_, subsystemsSequence_);


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
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::evaluatePlan(
    const scalar_t &time,
    state_vector_t &mpcState,
    size_t &subsystem) {

  if (time > mpcTimeTrajectory_.back())
    ROS_WARN_STREAM("The requested time is greater than the received plan: "
                        + std::to_string(time) + ">" + std::to_string(mpcTimeTrajectory_.back()));

  mpcLinInterpolateState_.interpolate(time, mpcState);

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
                       LOGIC_RULES_T>::initRollout(const controlled_system_base_t &controlSystemBase,
                                                   const Rollout_Settings &rolloutSettings) {

  rolloutPtr_ = rollout_base_ptr_t(new time_triggered_rollout_t(controlSystemBase, rolloutSettings, "mrt"));
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
      input_vector_array_t inputTrajectoryDummy;
      rolloutPtr_->run(activePartitionIndex,
                       t0,
                       initState,
                       final_time,
                       mpcController_.get(),
                       *logicMachinePtr_,
                       mpcTimeTrajectory_,
                       eventsPastTheEndIndeces,
                       mpcStateTrajectory_,
                       inputTrajectoryDummy);
    } else {
      throw std::runtime_error("MRT_ROS_interface: policy not updated before rollout.");
    }
  } else {
    throw std::runtime_error("MRT_ROS_interface: rolloutPtr not initialized, call initRollout first.");
  }

  //TODO(jcarius) resetting should not be necessary
  // Set rollout to be the mpc feedforward trajectory
  mpcLinInterpolateState_.reset();
  mpcLinInterpolateState_.setTimeStamp(&mpcTimeTrajectory_);
  mpcLinInterpolateState_.setData(&mpcStateTrajectory_);

  loadModifiedPolicy(logicUpdated_, policyUpdated_, *mpcController_,
                                mpcTimeTrajectory_, mpcStateTrajectory_,
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
  mpcPolicySubscriber_.shutdown();

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

    mpcPolicySubscriber_ = mrtRosNodeHandlePtr_->subscribe(
        robotName_ + "_mpc_policy",
        1,
        &MRT_ROS_Interface::mpcPolicyCallback,
        this, ::ros::TransportHints().udp());

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
