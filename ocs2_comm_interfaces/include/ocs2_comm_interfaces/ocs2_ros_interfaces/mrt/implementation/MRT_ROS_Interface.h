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

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::MRT_ROS_Interface(std::string robotName /*= "robot"*/,
                                                           std::shared_ptr<HybridLogicRules> logicRules /*= nullptr*/)
    : Base(logicRules) {
  robotName_ = std::move(robotName);

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
MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::~MRT_ROS_Interface() {
  shutdownNodes();
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
  Base::reset();
  std::lock_guard<std::mutex> lkUpdate(this->policyBufferMutex_);
  terminateThread_ = false;
  readyToPublish_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  this->policyReceivedEver_ = false;

  ocs2_comm_interfaces::reset resetSrv;
  resetSrv.request.reset = true;

  RosMsgConversions<STATE_DIM, INPUT_DIM>::CreateTargetTrajectoriesMsg(initCostDesiredTrajectories, resetSrv.request.targetTrajectories);

  if (mpcResetServiceClient_.waitForExistence(ros::Duration(5.0))) {
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
  while (!terminateThread_) {
    std::unique_lock<std::mutex> lk(publisherMutex_);

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    if (terminateThread_) {
      break;
    }

    mpcObservationMsgBuffer_ = std::move(mpcObservationMsg_);

    readyToPublish_ = false;

    lk.unlock();
    msgReady_.notify_one();

    mpcObservationPublisher_.publish(mpcObservationMsgBuffer_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcPolicyCallback(const ocs2_comm_interfaces::mpc_flattened_controller::ConstPtr& msg) {
  //	std::cout << "\t Plan is received at time: " << msg->initObservation.time << std::endl;

  std::lock_guard<std::mutex> lk(this->policyBufferMutex_);

  // if mpc did not update the policy
  if (!static_cast<bool>(msg->controllerIsUpdated)) {
    this->mpcInitObservationBuffer_ = system_observation_t();
    this->mpcCostDesiredTrajectoriesBuffer_.clear();
    this->policyUpdatedBuffer_ = false;
    this->eventTimesBuffer_.clear();
    this->subsystemsSequenceBuffer_.clear();
    this->mpcTimeTrajectoryBuffer_.clear();
    this->mpcStateTrajectoryBuffer_.clear();
    this->mpcControllerBuffer_.reset(nullptr);

    this->newPolicyInBuffer_ = true;

    return;
  }

  ros_msg_conversions_t::ReadObservationMsg(msg->initObservation, this->mpcInitObservationBuffer_);
  ros_msg_conversions_t::ReadTargetTrajectoriesMsg(msg->planTargetTrajectories, this->mpcCostDesiredTrajectoriesBuffer_);
  ros_msg_conversions_t::ReadModeSequenceMsg(msg->modeSequence, this->eventTimesBuffer_, this->subsystemsSequenceBuffer_);

  this->policyUpdatedBuffer_ = msg->controllerIsUpdated;

  const scalar_t partitionInitMargin = 1e-1;  //! @badcode Is this necessary?
  this->partitioningTimesUpdate(this->mpcInitObservationBuffer_.time() - partitionInitMargin, this->partitioningTimesBuffer_);

  const size_t N = msg->timeTrajectory.size();
  this->mpcTimeTrajectoryBuffer_.clear();
  this->mpcTimeTrajectoryBuffer_.reserve(N);
  this->mpcStateTrajectoryBuffer_.clear();
  this->mpcStateTrajectoryBuffer_.reserve(N);

  for (size_t i = 0; i < N; i++) {
    this->mpcTimeTrajectoryBuffer_.push_back(msg->timeTrajectory[i]);
    this->mpcStateTrajectoryBuffer_.push_back(
        Eigen::Map<const Eigen::Matrix<float, STATE_DIM, 1>>(msg->stateTrajectory[i].value.data(), STATE_DIM).template cast<scalar_t>());
  }  // end of i loop

  // instantiate the correct controller
  switch (msg->controllerType) {
    case ocs2_comm_interfaces::mpc_flattened_controller::CONTROLLER_FEEDFORWARD: {
      using controller_t = FeedforwardController<STATE_DIM, INPUT_DIM>;
      this->mpcControllerBuffer_.reset(new controller_t());
      break;
    }
    case ocs2_comm_interfaces::mpc_flattened_controller::CONTROLLER_LINEAR: {
      using controller_t = LinearController<STATE_DIM, INPUT_DIM>;
      this->mpcControllerBuffer_.reset(new controller_t());
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
  this->mpcControllerBuffer_->unFlatten(this->mpcTimeTrajectoryBuffer_, controllerDataPtrArray);

  // customized adjustment
  this->modifyBufferPolicy(this->mpcInitObservationBuffer_, *this->mpcControllerBuffer_, this->mpcTimeTrajectoryBuffer_,
                           this->mpcStateTrajectoryBuffer_, this->eventTimesBuffer_, this->subsystemsSequenceBuffer_);

  if (!this->policyReceivedEver_ && this->policyUpdatedBuffer_) {
    this->policyReceivedEver_ = true;
    this->initPlanObservation_ = this->mpcInitObservationBuffer_;
    this->initCall(this->initPlanObservation_);
  }

  this->newPolicyInBuffer_ = true;
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
