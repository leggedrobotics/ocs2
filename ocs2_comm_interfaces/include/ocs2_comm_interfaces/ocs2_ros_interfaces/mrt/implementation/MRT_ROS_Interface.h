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
MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::MRT_ROS_Interface(
    std::string robotName /*= "robot"*/, ros::TransportHints mrtTransportHints /* = ::ros::TransportHints().tcpNoDelay()*/)
    : Base(), robotName_(std::move(robotName)), mrtTransportHints_(mrtTransportHints) {
// Start thread for publishing
#ifdef PUBLISH_THREAD
  // Close old thread if it is already running
  shutdownPublisher();
  terminateThread_ = false;
  readyToPublish_ = false;
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
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::resetMpcNode(const CostDesiredTrajectories& initCostDesiredTrajectories) {
  this->policyReceivedEver_ = false;

  ocs2_msgs::reset resetSrv;
  resetSrv.request.reset = true;

  ros_msg_conversions::createTargetTrajectoriesMsg(initCostDesiredTrajectories, resetSrv.request.targetTrajectories);

  while (!mpcResetServiceClient_.waitForExistence(ros::Duration(5.0)) && ::ros::ok() && ::ros::master::check()) {
    ROS_ERROR_STREAM("Failed to call service to reset MPC, retrying...");
  }

  mpcResetServiceClient_.call(resetSrv);
  ROS_INFO_STREAM("MPC node is reset.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::setCurrentObservation(const system_observation_t& currentObservation) {
#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

  // create the message
  ros_msg_conversions::createObservationMsg(currentObservation, mpcObservationMsg_);

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
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg) {
  //	std::cout << "\t Plan is received at time: " << msg->initObservation.time << std::endl;

  std::lock_guard<std::mutex> lk(this->policyBufferMutex_);
  auto& timeBuffer = this->primalSolutionBuffer_->timeTrajectory_;
  auto& stateBuffer = this->primalSolutionBuffer_->stateTrajectory_;
  auto& inputBuffer = this->primalSolutionBuffer_->inputTrajectory_;
  auto& controlBuffer = this->primalSolutionBuffer_->controllerPtr_;
  auto& modeScheduleBuffer = this->primalSolutionBuffer_->modeSchedule_;
  auto& initObservationBuffer = this->commandBuffer_->mpcInitObservation_;
  auto& costDesiredBuffer = this->commandBuffer_->mpcCostDesiredTrajectories_;

  // if MPC did not update the policy
  if (!static_cast<bool>(msg->controllerIsUpdated)) {
    timeBuffer.clear();
    stateBuffer.clear();
    inputBuffer.clear();
    controlBuffer.reset(nullptr);
    modeScheduleBuffer = ModeSchedule({}, {0});
    initObservationBuffer = system_observation_t();
    costDesiredBuffer.clear();

    this->policyUpdatedBuffer_ = false;
    this->newPolicyInBuffer_ = true;

    return;
  }

  ros_msg_conversions::readObservationMsg(msg->initObservation, initObservationBuffer);
  ros_msg_conversions::readTargetTrajectoriesMsg(msg->planTargetTrajectories, costDesiredBuffer);
  modeScheduleBuffer = ros_msg_conversions::readModeScheduleMsg(msg->modeSchedule);

  this->policyUpdatedBuffer_ = msg->controllerIsUpdated;

  const scalar_t partitionInitMargin = 1e-1;  //! @badcode Is this necessary?
  this->partitioningTimesUpdate(initObservationBuffer.time() - partitionInitMargin, this->partitioningTimesBuffer_);

  const size_t N = msg->timeTrajectory.size();

  timeBuffer.clear();
  timeBuffer.reserve(N);
  stateBuffer.clear();
  stateBuffer.reserve(N);
  inputBuffer.clear();
  inputBuffer.reserve(N);

  for (size_t i = 0; i < N; i++) {
    timeBuffer.emplace_back(msg->timeTrajectory[i]);
    stateBuffer.emplace_back(
        Eigen::Map<const Eigen::Matrix<float, STATE_DIM, 1>>(msg->stateTrajectory[i].value.data(), STATE_DIM).template cast<scalar_t>());
    inputBuffer.emplace_back(
        Eigen::Map<const Eigen::Matrix<float, INPUT_DIM, 1>>(msg->inputTrajectory[i].value.data(), INPUT_DIM).template cast<scalar_t>());
  }  // end of i loop

  // instantiate the correct controller
  switch (msg->controllerType) {
    case ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD: {
      using specific_controller_t = FeedforwardController<STATE_DIM, INPUT_DIM>;
      controlBuffer.reset(new specific_controller_t());
      break;
    }
    case ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR: {
      using specific_controller_t = LinearController<STATE_DIM, INPUT_DIM>;
      controlBuffer.reset(new specific_controller_t());
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
  controlBuffer->unFlatten(timeBuffer, controllerDataPtrArray);

  // allow user to modify the buffer
  this->modifyBufferPolicy(*this->commandBuffer_, *this->primalSolutionBuffer_);

  if (!this->policyReceivedEver_ && this->policyUpdatedBuffer_) {
    this->policyReceivedEver_ = true;
    this->initPlanObservation_ = initObservationBuffer;
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
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::spinMRT() {
  mrtCallbackQueue_.callOne();
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Interface<STATE_DIM, INPUT_DIM>::launchNodes(ros::NodeHandle& nodeHandle) {
  this->reset();

  // display
  ROS_INFO_STREAM("MRT node is setting up ...");

  // Observation publisher
  mpcObservationPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1);

  // SLQ-MPC subscriber
  auto ops = ros::SubscribeOptions::create<ocs2_msgs::mpc_flattened_controller>(
      robotName_ + "_mpc_policy",                                                         // topic name
      1,                                                                                  // queue length
      boost::bind(&MRT_ROS_Interface::mpcPolicyCallback, this, boost::placeholders::_1),  // callback
      ros::VoidConstPtr(),                                                                // tracked object
      &mrtCallbackQueue_                                                                  // pointer to callback queue object
  );
  ops.transport_hints = mrtTransportHints_;
  mpcPolicySubscriber_ = nodeHandle.subscribe(ops);

  // MPC reset service client
  mpcResetServiceClient_ = nodeHandle.serviceClient<ocs2_msgs::reset>(robotName_ + "_mpc_reset");

  // display
#ifdef PUBLISH_THREAD
  ROS_INFO_STREAM("Publishing MRT messages on a separate thread.");
#endif

  ROS_INFO_STREAM("MRT node is ready.");

  spinMRT();
}

}  // namespace ocs2
