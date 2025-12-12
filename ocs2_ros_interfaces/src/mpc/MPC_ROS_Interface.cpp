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

#include "ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h"

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

const rclcpp::Logger LOGGER = rclcpp::get_logger("MPC_ROS_Interface");

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_ROS_Interface::MPC_ROS_Interface(MPC_BASE& mpc, std::string topicPrefix)
    : mpc_(mpc),
      topicPrefix_(std::move(topicPrefix)),
      bufferPrimalSolutionPtr_(new PrimalSolution()),
      publisherPrimalSolutionPtr_(new PrimalSolution()),
      bufferCommandPtr_(new CommandData()),
      publisherCommandPtr_(new CommandData()),
      bufferPerformanceIndicesPtr_(new PerformanceIndex),
      publisherPerformanceIndicesPtr_(new PerformanceIndex) {
  // start thread for publishing
#ifdef PUBLISH_THREAD
  publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorker, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_ROS_Interface::~MPC_ROS_Interface() { shutdownNode(); }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::resetMpcNode(
    TargetTrajectories&& initTargetTrajectories) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);
  mpc_.reset();
  mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(
      std::move(initTargetTrajectories));
  mpcTimer_.reset();
  resetRequestedEver_ = true;
  terminateThread_ = false;
  readyToPublish_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::resetMpcCallback(
    const std::shared_ptr<ocs2_msgs::srv::Reset::Request> req,
    std::shared_ptr<ocs2_msgs::srv::Reset::Response> res) {
  if (static_cast<bool>(req->reset)) {
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(
        req->target_trajectories);
    resetMpcNode(std::move(targetTrajectories));
    res->done = static_cast<uint8_t>(true);

    std::cerr << "\n#####################################################"
              << "\n#####################################################"
              << "\n#################  MPC is reset.  ###################"
              << "\n#####################################################"
              << "\n#####################################################\n";
  } else {
    RCLCPP_WARN_STREAM(LOGGER, "[MPC_ROS_Interface] Reset request failed!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::MpcFlattenedController MPC_ROS_Interface::createMpcPolicyMsg(
    const PrimalSolution& primalSolution, const CommandData& commandData,
    const PerformanceIndex& performanceIndices) {
  ocs2_msgs::msg::MpcFlattenedController mpcPolicyMsg;

  mpcPolicyMsg.init_observation = ros_msg_conversions::createObservationMsg(
      commandData.mpcInitObservation_);
  mpcPolicyMsg.plan_target_trajectories =
      ros_msg_conversions::createTargetTrajectoriesMsg(
          commandData.mpcTargetTrajectories_);
  mpcPolicyMsg.mode_schedule =
      ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);
  mpcPolicyMsg.performance_indices =
      ros_msg_conversions::createPerformanceIndicesMsg(
          commandData.mpcInitObservation_.time, performanceIndices);

  switch (primalSolution.controllerPtr_->getType()) {
    case ControllerType::FEEDFORWARD:
      mpcPolicyMsg.controller_type =
          ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD;
      break;
    case ControllerType::LINEAR:
      mpcPolicyMsg.controller_type =
          ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR;
      break;
    default:
      throw std::runtime_error(
          "MPC_ROS_Interface::createMpcPolicyMsg: Unknown ControllerType");
  }

  // maximum length of the message
  const size_t N = primalSolution.timeTrajectory_.size();

  mpcPolicyMsg.time_trajectory.clear();
  mpcPolicyMsg.time_trajectory.reserve(N);
  mpcPolicyMsg.state_trajectory.clear();
  mpcPolicyMsg.state_trajectory.reserve(N);
  mpcPolicyMsg.data.clear();
  mpcPolicyMsg.data.reserve(N);
  mpcPolicyMsg.post_event_indices.clear();
  mpcPolicyMsg.post_event_indices.reserve(
      primalSolution.postEventIndices_.size());

  // time
  for (auto t : primalSolution.timeTrajectory_) {
    mpcPolicyMsg.time_trajectory.emplace_back(t);
  }

  // post-event indices
  for (auto ind : primalSolution.postEventIndices_) {
    mpcPolicyMsg.post_event_indices.emplace_back(static_cast<uint16_t>(ind));
  }

  // state
  for (size_t k = 0; k < N; k++) {
    ocs2_msgs::msg::MpcState mpcState;
    mpcState.value.resize(primalSolution.stateTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++) {
      mpcState.value[j] = primalSolution.stateTrajectory_[k](j);
    }
    mpcPolicyMsg.state_trajectory.emplace_back(mpcState);
  }  // end of k loop

  // input
  for (size_t k = 0; k < N; k++) {
    ocs2_msgs::msg::MpcInput mpcInput;
    mpcInput.value.resize(primalSolution.inputTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++) {
      mpcInput.value[j] = primalSolution.inputTrajectory_[k](j);
    }
    mpcPolicyMsg.input_trajectory.emplace_back(mpcInput);
  }  // end of k loop

  // controller
  scalar_array_t timeTrajectoryTruncated;
  std::vector<std::vector<float>*> policyMsgDataPointers;
  policyMsgDataPointers.reserve(N);
  for (auto t : primalSolution.timeTrajectory_) {
    mpcPolicyMsg.data.emplace_back(ocs2_msgs::msg::ControllerData());

    policyMsgDataPointers.push_back(&mpcPolicyMsg.data.back().data);
    timeTrajectoryTruncated.push_back(t);
  }  // end of k loop

  // serialize controller into data buffer
  primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated,
                                         policyMsgDataPointers);

  return mpcPolicyMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::publisherWorker() {
  while (!terminateThread_) {
    std::unique_lock<std::mutex> lk(publisherMutex_);

    msgReady_.wait(lk, [&] { return (readyToPublish_ || terminateThread_); });

    if (terminateThread_) {
      break;
    }

    {
      std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);
      publisherCommandPtr_.swap(bufferCommandPtr_);
      publisherPrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
      publisherPerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
    }

    ocs2_msgs::msg::MpcFlattenedController mpcPolicyMsg =
        createMpcPolicyMsg(*publisherPrimalSolutionPtr_, *publisherCommandPtr_,
                           *publisherPerformanceIndicesPtr_);

    // publish the message
    mpcPolicyPublisher_->publish(mpcPolicyMsg);

    readyToPublish_ = false;
    lk.unlock();
    msgReady_.notify_one();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::copyToBuffer(
    const SystemObservation& mpcInitObservation) {
  // buffer policy mutex
  std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);

  // get solution
  scalar_t finalTime =
      mpcInitObservation.time + mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) {
    finalTime = mpc_.getSolverPtr()->getFinalTime();
  }
  mpc_.getSolverPtr()->getPrimalSolution(finalTime,
                                         bufferPrimalSolutionPtr_.get());

  // command
  bufferCommandPtr_->mpcInitObservation_ = mpcInitObservation;
  bufferCommandPtr_->mpcTargetTrajectories_ =
      mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();

  // performance indices
  *bufferPerformanceIndicesPtr_ = mpc_.getSolverPtr()->getPerformanceIndeces();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::mpcObservationCallback(
    const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);

  if (!resetRequestedEver_.load()) {
    RCLCPP_WARN_STREAM(LOGGER,
                       "MPC should be reset first. Either call "
                       "MPC_ROS_Interface::reset() or use the reset service.");
    return;
  }

  // current time, state, input, and subsystem
  const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

  // measure the delay in running MPC
  mpcTimer_.startTimer();

  // run MPC
  bool controllerIsUpdated =
      mpc_.run(currentObservation.time, currentObservation.state);
  if (!controllerIsUpdated) {
    return;
  }
  copyToBuffer(currentObservation);

  // measure the delay for sending ROS messages
  mpcTimer_.endTimer();

  // check MPC delay and solution window compatibility
  scalar_t timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
  }
  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) {
    std::cerr << "WARNING: The solution time window might be shorter than the "
                 "MPC delay!\n";
  }

  // display
  if (mpc_.settings().debugPrint_) {
    std::cerr << '\n';
    std::cerr << "\n### MPC_ROS Benchmarking";
    std::cerr << "\n###   Maximum : "
              << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds()
              << "[ms].";
    std::cerr << "\n###   Latest  : "
              << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]."
              << std::endl;
  }

#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();

#else
  ocs2_msgs::msg::MpcFlattenedController mpcPolicyMsg =
      createMpcPolicyMsg(*bufferPrimalSolutionPtr_, *bufferCommandPtr_,
                         *bufferPerformanceIndicesPtr_);
  mpcPolicyPublisher_.publish(mpcPolicyMsg);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::shutdownNode() {
#ifdef PUBLISH_THREAD
  RCLCPP_INFO(LOGGER, "Shutting down workers ...");

  std::unique_lock<std::mutex> lk(publisherMutex_);
  terminateThread_ = true;
  lk.unlock();

  msgReady_.notify_all();

  if (publisherWorker_.joinable()) {
    publisherWorker_.join();
  }

  RCLCPP_INFO(LOGGER, "All workers are shut down.");
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::spin() {
  RCLCPP_INFO(LOGGER, "Start spinning now ...");
  // Equivalent to ros::spin() + check if master is alive
  while (rclcpp::ok()) {
    rclcpp::spin(node_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::launchNodes(const rclcpp::Node::SharedPtr& node) {
  RCLCPP_INFO(LOGGER, "MPC node is setting up ...");
  node_ = node;

  // Observation subscriber
  mpcObservationSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
          topicPrefix_ + "_mpc_observation", 1,
          std::bind(&MPC_ROS_Interface::mpcObservationCallback, this,
                    std::placeholders::_1));

  // MPC publisher
  mpcPolicyPublisher_ =
      node_->create_publisher<ocs2_msgs::msg::MpcFlattenedController>(
          topicPrefix_ + "_mpc_policy", 1);

  // MPC reset service server
  mpcResetServiceServer_ = node_->create_service<ocs2_msgs::srv::Reset>(
      topicPrefix_ + "_mpc_reset",
      [this](const std::shared_ptr<ocs2_msgs::srv::Reset::Request>& request,
             const std::shared_ptr<ocs2_msgs::srv::Reset::Response>& response) {
        return resetMpcCallback(request, response);
      });

  // display
#ifdef PUBLISH_THREAD
  RCLCPP_INFO(LOGGER, "Publishing SLQ-MPC messages on a separate thread.");
#endif

  RCLCPP_INFO(LOGGER, "MPC node is ready.");

  // spin
  spin();
}

}  // namespace ocs2
