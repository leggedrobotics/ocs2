/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ocs2 {
namespace {

template <typename Container>
bool allFinite(const Container& values) {
  return std::all_of(values.begin(), values.end(), [](const auto value) { return std::isfinite(value); });
}

}  // namespace

const rclcpp::Logger LOGGER = rclcpp::get_logger("MRT_ROS_Interface");

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Interface::MRT_ROS_Interface(std::string topicPrefix)
    : topicPrefix_(std::move(topicPrefix)) {
// Start thread for publishing
#ifdef PUBLISH_THREAD
  // Close old thread if it is already running
  shutdownPublisher();
  terminateThread_ = false;
  readyToPublish_ = false;
  publisherWorker_ =
      std::thread(&MRT_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Interface::~MRT_ROS_Interface() { shutdownNodes(); }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::reset() {
  std::lock_guard<std::mutex> lock(policyGenerationMutex_);
  expectedResetEpoch_ = 0;
  lastPolicySequence_ = 0;
  MRT_BASE::reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::acknowledgeMpcReset(uint64_t resetEpoch) {
  if (resetEpoch == 0) {
    throw std::invalid_argument("[MRT_ROS_Interface::acknowledgeMpcReset] resetEpoch must be nonzero.");
  }
  std::lock_guard<std::mutex> lock(policyGenerationMutex_);
  MRT_BASE::reset();
  lastPolicySequence_ = 0;
  expectedResetEpoch_ = resetEpoch;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::resetMpcNode(
    const TargetTrajectories& initTargetTrajectories) {
  this->reset();

  if (!node_) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::resetMpcNode] launchNodes() must be called before "
        "resetMpcNode().");
  }
  if (!mpcResetServiceClient_) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::resetMpcNode] Reset service client is not "
        "initialized.");
  }

  auto resetSrvRequest = std::make_shared<ocs2_msgs::srv::Reset::Request>();
  resetSrvRequest->reset = true;
  resetSrvRequest->target_trajectories =
      ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);

  while (!mpcResetServiceClient_->wait_for_service(std::chrono::seconds(5)) &&
         rclcpp::ok()) {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "Failed to call service to reset MPC, retrying...");
  }
  if (!rclcpp::ok()) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::resetMpcNode] ROS shutdown while waiting for MPC "
        "reset service.");
  }

  auto future =
      mpcResetServiceClient_->async_send_request(resetSrvRequest);
  const auto status =
      rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::resetMpcNode] MPC reset service call failed or "
        "timed out.");
  }
  const auto response = future.get();
  if (!response->done) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::resetMpcNode] MPC reset service returned done=false.");
  }
  if (response->reset_epoch == 0) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::resetMpcNode] MPC reset service returned reset_epoch=0.");
  }
  acknowledgeMpcReset(response->reset_epoch);

  RCLCPP_INFO_STREAM(LOGGER, "MPC node has been reset.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::setCurrentObservation(
    const SystemObservation& currentObservation) {
#ifdef PUBLISH_THREAD
  std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

  // create the message
  mpcObservationMsg_ =
      ros_msg_conversions::createObservationMsg(currentObservation);

  // publish the current observation
#ifdef PUBLISH_THREAD
  readyToPublish_ = true;
  lk.unlock();
  msgReady_.notify_one();
#else
  mpcObservationPublisher_->publish(mpcObservationMsg_);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::publisherWorkerThread() {
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

    mpcObservationPublisher_->publish(mpcObservationMsgBuffer_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::readPolicyMsg(
    const ocs2_msgs::msg::MpcFlattenedController& msg, CommandData& commandData,
    PrimalSolution& primalSolution, PerformanceIndex& performanceIndices) {
  const size_t N = msg.time_trajectory.size();
  if (N == 0) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::readPolicyMsg] controller message is empty!");
  }
  if (msg.state_trajectory.size() != N || msg.input_trajectory.size() != N) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::readPolicyMsg] state and input trajectories must "
        "have same length!");
  }
  if (msg.data.size() != N) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::readPolicyMsg] Data has the wrong length!");
  }
  if (msg.controller_type != ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD &&
      msg.controller_type != ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Unknown controllerType!");
  }

  const size_t stateDimension = msg.state_trajectory.front().value.size();
  const size_t inputDimension = msg.input_trajectory.front().value.size();
  if (stateDimension == 0) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] State vectors must not be empty!");
  }
  for (size_t i = 0; i < N; ++i) {
    if (!std::isfinite(msg.time_trajectory[i]) ||
        (i > 0 && msg.time_trajectory[i] < msg.time_trajectory[i - 1])) {
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Time trajectory must be finite and non-decreasing!");
    }
    const auto& state = msg.state_trajectory[i].value;
    const auto& input = msg.input_trajectory[i].value;
    const auto& controllerData = msg.data[i].data;
    if (state.size() != stateDimension || input.size() != inputDimension) {
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] State or input dimensions change within the policy!");
    }
    if (!allFinite(state) || !allFinite(input) || !allFinite(controllerData)) {
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Policy contains NaN or Inf!");
    }
    const size_t expectedControllerDataSize =
        msg.controller_type == ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD
            ? inputDimension
            : inputDimension + inputDimension * stateDimension;
    if (controllerData.size() != expectedControllerDataSize) {
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Controller data has the wrong length!");
    }
  }
  for (size_t i = 0; i < msg.post_event_indices.size(); ++i) {
    const size_t index = msg.post_event_indices[i];
    if (index >= N || (i > 0 && index <= msg.post_event_indices[i - 1])) {
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Post-event indices must be increasing and within the policy!");
    }
  }

  commandData.mpcInitObservation_ = ros_msg_conversions::readObservationMsg(msg.init_observation);
  commandData.mpcTargetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(msg.plan_target_trajectories);
  performanceIndices = ros_msg_conversions::readPerformanceIndicesMsg(msg.performance_indices);

  primalSolution.clear();

  primalSolution.modeSchedule_ =
      ros_msg_conversions::readModeScheduleMsg(msg.mode_schedule);

  size_array_t stateDim(N);
  size_array_t inputDim(N);
  primalSolution.timeTrajectory_.reserve(N);
  primalSolution.stateTrajectory_.reserve(N);
  primalSolution.inputTrajectory_.reserve(N);
  for (size_t i = 0; i < N; i++) {
    stateDim[i] = msg.state_trajectory[i].value.size();
    inputDim[i] = msg.input_trajectory[i].value.size();
    primalSolution.timeTrajectory_.emplace_back(msg.time_trajectory[i]);
    primalSolution.stateTrajectory_.emplace_back(
        Eigen::Map<const Eigen::VectorXf>(msg.state_trajectory[i].value.data(),
                                          stateDim[i])
            .cast<scalar_t>());
    primalSolution.inputTrajectory_.emplace_back(
        Eigen::Map<const Eigen::VectorXf>(msg.input_trajectory[i].value.data(),
                                          inputDim[i])
            .cast<scalar_t>());
  }

  primalSolution.postEventIndices_.reserve(msg.post_event_indices.size());
  for (auto ind : msg.post_event_indices) {
    primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
  }

  std::vector<std::vector<float> const*> controllerDataPtrArray(N, nullptr);
  for (size_t i = 0; i < N; i++) {
    controllerDataPtrArray[i] = &(msg.data[i].data);
  }

  // instantiate the correct controller
  switch (msg.controller_type) {
    case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD: {
      auto controller = FeedforwardController::unFlatten(
          primalSolution.timeTrajectory_, controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(
          new FeedforwardController(std::move(controller)));
      break;
    }
    case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR: {
      auto controller = LinearController::unFlatten(
          stateDim, inputDim, primalSolution.timeTrajectory_,
          controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(
          new LinearController(std::move(controller)));
      break;
    }
    default:
      throw std::runtime_error(
          "[MRT_ROS_Interface::readPolicyMsg] Unknown controllerType!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::validatePolicyGeneration(
    uint64_t expectedResetEpoch, uint64_t lastPolicySequence,
    uint64_t messageResetEpoch, uint64_t messagePolicySequence) {
  if (expectedResetEpoch == 0) {
    throw std::runtime_error(
        "[MRT_ROS_Interface] MPC must be reset before accepting a policy.");
  }
  if (messageResetEpoch != expectedResetEpoch) {
    throw std::runtime_error(
        "[MRT_ROS_Interface] Policy reset_epoch does not match the acknowledged reset generation.");
  }
  if (messagePolicySequence == 0 || messagePolicySequence <= lastPolicySequence) {
    throw std::runtime_error(
        "[MRT_ROS_Interface] Policy sequence is zero, duplicate, or out of order.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::mpcPolicyCallback(
    const ocs2_msgs::msg::MpcFlattenedController::ConstSharedPtr& msg) {
  try {
    std::lock_guard<std::mutex> generationLock(policyGenerationMutex_);
    validatePolicyGeneration(expectedResetEpoch_, lastPolicySequence_, msg->reset_epoch, msg->policy_sequence);

    // read new policy and command from msg
    auto commandPtr = std::make_unique<CommandData>();
    auto primalSolutionPtr = std::make_unique<PrimalSolution>();
    auto performanceIndicesPtr = std::make_unique<PerformanceIndex>();
    readPolicyMsg(*msg, *commandPtr, *primalSolutionPtr,
                  *performanceIndicesPtr);

    this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr),
                       std::move(performanceIndicesPtr));
    lastPolicySequence_ = msg->policy_sequence;
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "[MRT_ROS_Interface] Dropping invalid policy message: "
                            << e.what());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::shutdownNodes() {
#ifdef PUBLISH_THREAD
  RCLCPP_INFO_STREAM(LOGGER, "Shutting down workers ...");

  shutdownPublisher();

  RCLCPP_INFO_STREAM(LOGGER, "All workers are shut down.");
#endif

  // clean up callback queue
  // callback_executor_.cancel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::shutdownPublisher() {
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
void MRT_ROS_Interface::spinMRT() {
  // callback_executor_.spin_once();
  if (!node_) {
    throw std::runtime_error(
        "[MRT_ROS_Interface::spinMRT] launchNodes() must be called before spinMRT().");
  }
  rclcpp::spin_some(node_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Interface::launchNodes(const rclcpp::Node::SharedPtr& node) {
  this->reset();
  node_ = node;
  // display
  RCLCPP_INFO_STREAM(LOGGER, "MRT node is setting up ...");

  // observation publisher
  mpcObservationPublisher_ =
      node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
          topicPrefix_ + "_mpc_observation", 1);

  // policy subscriber
  const auto latchedQos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  mpcPolicySubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
          topicPrefix_ + "_mpc_policy",  // topic name
          latchedQos,
          std::bind(&MRT_ROS_Interface::mpcPolicyCallback, this,
                    std::placeholders::_1));

  // MPC reset service client
  mpcResetServiceClient_ =
      node_->create_client<ocs2_msgs::srv::Reset>(topicPrefix_ + "_mpc_reset");

  // display
#ifdef PUBLISH_THREAD
  RCLCPP_INFO_STREAM(LOGGER, "Publishing MRT messages on a separate thread.");
#endif

  RCLCPP_INFO_STREAM(LOGGER, "MRT node is ready.");

  spinMRT();
}

}  // namespace ocs2
