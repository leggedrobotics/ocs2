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

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <utility>

const rclcpp::Logger LOGGER = rclcpp::get_logger("MPC_ROS_Interface");

namespace ocs2 {
namespace {

size_t inferDimFromTrajectory(const vector_array_t& trajectory,
                              bool& hasSamples) {
  hasSamples = !trajectory.empty();
  return hasSamples ? trajectory.front().size() : 0;
}

[[noreturn]] void throwTargetTrajectoriesError(const std::string& message) {
  throw std::runtime_error(
      std::string("[MPC_ROS_Interface] Invalid reset target trajectories: ") +
      message);
}

void validateResetTargetTrajectories(
    const TargetTrajectories& targetTrajectories,
    const TargetTrajectories& referenceTrajectories) {
  const auto N = targetTrajectories.timeTrajectory.size();
  if (N == 0) {
    throwTargetTrajectoriesError("timeTrajectory is empty.");
  }
  if (targetTrajectories.stateTrajectory.size() != N) {
    throwTargetTrajectoriesError("stateTrajectory length does not match timeTrajectory.");
  }
  if (!targetTrajectories.inputTrajectory.empty() &&
      targetTrajectories.inputTrajectory.size() != N) {
    throwTargetTrajectoriesError("inputTrajectory length does not match timeTrajectory.");
  }

  for (size_t i = 1; i < N; i++) {
    if (targetTrajectories.timeTrajectory[i] <
        targetTrajectories.timeTrajectory[i - 1]) {
      std::ostringstream stream;
      stream << "timeTrajectory must be non-decreasing, but t[" << i - 1
             << "]=" << targetTrajectories.timeTrajectory[i - 1] << " and t["
             << i << "]=" << targetTrajectories.timeTrajectory[i] << ".";
      throwTargetTrajectoriesError(stream.str());
    }
  }

  const auto stateDim = targetTrajectories.stateTrajectory.front().size();
  for (size_t i = 0; i < N; i++) {
    const auto sampleDim = targetTrajectories.stateTrajectory[i].size();
    if (sampleDim != stateDim) {
      std::ostringstream stream;
      stream << "stateTrajectory dimension mismatch at index " << i
             << ": expected " << stateDim << ", got " << sampleDim << ".";
      throwTargetTrajectoriesError(stream.str());
    }
  }

  size_t inputDim = 0;
  if (!targetTrajectories.inputTrajectory.empty()) {
    inputDim = targetTrajectories.inputTrajectory.front().size();
    for (size_t i = 0; i < N; i++) {
      const auto sampleDim = targetTrajectories.inputTrajectory[i].size();
      if (sampleDim != inputDim) {
        std::ostringstream stream;
        stream << "inputTrajectory dimension mismatch at index " << i
               << ": expected " << inputDim << ", got " << sampleDim << ".";
        throwTargetTrajectoriesError(stream.str());
      }
    }
  }

  bool hasReferenceState = false;
  const auto expectedStateDim =
      inferDimFromTrajectory(referenceTrajectories.stateTrajectory,
                            hasReferenceState);
  if (hasReferenceState && stateDim != expectedStateDim) {
    std::ostringstream stream;
    stream << "stateTrajectory dimension mismatch vs active reference: expected "
           << expectedStateDim << ", got " << stateDim << ".";
    throwTargetTrajectoriesError(stream.str());
  }

  bool hasReferenceInput = false;
  const auto expectedInputDim =
      inferDimFromTrajectory(referenceTrajectories.inputTrajectory,
                            hasReferenceInput);
  if (hasReferenceInput && !targetTrajectories.inputTrajectory.empty() &&
      inputDim != expectedInputDim) {
    std::ostringstream stream;
    stream << "inputTrajectory dimension mismatch vs active reference: expected "
           << expectedInputDim << ", got " << inputDim << ".";
    throwTargetTrajectoriesError(stream.str());
  }
  if (hasReferenceInput && targetTrajectories.inputTrajectory.empty() &&
      expectedInputDim > 0) {
    std::ostringstream stream;
    stream << "inputTrajectory is empty but active reference expects dimension "
           << expectedInputDim << ".";
    throwTargetTrajectoriesError(stream.str());
  }
  if (hasReferenceInput && expectedInputDim == 0 &&
      !targetTrajectories.inputTrajectory.empty()) {
    for (size_t i = 0; i < N; i++) {
      if (targetTrajectories.inputTrajectory[i].size() != 0) {
        std::ostringstream stream;
        stream << "inputTrajectory must be zero-dimension at index " << i
               << " to match active reference.";
        throwTargetTrajectoriesError(stream.str());
      }
    }
  }
}

[[noreturn]] void throwPolicyError(const std::string& message) {
  throw std::runtime_error("[MPC_ROS_Interface] Invalid policy: " + message);
}

float checkedPolicyFloat(scalar_t value, const std::string& field) {
  if (!std::isfinite(value)) {
    throwPolicyError(field + " must be finite.");
  }
  if (std::abs(value) > static_cast<scalar_t>(std::numeric_limits<float>::max())) {
    throwPolicyError(field + " exceeds the float32 message range.");
  }
  return static_cast<float>(value);
}

void validatePrimalSolutionForRos(const PrimalSolution& primalSolution) {
  if (primalSolution.controllerPtr_ == nullptr) {
    throwPolicyError("controllerPtr is null.");
  }
  const size_t N = primalSolution.timeTrajectory_.size();
  if (N == 0) {
    throwPolicyError("time trajectory is empty.");
  }
  if (primalSolution.stateTrajectory_.size() != N || primalSolution.inputTrajectory_.size() != N) {
    throwPolicyError("time, state, and input trajectory lengths do not match.");
  }

  const size_t stateDimension = primalSolution.stateTrajectory_.front().size();
  const size_t inputDimension = primalSolution.inputTrajectory_.front().size();
  if (stateDimension == 0) {
    throwPolicyError("state vectors must not be empty.");
  }
  for (size_t i = 0; i < N; ++i) {
    const scalar_t time = primalSolution.timeTrajectory_[i];
    if (!std::isfinite(time) || (i > 0 && time < primalSolution.timeTrajectory_[i - 1])) {
      throwPolicyError("time trajectory must be finite and non-decreasing.");
    }
    if (primalSolution.stateTrajectory_[i].size() != stateDimension ||
        primalSolution.inputTrajectory_[i].size() != inputDimension) {
      throwPolicyError("state or input dimensions change within the trajectory.");
    }
    if (!primalSolution.stateTrajectory_[i].allFinite() || !primalSolution.inputTrajectory_[i].allFinite()) {
      throwPolicyError("state or input trajectory contains NaN or Inf.");
    }
  }

  for (size_t i = 0; i < primalSolution.postEventIndices_.size(); ++i) {
    const size_t index = primalSolution.postEventIndices_[i];
    if (index >= N || index > std::numeric_limits<uint16_t>::max() ||
        (i > 0 && index <= primalSolution.postEventIndices_[i - 1])) {
      throwPolicyError("post-event indices must be increasing, fit uint16, and lie within the trajectory.");
    }
  }
}

}  // namespace

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
uint64_t MPC_ROS_Interface::resetMpcNode(
    TargetTrajectories&& initTargetTrajectories) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);
  mpcReady_ = false;
  readyToPublish_ = false;
  mpc_.reset();
  mpc_.getSolverPtr()->getReferenceManager().setTargetTrajectories(
      std::move(initTargetTrajectories));
  mpcTimer_.reset();
  ++resetEpoch_;
  policySequence_ = 0;
  mpcReady_ = true;
  terminateThread_ = false;
  return resetEpoch_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::resetMpcCallback(
    const std::shared_ptr<ocs2_msgs::srv::Reset::Request> req,
    std::shared_ptr<ocs2_msgs::srv::Reset::Response> res) {
  if (req->reset) {
    try {
      auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(
          req->target_trajectories);
      const auto& referenceTrajectories =
          mpc_.getSolverPtr()->getReferenceManager().getTargetTrajectories();
      validateResetTargetTrajectories(targetTrajectories, referenceTrajectories);

      res->reset_epoch = resetMpcNode(std::move(targetTrajectories));
      res->done = true;

      std::cerr << "\n#####################################################"
                << "\n#####################################################"
                << "\n#################  MPC is reset.  ###################"
                << "\n#####################################################"
                << "\n#####################################################\n";
    } catch (const std::exception& e) {
      res->done = false;
      RCLCPP_ERROR_STREAM(LOGGER,
                          "[MPC_ROS_Interface] Reset request failed: "
                              << e.what());
    }
  } else {
    res->done = false;
    RCLCPP_WARN_STREAM(LOGGER, "[MPC_ROS_Interface] Reset request failed!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::MpcFlattenedController MPC_ROS_Interface::createMpcPolicyMsg(
    const PrimalSolution& primalSolution, const CommandData& commandData,
    const PerformanceIndex& performanceIndices, uint64_t resetEpoch,
    uint64_t policySequence) {
  if (resetEpoch == 0 || policySequence == 0) {
    throwPolicyError("reset epoch and policy sequence must be nonzero.");
  }
  validatePrimalSolutionForRos(primalSolution);

  ocs2_msgs::msg::MpcFlattenedController mpcPolicyMsg;
  mpcPolicyMsg.reset_epoch = resetEpoch;
  mpcPolicyMsg.policy_sequence = policySequence;

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
      mpcState.value[j] = checkedPolicyFloat(primalSolution.stateTrajectory_[k](j), "state trajectory");
    }
    mpcPolicyMsg.state_trajectory.emplace_back(mpcState);
  }  // end of k loop

  // input
  for (size_t k = 0; k < N; k++) {
    ocs2_msgs::msg::MpcInput mpcInput;
    mpcInput.value.resize(primalSolution.inputTrajectory_[k].rows());
    for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++) {
      mpcInput.value[j] = checkedPolicyFloat(primalSolution.inputTrajectory_[k](j), "input trajectory");
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

  const size_t stateDimension = primalSolution.stateTrajectory_.front().size();
  const size_t inputDimension = primalSolution.inputTrajectory_.front().size();
  for (const auto& data : mpcPolicyMsg.data) {
    const size_t expectedSize =
        mpcPolicyMsg.controller_type == ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD
            ? inputDimension
            : inputDimension + inputDimension * stateDimension;
    if (data.data.size() != expectedSize ||
        !std::all_of(data.data.begin(), data.data.end(), [](float value) { return std::isfinite(value); })) {
      throwPolicyError("flattened controller data has the wrong length or contains NaN or Inf.");
    }
  }

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
    readyToPublish_ = false;
    lk.unlock();
    msgReady_.notify_one();

    {
      std::lock_guard<std::mutex> policyBufferLock(bufferMutex_);
      publisherCommandPtr_.swap(bufferCommandPtr_);
      publisherPrimalSolutionPtr_.swap(bufferPrimalSolutionPtr_);
      publisherPerformanceIndicesPtr_.swap(bufferPerformanceIndicesPtr_);
      std::swap(publisherResetEpoch_, bufferResetEpoch_);
      std::swap(publisherPolicySequence_, bufferPolicySequence_);
    }

    try {
      ocs2_msgs::msg::MpcFlattenedController mpcPolicyMsg =
          createMpcPolicyMsg(*publisherPrimalSolutionPtr_, *publisherCommandPtr_,
                             *publisherPerformanceIndicesPtr_, publisherResetEpoch_,
                             publisherPolicySequence_);

      std::lock_guard<std::mutex> resetLock(resetMutex_);
      if (mpcReady_ && publisherResetEpoch_ == resetEpoch_) {
        mpcPolicyPublisher_->publish(mpcPolicyMsg);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(LOGGER,
                          "[MPC_ROS_Interface] Policy publisher failed: "
                              << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(LOGGER,
                          "[MPC_ROS_Interface] Policy publisher failed: unknown exception");
    }

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
  bufferResetEpoch_ = resetEpoch_;
  bufferPolicySequence_ = ++policySequence_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::mpcObservationCallback(
    const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> resetLock(resetMutex_);

  if (!mpcReady_.load()) {
    RCLCPP_WARN_STREAM(LOGGER,
                       "MPC is not ready. Either call "
                       "MPC_ROS_Interface::reset() or use the reset service.");
    return;
  }

  try {
    // current time, state, input, and subsystem
    const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

    // measure the delay in running MPC
    mpcTimer_.startTimer();

    // run MPC
    bool controllerIsUpdated = false;
    try {
      controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
    } catch (...) {
      mpcReady_ = false;
      readyToPublish_ = false;
      throw;
    }
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
    const auto& performanceIndices = mpc_.getSolverPtr()->getPerformanceIndeces();
    diagnostic_msgs::msg::DiagnosticStatus solverStatus;
    solverStatus.name = topicPrefix_ + "/mpc_solver";
    solverStatus.hardware_id = topicPrefix_;
    solverStatus.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    solverStatus.message = "ok";
    const auto addValue = [&solverStatus](const std::string& key,
                                          const std::string& value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      solverStatus.values.push_back(std::move(kv));
    };
    solverStatus.values.reserve(12);
    addValue("solve_time_last_ms",
             std::to_string(mpcTimer_.getLastIntervalInMilliseconds()));
    addValue("solve_time_avg_ms",
             std::to_string(mpcTimer_.getAverageInMilliseconds()));
    addValue("solve_time_max_ms",
             std::to_string(mpcTimer_.getMaxIntervalInMilliseconds()));
    addValue("solve_time_samples",
             std::to_string(mpcTimer_.getNumTimedIntervals()));
    addValue("ddp_iterations_used",
             std::to_string(mpc_.getSolverPtr()->getNumIterations()));
    addValue("perf_merit", std::to_string(performanceIndices.merit));
    addValue("perf_cost", std::to_string(performanceIndices.cost));
    addValue("perf_dual_feas_sse",
             std::to_string(performanceIndices.dualFeasibilitiesSSE));
    addValue("perf_dynamics_violation_sse",
             std::to_string(performanceIndices.dynamicsViolationSSE));
    addValue("perf_eq_constraints_sse",
             std::to_string(performanceIndices.equalityConstraintsSSE));
    addValue("perf_ineq_constraints_sse",
             std::to_string(performanceIndices.inequalityConstraintsSSE));
    addValue("perf_eq_lagrangian",
             std::to_string(performanceIndices.equalityLagrangian));
    addValue("perf_ineq_lagrangian",
             std::to_string(performanceIndices.inequalityLagrangian));
    if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) {
      solverStatus.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      solverStatus.message =
          "solution time window might be shorter than the MPC delay";
      std::cerr << "WARNING: The solution time window might be shorter than the "
                   "MPC delay!\n";
    }
    diagnostic_msgs::msg::DiagnosticArray solverDiagnosticsMsg;
    solverDiagnosticsMsg.header.stamp = node_->now();
    solverDiagnosticsMsg.status.push_back(std::move(solverStatus));
    mpcSolverDiagnosticsPublisher_->publish(std::move(solverDiagnosticsMsg));

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
                           *bufferPerformanceIndicesPtr_, bufferResetEpoch_,
                           bufferPolicySequence_);
    mpcPolicyPublisher_->publish(mpcPolicyMsg);
#endif
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "[MPC_ROS_Interface] Observation callback failed: "
                            << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "[MPC_ROS_Interface] Observation callback failed: unknown exception");
  }
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
  if (!node_) {
    throw std::runtime_error(
        "[MPC_ROS_Interface::spin] launchNodes() must be called before spin().");
  }
  rclcpp::spin(node_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ROS_Interface::launchNodes(const rclcpp::Node::SharedPtr& node) {
  RCLCPP_INFO(LOGGER, "MPC node is setting up ...");
  if (!node) {
    throw std::runtime_error(
        "[MPC_ROS_Interface::launchNodes] node is null.");
  }
  node_ = node;

  // Observation subscriber
  mpcObservationSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
          topicPrefix_ + "_mpc_observation", 1,
          std::bind(&MPC_ROS_Interface::mpcObservationCallback, this,
                    std::placeholders::_1));

  // MPC publisher
  const auto latchedQos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  mpcPolicyPublisher_ =
      node_->create_publisher<ocs2_msgs::msg::MpcFlattenedController>(
          topicPrefix_ + "_mpc_policy", latchedQos);
  mpcSolverDiagnosticsPublisher_ =
      node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          topicPrefix_ + "_mpc_solver_diagnostics", 10);

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
