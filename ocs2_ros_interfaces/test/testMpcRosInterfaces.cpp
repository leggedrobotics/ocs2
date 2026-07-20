#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

namespace ocs2 {
namespace {

class MrtRosInterfaceAccess : public MRT_ROS_Interface {
 public:
  static void validate(uint64_t expectedResetEpoch, uint64_t lastPolicySequence,
                       uint64_t messageResetEpoch, uint64_t messagePolicySequence) {
    validatePolicyGeneration(expectedResetEpoch, lastPolicySequence,
                             messageResetEpoch, messagePolicySequence);
  }

  static void read(const ocs2_msgs::msg::MpcFlattenedController& msg, CommandData& commandData,
                   PrimalSolution& primalSolution, PerformanceIndex& performanceIndex) {
    readPolicyMsg(msg, commandData, primalSolution, performanceIndex);
  }
};

class MpcRosInterfaceAccess : public MPC_ROS_Interface {
 public:
  using MPC_ROS_Interface::MPC_ROS_Interface;

  static ocs2_msgs::msg::MpcFlattenedController create(const PrimalSolution& primalSolution,
                                                       const CommandData& commandData,
                                                       const PerformanceIndex& performanceIndex,
                                                       uint64_t resetEpoch, uint64_t policySequence) {
    return createMpcPolicyMsg(primalSolution, commandData, performanceIndex, resetEpoch, policySequence);
  }

  void configurePolicyPublisher(const rclcpp::Node::SharedPtr& node, const std::string& topic) {
    node_ = node;
    mpcPolicyPublisher_ = node->create_publisher<ocs2_msgs::msg::MpcFlattenedController>(topic, 10);
  }

  void setCurrentGeneration(uint64_t resetEpoch) {
    std::lock_guard<std::mutex> lock(resetMutex_);
    resetEpoch_ = resetEpoch;
    policySequence_ = 0;
    mpcReady_ = true;
  }

  void stagePolicy(PrimalSolution primalSolution, const CommandData& commandData,
                   uint64_t resetEpoch, uint64_t policySequence) {
    std::lock_guard<std::mutex> lock(bufferMutex_);
    *bufferPrimalSolutionPtr_ = std::move(primalSolution);
    *bufferCommandPtr_ = commandData;
    *bufferPerformanceIndicesPtr_ = PerformanceIndex{};
    bufferResetEpoch_ = resetEpoch;
    bufferPolicySequence_ = policySequence;
  }

  void requestPublish() {
    {
      std::lock_guard<std::mutex> lock(publisherMutex_);
      readyToPublish_ = true;
    }
    msgReady_.notify_one();
  }
};

class InertMpc final : public MPC_BASE {
 public:
  InertMpc() : MPC_BASE(mpc::Settings{}) {}

  SolverBase* getSolverPtr() override { return nullptr; }
  const SolverBase* getSolverPtr() const override { return nullptr; }

 private:
  void calculateController(scalar_t, const vector_t&, scalar_t) override {}
};

struct BlockingFlattenState final {
  std::mutex mutex;
  std::condition_variable condition;
  bool started = false;
  bool release = false;
};

class BlockingController final : public ControllerBase {
 public:
  explicit BlockingController(std::shared_ptr<BlockingFlattenState> state) : state_(std::move(state)) {}

  vector_t computeInput(scalar_t, const vector_t&) override { return vector_t::Constant(1, 0.1); }
  void concatenate(const ControllerBase*, int, int) override {}
  int size() const override { return 2; }
  ControllerType getType() const override { return ControllerType::FEEDFORWARD; }
  void clear() override {}
  bool empty() const override { return false; }
  BlockingController* clone() const override { return new BlockingController(state_); }

  void flatten(const scalar_array_t&, const std::vector<std::vector<float>*>& data) const override {
    {
      std::unique_lock<std::mutex> lock(state_->mutex);
      state_->started = true;
      state_->condition.notify_all();
      state_->condition.wait(lock, [&]() { return state_->release; });
    }
    for (auto* sample : data) {
      *sample = {0.1F};
    }
  }

 private:
  std::shared_ptr<BlockingFlattenState> state_;
};

ocs2_msgs::msg::MpcObservation makeObservationMsg() {
  ocs2_msgs::msg::MpcObservation msg;
  msg.time = 1.0;
  msg.state.value = {0.25F};
  msg.input.value = {0.1F};
  msg.mode = 0;
  return msg;
}

ocs2_msgs::msg::MpcTargetTrajectories makeTargetMsg() {
  ocs2_msgs::msg::MpcTargetTrajectories msg;
  msg.time_trajectory = {1.0, 2.0};
  msg.state_trajectory.resize(2);
  msg.input_trajectory.resize(2);
  msg.state_trajectory[0].value = {0.0F};
  msg.state_trajectory[1].value = {1.0F};
  msg.input_trajectory[0].value = {0.0F};
  msg.input_trajectory[1].value = {0.0F};
  return msg;
}

ocs2_msgs::msg::MpcFlattenedController makePolicyMsg() {
  ocs2_msgs::msg::MpcFlattenedController msg;
  msg.reset_epoch = 3;
  msg.policy_sequence = 7;
  msg.controller_type = ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD;
  msg.init_observation = makeObservationMsg();
  msg.plan_target_trajectories = makeTargetMsg();
  msg.mode_schedule.mode_sequence = {0};
  msg.performance_indices.init_time = 1.0F;
  msg.time_trajectory = {1.0, 2.0};
  msg.state_trajectory.resize(2);
  msg.input_trajectory.resize(2);
  msg.data.resize(2);
  for (size_t i = 0; i < 2; ++i) {
    msg.state_trajectory[i].value = {static_cast<float>(i)};
    msg.input_trajectory[i].value = {0.1F};
    msg.data[i].data = {0.1F};
  }
  return msg;
}

PrimalSolution makePrimalSolution() {
  PrimalSolution solution;
  solution.timeTrajectory_ = {1.0, 2.0};
  solution.stateTrajectory_ = {vector_t::Zero(1), vector_t::Ones(1)};
  solution.inputTrajectory_ = {vector_t::Constant(1, 0.1), vector_t::Constant(1, 0.1)};
  solution.modeSchedule_ = ModeSchedule({}, {0});
  solution.controllerPtr_ =
      std::make_unique<FeedforwardController>(solution.timeTrajectory_, solution.inputTrajectory_);
  return solution;
}

PrimalSolution makeBlockingPrimalSolution(const std::shared_ptr<BlockingFlattenState>& state) {
  auto solution = makePrimalSolution();
  solution.controllerPtr_ = std::make_unique<BlockingController>(state);
  return solution;
}

CommandData makeCommandData() {
  CommandData command;
  command.mpcInitObservation_.time = 1.0;
  command.mpcInitObservation_.state = vector_t::Constant(1, 0.25);
  command.mpcInitObservation_.input = vector_t::Constant(1, 0.1);
  command.mpcTargetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(makeTargetMsg());
  return command;
}

TEST(PolicyGeneration, acceptsOnlyCurrentIncreasingSequence) {
  EXPECT_NO_THROW(MrtRosInterfaceAccess::validate(4, 0, 4, 1));
  EXPECT_NO_THROW(MrtRosInterfaceAccess::validate(4, 7, 4, 8));

  EXPECT_THROW(MrtRosInterfaceAccess::validate(0, 0, 0, 1), std::runtime_error);
  EXPECT_THROW(MrtRosInterfaceAccess::validate(4, 0, 3, 1), std::runtime_error);
  EXPECT_THROW(MrtRosInterfaceAccess::validate(4, 0, 5, 1), std::runtime_error);
  EXPECT_THROW(MrtRosInterfaceAccess::validate(4, 0, 4, 0), std::runtime_error);
  EXPECT_THROW(MrtRosInterfaceAccess::validate(4, 7, 4, 7), std::runtime_error);
  EXPECT_THROW(MrtRosInterfaceAccess::validate(4, 7, 4, 6), std::runtime_error);
}

TEST(PolicyGeneration, rejectsPreviousEpochAcrossRandomizedResetPublishOrdering) {
  std::mt19937 random(0x5A17U);
  uint64_t currentEpoch = 1;
  for (size_t iteration = 0; iteration < 1000; ++iteration) {
    const uint64_t previousEpoch = currentEpoch++;
    uint64_t lastAcceptedSequence = 0;
    const auto validateCurrent = [&]() {
      EXPECT_NO_THROW(MrtRosInterfaceAccess::validate(currentEpoch, lastAcceptedSequence, currentEpoch, 1));
      lastAcceptedSequence = 1;
    };
    const auto rejectPrevious = [&]() {
      EXPECT_THROW(MrtRosInterfaceAccess::validate(currentEpoch, lastAcceptedSequence, previousEpoch, 1),
                   std::runtime_error);
    };

    if ((random() & 1U) == 0U) {
      rejectPrevious();
      validateCurrent();
    } else {
      validateCurrent();
      rejectPrevious();
    }
  }
}

TEST(PolicyGeneration, blockedPreResetPublisherCannotPublishAfterResetAcknowledgement) {
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  InertMpc mpc;
  MpcRosInterfaceAccess interface(mpc, "policy_race");
  auto publisherNode = std::make_shared<rclcpp::Node>("policy_race_publisher");
  auto subscriberNode = std::make_shared<rclcpp::Node>("policy_race_subscriber");
  const std::string topic = "policy_race_controller";
  interface.configurePolicyPublisher(publisherNode, topic);

  std::mutex messagesMutex;
  std::condition_variable messagesCondition;
  std::vector<ocs2_msgs::msg::MpcFlattenedController> messages;
  auto subscription = subscriberNode->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
      topic, 10, [&](const ocs2_msgs::msg::MpcFlattenedController& msg) {
        {
          std::lock_guard<std::mutex> lock(messagesMutex);
          messages.push_back(msg);
        }
        messagesCondition.notify_all();
      });
  (void)subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisherNode);
  executor.add_node(subscriberNode);
  std::thread spinThread([&]() { executor.spin(); });

  const auto discoveryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (publisherNode->count_subscribers(topic) == 0 && std::chrono::steady_clock::now() < discoveryDeadline) {
    std::this_thread::yield();
  }
  EXPECT_EQ(publisherNode->count_subscribers(topic), 1U);

  interface.setCurrentGeneration(1);
  const auto blockingState = std::make_shared<BlockingFlattenState>();
  interface.stagePolicy(makeBlockingPrimalSolution(blockingState), makeCommandData(), 1, 1);
  interface.requestPublish();

  bool flattenStarted = false;
  {
    std::unique_lock<std::mutex> lock(blockingState->mutex);
    flattenStarted = blockingState->condition.wait_for(lock, std::chrono::seconds(2), [&]() {
      return blockingState->started;
    });
  }
  EXPECT_TRUE(flattenStarted);

  // This models a successful reset acknowledgement while the old policy is still serializing.
  interface.setCurrentGeneration(2);
  {
    std::lock_guard<std::mutex> lock(blockingState->mutex);
    blockingState->release = true;
  }
  blockingState->condition.notify_all();

  interface.stagePolicy(makePrimalSolution(), makeCommandData(), 2, 1);
  interface.requestPublish();

  bool receivedCurrentPolicy = false;
  {
    std::unique_lock<std::mutex> lock(messagesMutex);
    receivedCurrentPolicy = messagesCondition.wait_for(lock, std::chrono::seconds(2), [&]() {
      return !messages.empty() && messages.back().reset_epoch == 2;
    });
  }

  interface.shutdownNode();
  executor.cancel();
  spinThread.join();

  EXPECT_TRUE(receivedCurrentPolicy);
  std::lock_guard<std::mutex> lock(messagesMutex);
  ASSERT_EQ(messages.size(), 1U);
  EXPECT_EQ(messages.front().reset_epoch, 2U);
  EXPECT_EQ(messages.front().policy_sequence, 1U);
}

TEST(RosMessageValidation, rejectsMalformedObservationAndTarget) {
  auto observation = makeObservationMsg();
  EXPECT_NO_THROW(ros_msg_conversions::readObservationMsg(observation));

  observation.state.value[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(ros_msg_conversions::readObservationMsg(observation), std::runtime_error);

  auto target = makeTargetMsg();
  EXPECT_NO_THROW(ros_msg_conversions::readTargetTrajectoriesMsg(target));

  target.time_trajectory[1] = target.time_trajectory[0] - 1.0;
  EXPECT_THROW(ros_msg_conversions::readTargetTrajectoriesMsg(target), std::runtime_error);

  target = makeTargetMsg();
  target.state_trajectory[1].value.push_back(2.0F);
  EXPECT_THROW(ros_msg_conversions::readTargetTrajectoriesMsg(target), std::runtime_error);
}

TEST(PolicyMessageValidation, acceptsValidPolicyAndRejectsMalformedFields) {
  auto readPolicy = [](const ocs2_msgs::msg::MpcFlattenedController& msg) {
    CommandData command;
    PrimalSolution solution;
    PerformanceIndex performance;
    MrtRosInterfaceAccess::read(msg, command, solution, performance);
    return solution;
  };

  auto policy = makePolicyMsg();
  const auto solution = readPolicy(policy);
  ASSERT_EQ(solution.timeTrajectory_.size(), 2);
  ASSERT_NE(solution.controllerPtr_, nullptr);

  policy = makePolicyMsg();
  policy.time_trajectory[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(readPolicy(policy), std::runtime_error);

  policy = makePolicyMsg();
  policy.data[0].data.clear();
  EXPECT_THROW(readPolicy(policy), std::runtime_error);

  policy = makePolicyMsg();
  policy.post_event_indices = {2};
  EXPECT_THROW(readPolicy(policy), std::runtime_error);

  policy = makePolicyMsg();
  policy.mode_schedule.mode_sequence.clear();
  EXPECT_THROW(readPolicy(policy), std::runtime_error);
}

TEST(PolicyMessageValidation, validatesPolicyBeforePublication) {
  auto solution = makePrimalSolution();
  const auto command = makeCommandData();
  const PerformanceIndex performance;

  const auto msg = MpcRosInterfaceAccess::create(solution, command, performance, 3, 7);
  EXPECT_EQ(msg.reset_epoch, 3);
  EXPECT_EQ(msg.policy_sequence, 7);

  EXPECT_THROW(MpcRosInterfaceAccess::create(solution, command, performance, 0, 7), std::runtime_error);

  solution.stateTrajectory_[0](0) = std::numeric_limits<scalar_t>::infinity();
  EXPECT_THROW(MpcRosInterfaceAccess::create(solution, command, performance, 3, 8), std::runtime_error);
}

}  // namespace
}  // namespace ocs2
