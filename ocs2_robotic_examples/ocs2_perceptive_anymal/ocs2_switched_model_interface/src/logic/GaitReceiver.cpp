//
// Created by rgrandia on 16.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitReceiver.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"
#include "ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h"

namespace switched_model {

GaitReceiver::GaitReceiver(const rclcpp::Node::SharedPtr& node,
                           ocs2::Synchronized<GaitSchedule>& gaitSchedule,
                           const std::string& robotName)
    : gaitSchedulePtr_(&gaitSchedule), gaitUpdated_(false) {
  mpcModeSequenceSubscriber_ =
      node->create_subscription<ocs2_msgs::msg::ModeSchedule>(
          robotName + "_mpc_mode_schedule", 1,
          std::bind(&GaitReceiver::mpcModeSequenceCallback, this,
                    std::placeholders::_1));
  mpcScheduledModeSequenceSubscriber_ =
      node->create_subscription<ocs2_msgs::msg::ModeSchedule>(
          robotName + "_mpc_scheduled_mode_schedule", 1,
          std::bind(&GaitReceiver::mpcModeScheduledGaitCallback, this,
                    std::placeholders::_1));
  mpcGaitSequenceSubscriber_ = node->create_subscription<
      ocs2_switched_model_msgs::msg::ScheduledGaitSequence>(
      robotName + "_mpc_gait_schedule", 1,
      std::bind(&GaitReceiver::mpcGaitSequenceCallback, this,
                std::placeholders::_1));
}

void GaitReceiver::preSolverRun(
    scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
    const ocs2::ReferenceManagerInterface& referenceManager) {
  if (gaitUpdated_) {
    {
      std::lock_guard<std::mutex> lock(receivedGaitMutex_);
      setGaitAction_(initTime, finalTime, currentState,
                     referenceManager.getTargetTrajectories());
    }
    std::cout << std::endl;
    gaitUpdated_ = false;
  }
}

void GaitReceiver::mpcModeSequenceCallback(
    const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr& msg) {
  const auto ModeSequenceTemplate = readModeSequenceTemplateMsg(*msg);
  const auto gait = toGait(ModeSequenceTemplate);

  {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime,
                         const vector_t& currentState,
                         const ocs2::TargetTrajectories& targetTrajectories) {
      std::cout << "[GaitReceiver]: Setting new gait after time " << finalTime
                << "\n[GaitReceiver]: " << gait;
      this->gaitSchedulePtr_->lock()->setGaitAfterTime(gait, finalTime);
    };
    gaitUpdated_ = true;
  }
}

void GaitReceiver::mpcModeScheduledGaitCallback(
    const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr& msg) {
  const auto ModeSequenceTemplate = readModeSequenceTemplateMsg(*msg);
  const auto gait = toGait(ModeSequenceTemplate);
  const auto scheduledGaitTime = ModeSequenceTemplate.switchingTimes.front();

  std::cout << "ScheduledGaitCallback:\n";
  std::cout << "\nReceivedGait:\n" << gait << "\n";

  {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime,
                         const vector_t& currentState,
                         const ocs2::TargetTrajectories& targetTrajectories) {
      std::cout
          << "[GaitReceiver]: Received new scheduled gait, setting it at time "
          << scheduledGaitTime << ", current time: " << initTime
          << "\n[GaitReceiver]: " << gait;
      this->gaitSchedulePtr_->lock()->setGaitAtTime(gait, scheduledGaitTime);
    };
    gaitUpdated_ = true;
  }
}

void GaitReceiver::mpcGaitSequenceCallback(
    const ocs2_switched_model_msgs::msg::ScheduledGaitSequence::ConstSharedPtr&
        msg) {
  const auto scheduledGaitSequence = ros_msg_conversions::fromMessage(*msg);

  std::cout << "ScheduledGaitCallback:\n";

  {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime,
                         const vector_t& currentState,
                         const ocs2::TargetTrajectories& targetTrajectories) {
      this->gaitSchedulePtr_->lock()->setGaitSequenceAtTime(
          scheduledGaitSequence.second, scheduledGaitSequence.first);
    };
    gaitUpdated_ = true;
  }
}

}  // namespace switched_model
