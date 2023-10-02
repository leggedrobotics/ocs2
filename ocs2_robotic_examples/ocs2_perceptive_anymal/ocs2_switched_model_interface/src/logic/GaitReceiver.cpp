//
// Created by rgrandia on 16.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitReceiver.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"
#include "ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h"

namespace switched_model {

GaitReceiver::GaitReceiver(ros::NodeHandle nodeHandle, ocs2::Synchronized<GaitSchedule>& gaitSchedule, const std::string& robotName)
    : gaitSchedulePtr_(&gaitSchedule), gaitUpdated_(false) {
  mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &GaitReceiver::mpcModeSequenceCallback, this,
                                                    ::ros::TransportHints().udp());
  mpcScheduledModeSequenceSubscriber_ = nodeHandle.subscribe(
      robotName + "_mpc_scheduled_mode_schedule", 1, &GaitReceiver::mpcModeScheduledGaitCallback, this, ::ros::TransportHints().udp());
  mpcGaitSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_gait_schedule", 1, &GaitReceiver::mpcGaitSequenceCallback, this,
                                                    ::ros::TransportHints().udp());
}

void GaitReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ocs2::ReferenceManagerInterface& referenceManager) {
  if (gaitUpdated_) {
    {
      std::lock_guard<std::mutex> lock(receivedGaitMutex_);
      setGaitAction_(initTime, finalTime, currentState, referenceManager.getTargetTrajectories());
    }
    std::cout << std::endl;
    gaitUpdated_ = false;
  }
}

void GaitReceiver::mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg) {
  const auto modeSequenceTemplate = readModeSequenceTemplateMsg(*msg);
  const auto gait = toGait(modeSequenceTemplate);

  {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                         const ocs2::TargetTrajectories& targetTrajectories) {
      std::cout << "[GaitReceiver]: Setting new gait after time " << finalTime << "\n[GaitReceiver]: " << gait;
      this->gaitSchedulePtr_->lock()->setGaitAfterTime(gait, finalTime);
    };
    gaitUpdated_ = true;
  }
}

void GaitReceiver::mpcModeScheduledGaitCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg) {
  const auto modeSequenceTemplate = readModeSequenceTemplateMsg(*msg);
  const auto gait = toGait(modeSequenceTemplate);
  const auto scheduledGaitTime = modeSequenceTemplate.switchingTimes.front();

  std::cout << "ScheduledGaitCallback:\n";
  std::cout << "\nReceivedGait:\n" << gait << "\n";

  {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                         const ocs2::TargetTrajectories& targetTrajectories) {
      std::cout << "[GaitReceiver]: Received new scheduled gait, setting it at time " << scheduledGaitTime << ", current time: " << initTime
                << "\n[GaitReceiver]: " << gait;
      this->gaitSchedulePtr_->lock()->setGaitAtTime(gait, scheduledGaitTime);
    };
    gaitUpdated_ = true;
  }
}

void GaitReceiver::mpcGaitSequenceCallback(const ocs2_switched_model_msgs::scheduled_gait_sequenceConstPtr& msg) {
  const auto scheduledGaitSequence = ros_msg_conversions::fromMessage(*msg);

  std::cout << "ScheduledGaitCallback:\n";
  std::cout << *msg << std::endl;

  {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                         const ocs2::TargetTrajectories& targetTrajectories) {
      this->gaitSchedulePtr_->lock()->setGaitSequenceAtTime(scheduledGaitSequence.second, scheduledGaitSequence.first);
    };
    gaitUpdated_ = true;
  }
}

}  // namespace switched_model
