//
// Created by rgrandia on 16.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitReceiver.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"

namespace switched_model {

GaitReceiver::GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<LockableGaitSchedule> gaitSchedulePtr, const std::string& robotName)
    : gaitSchedulePtr_(std::move(gaitSchedulePtr)), gaitUpdated_(false) {
  mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &GaitReceiver::mpcModeSequenceCallback, this,
                                                    ::ros::TransportHints().udp());
}

void GaitReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  if (gaitUpdated_) {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    std::lock_guard<LockableGaitSchedule> gaitLock(*gaitSchedulePtr_);
    std::cout << "[GaitReceiver]: Setting new gait after time " << finalTime << std::endl;
    gaitSchedulePtr_->setGaitAfterTime(receivedGait_, finalTime);
    gaitUpdated_ = false;
  }
}

void GaitReceiver::mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg) {
  auto modeSequenceTemplate = readModeSequenceTemplateMsg(*msg);
  const auto receivedGait = [&] {
    Gait gait;
    gait.duration = modeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                  [&](scalar_t eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = modeSequenceTemplate.modeSequence;
    return gait;
  }();

  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  receivedGait_ = receivedGait;
  gaitUpdated_ = true;
}

}  // namespace switched_model
