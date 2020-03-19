//
// Created by rgrandia on 16.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitReceiver.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

GaitReceiver::GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string& robotName)
    : gaitSchedulePtr_(std::move(gaitSchedulePtr)), gaitUpdated_(false) {
  mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &GaitReceiver::mpcModeSequenceCallback, this,
                                                    ::ros::TransportHints().udp());
}

void GaitReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  if (gaitUpdated_) {
    std::cout << "[GaitReceiver]: Setting new gait after time " << finalTime << std::endl;
    gaitSchedule_.setGaitAfterTime(receivedGait_, finalTime);
    gaitUpdated_ = false;
  }
}

void GaitReceiver::mpcModeSequenceCallback(const ocs2_msgs::mode_sequence::ConstPtr& msg) {
  ocs2::ModeSequenceTemplate<double> modeSequenceTemplate;
  ocs2::RosMsgConversions<STATE_DIM, INPUT_DIM>::readModeSequenceTemplateMsg(*msg, modeSequenceTemplate);
  const auto receivedGait = [&] {
    Gait gait;
    gait.duration = modeSequenceTemplate.templateSwitchingTimes_.back();
    // Events: from time -> phase
    std::for_each(modeSequenceTemplate.templateSwitchingTimes_.begin() + 1, modeSequenceTemplate.templateSwitchingTimes_.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = modeSequenceTemplate.templateSubsystemsSequence_;
    return gait;
  }();

  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  receivedGait_ = receivedGait;
  gaitUpdated_ = true;
}

}  // namespace switched_model
