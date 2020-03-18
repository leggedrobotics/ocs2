//
// Created by rgrandia on 16.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitReceiver.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

GaitReceiver::GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string& robotName)
    : gaitSchedulePtr_(std::move(gaitSchedulePtr)), receivedGait_({0.0, 1.0}, {ModeNumber::STANCE}), gaitUpdated_(false) {
  mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &GaitReceiver::mpcModeSequenceCallback, this,
                                                    ::ros::TransportHints().udp());
}

void GaitReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  if (gaitUpdated_) {
    std::cout << "[GaitReceiver]: Setting new gait after time " << finalTime << "\n";
    std::cout << receivedGait_;
    const auto timeHorizon = finalTime - initTime;
    gaitSchedulePtr_->insertModeSequenceTemplate(receivedGait_, finalTime, timeHorizon);
    gaitUpdated_ = false;
  }
}

void GaitReceiver::mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  receivedGait_ = readModeSequenceTemplateMsg(*msg);
  gaitUpdated_ = true;
}

}  // namespace switched_model
