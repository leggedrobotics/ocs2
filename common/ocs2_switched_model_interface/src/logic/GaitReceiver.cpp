//
// Created by rgrandia on 16.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitReceiver.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"
#include "ocs2_switched_model_ros_interfaces/RosMsgConversions.h"

namespace switched_model {

GaitReceiver::GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<LockableGaitSchedule> gaitSchedulePtr, const std::string& robotName)
    : gaitSchedulePtr_(std::move(gaitSchedulePtr)), gaitUpdated_(false) {
  mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_mode_schedule", 1, &GaitReceiver::mpcModeSequenceCallback, this,
                                                    ::ros::TransportHints().udp());
  mpcScheduledGaitSubscriber_ = nodeHandle.subscribe(robotName + "_mpc_scheduled_mode_schedule", 1, &GaitReceiver::mpcScheduledGaitCallback,
                                                     this, ::ros::TransportHints().udp());
}

void GaitReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  if (gaitUpdated_) {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    {
      std::lock_guard<LockableGaitSchedule> gaitLock(*gaitSchedulePtr_);
      setGaitAction_(initTime, finalTime, currentState, costDesiredTrajectory);
    }
    gaitUpdated_ = false;
    std::cout << std::endl;
  }
}

void GaitReceiver::mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg) {
  auto modeSequenceTemplate = readModeSequenceTemplateMsg(*msg);
  Gait gait;
  gait.duration = modeSequenceTemplate.switchingTimes.back();
  // Events: from time -> phase
  std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                [&](scalar_t eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
  // Modes:
  gait.modeSequence = modeSequenceTemplate.modeSequence;

  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                       const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
    std::cout << "[GaitReceiver]: Setting new gait after time " << finalTime << "\n[GaitReceiver]: " << gait;
    gaitSchedulePtr_->setGaitAfterTime(gait, finalTime);
  };
  gaitUpdated_ = true;
}

void GaitReceiver::mpcScheduledGaitCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg) {
  auto modeSequenceTemplate = readModeSequenceTemplateMsg(*msg);
  std::cout << "ScheduledGaitCallback:\n";
  Gait gait;
  gait.duration = modeSequenceTemplate.switchingTimes.back();

  // Events: from time -> phase
  std::for_each(modeSequenceTemplate.switchingTimes.begin() + 1, modeSequenceTemplate.switchingTimes.end() - 1,
                [&](scalar_t eventTime) { gait.eventPhases.push_back((eventTime) / gait.duration); });

  // Modes:
  gait.modeSequence = modeSequenceTemplate.modeSequence;
  std::cout << "\nReceivedGait:\n" << gait << "\n";

  setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                       const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
    std::cout << "[GaitReceiver]: Received new scheduled gait, setting it at time " << modeSequenceTemplate.switchingTimes.front()
              << ", current time: " << initTime << "\n[GaitReceiver]: " << gait;
    gaitSchedulePtr_->setGaitAtTime(gait, modeSequenceTemplate.switchingTimes.front());
  };
  gaitUpdated_ = true;
}

void GaitReceiver::gaitSequenceCallback(const switched_model_msgs::gait_sequence_::ConstPtr& msg) {
  std::pair<GaitSequence&, std::vector<ocs2::scalar_t>&> scheduledGaitSequence;
  switched_model::ros_msg_conversions::readGaitSequenceMsg(*msg, scheduledGaitSequence.first, scheduledGaitSequence.second);

  std::cout << "ScheduledGaitCallback:\n";
  std::cout << *msg << modeSequenceTemplate << std::endl;

  // For terrain offsetting
  // const auto HEIGHT_STATE_IDX = 5;
  // ocs2::SystemObservation<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM> observation =
  //   ocs2MotionPlanner_->getCurrentObservation();
  // for (auto& state : desiredTrajectory_.desiredStateTrajectory()) {
  //   state[HEIGHT_STATE_IDX] += -ocs2TerrainPerception_->getHeightInTerrainFrame();
  // }

  {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    setGaitAction_ = [=](scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                         const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
      const auto& gaitSequence = scheduledGaitSequence.first;
      const auto& startTimes = scheduledGaitSequence.second;
      assert(gaitSequence.size() == startTimes.size());
      for (auto i = 0; i < gaitSequence.size(); ++i) {
        gaitSchedulePtr_->setGaitSequenceAtTime({gaitSequence[i]}, startTimes[i]);
      }
    };
    gaitUpdated_ = true;
  }
}

}  // namespace switched_model
