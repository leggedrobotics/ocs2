//
// Created by rgrandia on 18.03.20.
//

#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <mutex>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_switched_model_msgs/msg/scheduled_gait_sequence.hpp>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"
#include "rclcpp/rclcpp.hpp"

namespace switched_model {

class GaitReceiver : public ocs2::SolverSynchronizedModule {
 public:
  GaitReceiver(const rclcpp::Node::SharedPtr& node,
               ocs2::Synchronized<GaitSchedule>& gaitSchedule,
               const std::string& robotName);

  void preSolverRun(
      scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
      const ocs2::ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override{};

 private:
  void mpcModeSequenceCallback(
      const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr& msg);
  void mpcModeScheduledGaitCallback(
      const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr& msg);
  void mpcGaitSequenceCallback(
      const ocs2_switched_model_msgs::msg::ScheduledGaitSequence::ConstSharedPtr&
          msg);

  rclcpp::Subscription<ocs2_msgs::msg::ModeSchedule>::SharedPtr
      mpcModeSequenceSubscriber_;
  rclcpp::Subscription<ocs2_msgs::msg::ModeSchedule>::SharedPtr
      mpcScheduledModeSequenceSubscriber_;
  rclcpp::Subscription<ocs2_switched_model_msgs::msg::ScheduledGaitSequence>::
      SharedPtr mpcGaitSequenceSubscriber_;

  ocs2::Synchronized<GaitSchedule>* gaitSchedulePtr_;

  std::atomic_bool gaitUpdated_;

  std::mutex receivedGaitMutex_;  // protects the setGaitAction_ variable
  std::function<void(scalar_t initTime, scalar_t finalTime,
                     const vector_t& currentState,
                     const ocs2::TargetTrajectories& targetTrajectories)>
      setGaitAction_;
};

}  // namespace switched_model
