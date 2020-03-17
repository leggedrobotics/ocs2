//
// Created by rgrandia on 16.03.20.
//

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_msgs/mode_sequence.h>

#include <ocs2_core/logic/rules/HybridLogicRules.h>

#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"

namespace switched_model {

class GaitReceiver : public ocs2::SolverSynchronizedModule<STATE_DIM, INPUT_DIM> {
 public:
  GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<ocs2::HybridLogicRules> logicRulesPtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory, const ocs2::ModeSchedule& modeSchedule) override;

  void postSolverRun(const primal_solution_t& primalSolution) override{};

 private:
  void mpcModeSequenceCallback(const ocs2_msgs::mode_sequence::ConstPtr& msg);

  std::shared_ptr<ocs2::HybridLogicRules> logicRulesPtr_;

  GaitSchedule gaitSchedule_;
  ros::Subscriber mpcModeSequenceSubscriber_;

  std::mutex receivedGaitMutex_;
  Gait receivedGait_;
  bool gaitUpdated_;
};

}  // namespace switched_model
