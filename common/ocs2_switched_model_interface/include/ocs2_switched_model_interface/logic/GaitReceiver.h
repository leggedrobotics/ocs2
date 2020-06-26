//
// Created by rgrandia on 18.03.20.
//

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"
#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"

namespace switched_model {

class GaitReceiver : public ocs2::SolverSynchronizedModule {
 public:
  GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override;

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override{};

 private:
  void mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg);

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;

  ros::Subscriber mpcModeSequenceSubscriber_;

  std::mutex receivedGaitMutex_;
  std::atomic_bool gaitUpdated_;
  ModeSequenceTemplate receivedGait_;
};

}  // namespace switched_model
