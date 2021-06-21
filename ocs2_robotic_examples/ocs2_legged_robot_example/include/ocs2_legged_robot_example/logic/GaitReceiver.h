#pragma once

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ros/ros.h>
#include <mutex>

#include <ocs2_legged_robot_example/common/definitions.h>
#include <ocs2_legged_robot_example/logic/GaitSchedule.h>
#include <ocs2_legged_robot_example/logic/ModeSequenceTemplate.h>
#include <ocs2_legged_robot_example/logic/MotionPhaseDefinition.h>

namespace ocs2 {
namespace legged_robot {
class GaitReceiver : public ocs2::SolverSynchronizedModule {
 public:
  GaitReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<GaitSchedule> gaitSchedulePtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void mpcModeSequenceCallback(const ocs2_msgs::mode_schedule::ConstPtr& msg);

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;

  ros::Subscriber mpcModeSequenceSubscriber_;

  std::mutex receivedGaitMutex_;
  std::atomic_bool gaitUpdated_;
  ModeSequenceTemplate receivedGait_;
};

}  // namespace legged_robot
}  // namespace ocs2
