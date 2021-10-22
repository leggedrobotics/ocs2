#include "ocs2_legged_robot_raisim/LeggedRobotRaisimVisualizer.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotRaisimVisualizer::LeggedRobotRaisimVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                                         const PinocchioEndEffectorKinematics& endEffectorKinematics,
                                                         ros::NodeHandle& nodeHandle, scalar_t maxUpdateFrequency,
                                                         raisim::HeightMap const* terrain)
    : LeggedRobotVisualizer(pinocchioInterface, centroidalModelInfo, endEffectorKinematics, nodeHandle, maxUpdateFrequency),
      terrain_(terrain) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotRaisimVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution,
                                         const CommandData& command) {
  SystemObservation raisimObservation = observation;
  PrimalSolution raisimPrimalSolution = primalSolution;
  CommandData raisimCommand = command;
  // height relative to terrain
  if (terrain_ != nullptr) {
    raisimObservation.state(8) += terrain_->getHeight(observation.state(6), observation.state(7));
    for (size_t i = 0; i < primalSolution.stateTrajectory_.size(); i++) {
      raisimPrimalSolution.stateTrajectory_[i](8) +=
          terrain_->getHeight(primalSolution.stateTrajectory_[i](6), primalSolution.stateTrajectory_[i](7));
    }
    raisimCommand.mpcInitObservation_.state(8) +=
        terrain_->getHeight(command.mpcInitObservation_.state(6), command.mpcInitObservation_.state(7));
    for (size_t i = 0; i < command.mpcTargetTrajectories_.stateTrajectory.size(); i++) {
      raisimCommand.mpcTargetTrajectories_.stateTrajectory[i](8) +=
          terrain_->getHeight(command.mpcTargetTrajectories_.stateTrajectory[i](6), command.mpcTargetTrajectories_.stateTrajectory[i](7));
    }
  }
  LeggedRobotVisualizer::update(raisimObservation, raisimPrimalSolution, raisimCommand);
}

}  // namespace legged_robot
}  // namespace ocs2
