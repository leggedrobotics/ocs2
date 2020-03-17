//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const ComModelBase<double>& comModel,
                                               const KinematicsModelBase<double>& kinematicsModel,
                                               std::shared_ptr<const ocs2::HybridLogicRules> logicRulesPtr)
    : settings_(settings), comModel_(comModel.clone()), kinematicsModel_(kinematicsModel.clone()), logicRulesPtr_(logicRulesPtr) {}

void SwingTrajectoryPlanner::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                          const ocs2::CostDesiredTrajectories& costDesiredTrajectory,
                                          const ocs2::ModeSchedule& modeSchedule) {
  const auto basePose = comModel_->calculateBasePose(getComPose(currentState));
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  const auto& modeSchedule_ = logicRulesPtr_->getModeSchedule();
  assert(modeSchedule_.eventTimes().empty() || modeSchedule_.eventTimes().front() >= initTime);

  updateFeetTrajectories(initTime, finalTime, feetPositions, modeSchedule_);
  updateErrorTrajectories(initTime, feetPositions, modeSchedule_);

  std::cout << "[SwingTrajectoryPlanner]\n";
  std::cout << "Last contact:\n";
  std::cout << "0:\t t:" << lastContacts_[0].time << " h: " << lastContacts_[0].height << "\n";
  std::cout << "1:\t t:" << lastContacts_[1].time << " h: " << lastContacts_[1].height << "\n";
  std::cout << "2:\t t:" << lastContacts_[2].time << " h: " << lastContacts_[2].height << "\n";
  std::cout << "3:\t t:" << lastContacts_[3].time << " h: " << lastContacts_[3].height << "\n";
  std::cout << std::endl;
}

SwingTrajectoryPlanner::scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  const auto feedforwardVelocity = feetHeightTrajectories_[leg][index].velocity(time);
  // Feedback follows the planned trajectory s.t. de = - errorGain * e, with e = (z - z*)
  const auto feedbackVelocity = -settings_.errorGain * initialErrors_[leg] * std::exp(-settings_.errorGain * (time - initTime_));
  return feedforwardVelocity + feedbackVelocity;
}

void SwingTrajectoryPlanner::updateFeetTrajectories(scalar_t initTime, scalar_t finalTime,
                                                    const std::array<vector3_t, NUM_CONTACT_POINTS>& currentFeetPositions,
                                                    const ocs2::ModeSchedule& modeSchedule) {
  const auto terrainHeight = 0.0;

  std::cout << "ModeSchedule\n" << modeSchedule << std::endl;

  // Convert mode sequence to a contact flag vector per leg
  const std::array<std::vector<bool>, NUM_CONTACT_POINTS> contactSequencePerLeg =
      FeetPlannerBase::extractContactFlags(modeSchedule.modeSequence());

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    std::cout << "Leg : " << leg << std::endl;

    const auto footPhases = FeetPlannerBase::extractFootPhases(modeSchedule.eventTimes(), contactSequencePerLeg[leg]);
    auto& footTrajectory = feetHeightTrajectories_[leg];
    auto& footTrajectoryEvents = feetHeightTrajectoriesEvents_[leg];
    footTrajectory.clear();
    footTrajectoryEvents.clear();

    // If the first phase is a stance phase, register when it leaves contact.
    if (footPhases.front().type == FeetPlannerBase::FootPhaseType::Stance) {
      if (std::isnan(footPhases.front().endTime)) {
        lastContacts_[leg] = {finalTime, currentFeetPositions[leg].z()};
      } else {
        lastContacts_[leg] = {footPhases.front().endTime, currentFeetPositions[leg].z()};
      }
    }

    bool firstSwingPhase = true;
    for (const auto& footPhase : footPhases) {
      if (!std::isnan(footPhase.startTime) && footPhase.startTime >= finalTime) {
        // Phase starts after horizon, not need to plan a trajectory for it.
        break;
      }

      if (footPhase.type == FeetPlannerBase::FootPhaseType::Stance) {
        const auto startPoint = [&] {
          SplineCpg::Point point;
          if (std::isnan(footPhase.startTime)) {
            point.time = initTime;
            point.height = currentFeetPositions[leg].z();
          } else {
            point.time = footPhase.startTime;
            point.height = terrainHeight;
          }
          return point;
        }();

        const auto endPoint = [&] {
          SplineCpg::Point point;
          if (std::isnan(footPhase.endTime)) {
            point.time = finalTime;
          } else {
            point.time = footPhase.endTime;
          }
          point.height = startPoint.height;
          return point;
        }();

        footTrajectory.emplace_back(SplineCpg({0.0, 0.0}));
        footTrajectory.back().set(startPoint, endPoint, startPoint.height);
        footTrajectoryEvents.push_back(endPoint.time);

        std::cout << "\tstance\n";
        std::cout << "\tstart: \t t:" << startPoint.time << " h: " << startPoint.height << " v: " << 0.0 << "\n";
        std::cout << "\tmid:   \t h: " << startPoint.height << "\n";
        std::cout << "\tend:   \t t:" << endPoint.time << " h: " << endPoint.height << " v: " << 0.0 << "\n\n";
      } else if (footPhase.type == FeetPlannerBase::FootPhaseType::Swing) {
        const auto liftOff = [&] {
          CubicSpline::Node node;
          if (std::isnan(footPhase.startTime)) {
            node.time = lastContacts_[leg].time;
          } else {
            node.time = footPhase.startTime;
          }
          if (firstSwingPhase) {
            node.position = lastContacts_[leg].height;
            firstSwingPhase = false;
          } else {
            node.position = terrainHeight;
          }
          node.velocity = settings_.liftOffVelocity;
          return node;
        }();

        const auto touchDown = [&] {
          CubicSpline::Node node;
          if (std::isnan(footPhase.endTime)) {
            node.time = finalTime + settings_.touchdownAfterHorizon;
            node.position = terrainHeight + settings_.swingHeight;
            node.velocity = 0.0;
          } else {
            node.time = footPhase.endTime;
            node.position = terrainHeight;
            node.velocity = settings_.touchDownVelocity;
          }
          return node;
        }();

        const double scaling = std::min(1.0, (touchDown.time - liftOff.time) / settings_.swingTimeScale);
        const SplineCpg::Point startPoint{liftOff.time, liftOff.position};
        const SplineCpg::Point endPoint{touchDown.time, touchDown.position};
        footTrajectory.emplace_back(SplineCpg({scaling * liftOff.velocity, scaling * touchDown.velocity}));
        footTrajectory.back().set(startPoint, endPoint, scaling * (terrainHeight + settings_.swingHeight));
        footTrajectoryEvents.push_back(endPoint.time);

        std::cout << "\tswing\n";
        std::cout << "\tstart: \t t:" << startPoint.time << " h: " << startPoint.height << " v: " << scaling * liftOff.velocity << "\n";
        std::cout << "\tmid:   \t h: " << scaling * (terrainHeight + settings_.swingHeight) << " scaling: " << scaling << "\n";
        std::cout << "\tend:   \t t:" << endPoint.time << " h: " << endPoint.height << " v: " << scaling * touchDown.velocity << "\n\n";
      }
    }
  }
}

void SwingTrajectoryPlanner::updateErrorTrajectories(scalar_t initTime,
                                                     const std::array<vector3_t, NUM_CONTACT_POINTS>& currentFeetPositions,
                                                     const ocs2::ModeSchedule& modeSchedule) {
  const auto index = ocs2::lookup::findIndexInTimeArray(modeSchedule.eventTimes(), initTime);
  initTime_ = initTime;
  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    initialErrors_[leg] = currentFeetPositions[leg].z() - feetHeightTrajectories_[leg][index].position(initTime);
  }
}

}  // namespace switched_model
