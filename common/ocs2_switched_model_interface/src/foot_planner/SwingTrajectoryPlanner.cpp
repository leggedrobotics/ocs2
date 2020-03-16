//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const ComModelBase<double>& comModel,
                                               const KinematicsModelBase<double>& kinematicsModel)
    : settings_(settings), comModel_(comModel.clone()), kinematicsModel_(kinematicsModel.clone()) {}

void SwingTrajectoryPlanner::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                          const ocs2::CostDesiredTrajectories& costDesiredTrajectory,
                                          const ocs2::ModeSchedule& modeSchedule) {
  const auto basePose = comModel_->calculateBasePose(getComPose(currentState));
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  updateLastContacts(initTime, feetPositions, modeSchedule);
  updateFeetTrajectories(finalTime, modeSchedule);
  updateErrorTrajectories(initTime, feetPositions, modeSchedule);

  std::cout << "[SwingTrajectoryPlanner]\n";
  std::cout << "Last contact:\n";
  std::cout << "0:\t t:" << lastContacts_[0].time << " h: " << lastContacts_[0].height << "\n";
  std::cout << "1:\t t:" << lastContacts_[1].time << " h: " << lastContacts_[1].height << "\n";
  std::cout << "2:\t t:" << lastContacts_[2].time << " h: " << lastContacts_[2].height << "\n";
  std::cout << "3:\t t:" << lastContacts_[3].time << " h: " << lastContacts_[3].height << "\n";
  std::cout << std::endl;
}

SwingTrajectoryPlanner::scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(eventTimes_, time);
  const auto feedforwardVelocity = feetHeightTrajectories_[leg][index].velocity(time);
  // Feedback follows the planned trajectory s.t. de = - errorGain * e, with e = (z - z*)
  const auto feedbackVelocity = -settings_.errorGain * initialErrors_[leg] * std::exp(-settings_.errorGain * (time - initTime_));
  return feedforwardVelocity + feedbackVelocity;
}

void SwingTrajectoryPlanner::updateLastContacts(scalar_t time, const std::array<vector3_t, NUM_CONTACT_POINTS>& currentFeetPositions,
                                                const ocs2::ModeSchedule& modeSchedule) {
  const auto index = ocs2::lookup::findIndexInTimeArray(modeSchedule.eventTimes(), time);
  const auto contactFlags = modeNumber2StanceLeg(modeSchedule.modeSequence()[index]);

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    if (contactFlags[leg]) {
      lastContacts_[leg] = {time, currentFeetPositions[leg].z()};
    }
  }
}

void SwingTrajectoryPlanner::updateFeetTrajectories(scalar_t finalTime, const ocs2::ModeSchedule& modeSchedule) {
  const int numPhases = modeSchedule.modeSequence().size();
  eventTimes_ = modeSchedule.eventTimes();

  // Convert mode sequence to a contact flag vector per leg
  const std::array<std::vector<bool>, NUM_CONTACT_POINTS> contactSequencePerLeg =
      FeetPlannerBase::extractContactFlags(modeSchedule.modeSequence());

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    const auto& contactSequence = contactSequencePerLeg[leg];
    const auto& terrainHeight = lastContacts_[leg].height;
    auto& footTrajectory = feetHeightTrajectories_[leg];
    footTrajectory.clear();

    for (int phase = 0; phase < numPhases; phase++) {
      if (!contactSequence[phase]) {  // swingphase
        // Extract event time indices
        int liftoffIndex;
        int touchdownIndex;
        std::tie(liftoffIndex, touchdownIndex) = FeetPlannerBase::findIndex(phase, contactSequence);

        // Determine start of swing phase
        const auto startTime = (liftoffIndex < 0) ? lastContacts_[leg].time : eventTimes_[liftoffIndex];
        const SplineCpg::Point startPoint{startTime, terrainHeight};

        // Determine end of swing phase
        const auto endPoint = [&] {
          if (touchdownIndex + 1 < numPhases) {
            footTrajectory.emplace_back(SplineCpg({settings_.liftOffVelocity, settings_.touchDownVelocity}));
            return SplineCpg::Point{eventTimes_[touchdownIndex], terrainHeight};
          } else {  // touchdown is after the current mode sequence
            std::cout << "[SwingTrajectoryPlanner] Beyond the horizon \n";
            std::cout << "start:\t t:" << startPoint.time << " h: " << startPoint.height << "\n";
            std::cout << "end:\t t:" << std::max(finalTime, eventTimes_.back()) + settings_.touchdownAfterHorizon
                      << " h: " << terrainHeight + settings_.swingHeight << std::endl;
            footTrajectory.emplace_back(SplineCpg({settings_.liftOffVelocity, 0.0}));
            return SplineCpg::Point{std::max(finalTime, eventTimes_.back()) + settings_.touchdownAfterHorizon,
                                    terrainHeight + settings_.swingHeight};
          }
        }();

        // Construct the swing trajectory

        footTrajectory.back().set(startPoint, endPoint, terrainHeight + settings_.swingHeight);
      } else {  // stancePhase
        footTrajectory.emplace_back(SplineCpg({0.0, 0.0}));
        footTrajectory.back().set({0.0, terrainHeight}, {1.0, terrainHeight}, terrainHeight);
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
