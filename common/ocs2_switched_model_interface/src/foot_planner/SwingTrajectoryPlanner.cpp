//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const ComModelBase<scalar_t>& comModel,
                                               const KinematicsModelBase<scalar_t>& kinematicsModel, std::shared_ptr<const TerrainModel> terrainModelPtr)
    : settings_(std::move(settings)), comModel_(comModel.clone()), kinematicsModel_(kinematicsModel.clone()), terrainModelPtr_(terrainModelPtr) {}

void SwingTrajectoryPlanner::update(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                                    const ocs2::CostDesiredTrajectories costDesiredTrajectories,
                                    const ocs2::ModeSchedule& modeSchedule) {
  const auto basePose = comModel_->calculateBasePose(getComPose(currentState));
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  updateFeetTrajectories(initTime, finalTime, feetPositions, modeSchedule);
  updateErrorTrajectories(initTime, feetPositions, modeSchedule);
}



const TerrainPlane& SwingTrajectoryPlanner::getReferenceTerrainPlane(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return targetTerrains_[leg][index];
}

scalar_t SwingTrajectoryPlanner::getNormalDirectionVelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  const auto feedforwardVelocity = feetHeightTrajectories_[leg][index].velocity(time);
  // Feedback follows the planned trajectory s.t. de = - errorGain * e, with e = (z - z*)
  const auto feedbackVelocity = -settings_.errorGain * initialErrors_[leg] * std::exp(-settings_.errorGain * (time - initTime_));
  return feedforwardVelocity + feedbackVelocity;
}

scalar_t SwingTrajectoryPlanner::getNormalDirectionPositionConstraint(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].position(time);
}

void SwingTrajectoryPlanner::updateFeetTrajectories(scalar_t initTime, scalar_t finalTime,
                                                    const feet_array_t<vector3_t>& currentFeetPositions,
                                                    const ocs2::ModeSchedule& modeSchedule, const TerrainPlane& terrain) {
  //  std::cout << "ModeSchedule\n" << modeSchedule << std::endl;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg = extractContactFlags(modeSchedule.modeSequence);

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    //    std::cout << "Leg : " << leg << std::endl;

    const auto contactTimings = extractContactTimings(modeSchedule.eventTimes, contactSequencePerLeg[leg]);
    const auto selectTerrain

    auto& footTrajectory = feetNormalTrajectories_[leg];
    auto& footTrajectoryEvents = feetNormalTrajectoriesEvents_[leg];
    footTrajectory.clear();
    footTrajectoryEvents.clear();

    // If the first phase is a stance phase, register when it leaves contact.
    if (std::isnan(contactTimings.front().first)) {
      if (std::isnan(contactTimings.front().second)) {
        lastContacts_[leg] = {finalTime, currentFeetPositions[leg]};
      } else {
        lastContacts_[leg] = {contactTimings.front().second, currentFeetPositions[leg]};
      }
    }

    bool firstSwingPhase = true;
    for (const auto& footPhase : footPhases) {
      if (!std::isnan(footPhase.startTime) && footPhase.startTime >= finalTime) {
        // Phase starts after horizon, not need to plan a trajectory for it.
        break;
      }

      if (footPhase.type == FootPhaseType::Stance) {
        const auto startPoint = [&] {
          CubicSpline::Node point{};
          if (std::isnan(footPhase.startTime)) {
            point.time = initTime;
            point.position = terrainDistanceFromPositionInWorld(currentFeetPositions[leg], terrain);
          } else {
            point.time = footPhase.startTime;
            point.position = 0.0;
          }
          point.velocity = 0.0;
          return point;
        }();

        const auto endPoint = [&] {
          CubicSpline::Node point{};
          if (std::isnan(footPhase.endTime)) {
            point.time = finalTime;
          } else {
            point.time = footPhase.endTime;
          }
          point.position = startPoint.position;
          point.velocity = 0.0;
          return point;
        }();

        footTrajectory.emplace_back(startPoint, startPoint.position, endPoint);
        footTargetTerrains.push_back(terrain);
        footTrajectoryEvents.push_back(endPoint.time);

        //        std::cout << "\tstance\n";
        //        std::cout << "\tstart: \t t:" << startPoint.time << " h: " << startPoint.position << " v: " << startPoint.velocity <<
        //        "\n"; std::cout << "\tmid:   \t h: " << startPoint.position << "\n"; std::cout << "\tend:   \t t:" << endPoint.time << "
        //        h: " << endPoint.position << " v: " << endPoint.velocity << "\n\n";
      } else if (footPhase.type == FootPhaseType::Swing) {
        auto liftOff = [&] {
          CubicSpline::Node node{};
          if (std::isnan(footPhase.startTime)) {
            node.time = lastContacts_[leg].time;
          } else {
            node.time = footPhase.startTime;
          }
          if (firstSwingPhase) {
            node.position = terrainDistanceFromPositionInWorld(lastContacts_[leg].position, terrain);
            firstSwingPhase = false;
          } else {
            node.position = 0.0;
          }
          node.velocity = settings_.liftOffVelocity;
          return node;
        }();

        auto touchDown = [&] {
          CubicSpline::Node node{};
          if (std::isnan(footPhase.endTime)) {
            node.time = finalTime + settings_.touchdownAfterHorizon;
            node.position = settings_.swingHeight;
            node.velocity = 0.0;
          } else {
            node.time = footPhase.endTime;
            node.position = 0.0;
            node.velocity = settings_.touchDownVelocity;
          }
          return node;
        }();

        const scalar_t scaling = std::min(1.0, (touchDown.time - liftOff.time) / settings_.swingTimeScale);
        liftOff.velocity *= scaling;
        touchDown.velocity *= scaling;
        scalar_t midHeight = scaling * (0.5 * (liftOff.position + touchDown.position) + settings_.swingHeight);

        footTrajectory.emplace_back(liftOff, midHeight, touchDown);
        footTargetTerrains.push_back(terrain);
        footTrajectoryEvents.push_back(touchDown.time);

        //        std::cout << "\tswing\n";
        //        std::cout << "\tstart: \t t:" << liftOff.time << " h: " << liftOff.position << " v: " << liftOff.velocity << "\n";
        //        std::cout << "\tmid:   \t h: " << midHeight << " scaling: " << scaling << "\n";
        //        std::cout << "\tend:   \t t:" << touchDown.time << " h: " << touchDown.position << " v: " << touchDown.velocity << "\n\n";
      }
    }
  }
}


SwingTrajectoryPlannerSettings loadSwingTrajectorySettings(const std::string& filename, bool verbose) {
  SwingTrajectoryPlannerSettings settings{};

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const std::string prefix{"model_settings.swing_trajectory_settings."};

  if (verbose) {
    std::cerr << "\n #### Swing trajectory Settings:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  ocs2::loadData::loadPtreeValue(pt, settings.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingHeight, prefix + "swingHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.touchdownAfterHorizon, prefix + "touchdownAfterHorizon", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.errorGain, prefix + "errorGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingTimeScale, prefix + "swingTimeScale", verbose);

  if (verbose) {
    std::cerr << " #### ==================================================" << std::endl;
  }

  return settings;
}

std::vector<std::pair<scalar_t, scalar_t>> extractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<std::pair<scalar_t, scalar_t>> contactTimings;
  int currentPhase = 0;

  while (currentPhase < numPhases) {
    scalar_t startTime = (currentPhase == 0) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

    // Find when the contact phase ends
    while (currentPhase + 1 < numPhases && contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the phase
    scalar_t endTime = (currentPhase + 1 == numPhases) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

    // Add to phases
    contactTimings.emplace_back(startTime, endTime);
    ++currentPhase;
  }
  return contactTimings;
}

feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& modeSequence) {
  const size_t numPhases = modeSequence.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(modeSequence[i]);
    for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

}  // namespace switched_model
