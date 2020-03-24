//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const ComModelBase<double>& comModel,
                                               const KinematicsModelBase<double>& kinematicsModel)
    : settings_(std::move(settings)), comModel_(comModel.clone()), kinematicsModel_(kinematicsModel.clone()) {}

void SwingTrajectoryPlanner::update(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                                    const ocs2::ModeSchedule& modeSchedule, scalar_t terrainHeight) {
  const auto basePose = comModel_->calculateBasePose(getComPose(currentState));
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  updateFeetTrajectories(initTime, finalTime, feetPositions, modeSchedule, terrainHeight);
  updateErrorTrajectories(initTime, feetPositions, modeSchedule);

  std::cout << "[SwingTrajectoryPlanner]\n";
  std::cout << "Last contact:\n";
  std::cout << "0:\t t:" << lastContacts_[0].time << " h: " << lastContacts_[0].height << "\n";
  std::cout << "1:\t t:" << lastContacts_[1].time << " h: " << lastContacts_[1].height << "\n";
  std::cout << "2:\t t:" << lastContacts_[2].time << " h: " << lastContacts_[2].height << "\n";
  std::cout << "3:\t t:" << lastContacts_[3].time << " h: " << lastContacts_[3].height << "\n";
  std::cout << std::endl;
}

void SwingTrajectoryPlanner::update(const ocs2::ModeSchedule& modeSchedule,
                                    const std::array<scalar_array_t, NUM_CONTACT_POINTS>& liftOffHeightSequence,
                                    const std::array<scalar_array_t, NUM_CONTACT_POINTS>& touchDownHeightSequence) {}

auto SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const -> scalar_t {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  const auto feedforwardVelocity = feetHeightTrajectories_[leg][index].velocity(time);
  // Feedback follows the planned trajectory s.t. de = - errorGain * e, with e = (z - z*)
  const auto feedbackVelocity = -settings_.errorGain * initialErrors_[leg] * std::exp(-settings_.errorGain * (time - initTime_));
  return feedforwardVelocity + feedbackVelocity;
}

auto SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const -> scalar_t {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].position(time);
}

void SwingTrajectoryPlanner::updateFeetTrajectories(scalar_t initTime, scalar_t finalTime,
                                                    const std::array<vector3_t, NUM_CONTACT_POINTS>& currentFeetPositions,
                                                    const ocs2::ModeSchedule& modeSchedule, scalar_t terrainHeight) {
  std::cout << "ModeSchedule\n" << modeSchedule << std::endl;

  // Convert mode sequence to a contact flag vector per leg
  const std::array<std::vector<bool>, NUM_CONTACT_POINTS> contactSequencePerLeg = extractContactFlags(modeSchedule.modeSequence);

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    std::cout << "Leg : " << leg << std::endl;

    const auto footPhases = extractFootPhases(modeSchedule.eventTimes, contactSequencePerLeg[leg]);
    auto& footTrajectory = feetHeightTrajectories_[leg];
    auto& footTrajectoryEvents = feetHeightTrajectoriesEvents_[leg];
    footTrajectory.clear();
    footTrajectoryEvents.clear();

    // If the first phase is a stance phase, register when it leaves contact.
    if (footPhases.front().type == FootPhaseType::Stance) {
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

      if (footPhase.type == FootPhaseType::Stance) {
        const auto startPoint = [&] {
          CubicSpline::Node point{};
          if (std::isnan(footPhase.startTime)) {
            point.time = initTime;
            point.position = currentFeetPositions[leg].z();
          } else {
            point.time = footPhase.startTime;
            point.position = terrainHeight;
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
        footTrajectoryEvents.push_back(endPoint.time);

        std::cout << "\tstance\n";
        std::cout << "\tstart: \t t:" << startPoint.time << " h: " << startPoint.position << " v: " << startPoint.velocity << "\n";
        std::cout << "\tmid:   \t h: " << startPoint.position << "\n";
        std::cout << "\tend:   \t t:" << endPoint.time << " h: " << endPoint.position << " v: " << endPoint.velocity << "\n\n";
      } else if (footPhase.type == FootPhaseType::Swing) {
        auto liftOff = [&] {
          CubicSpline::Node node{};
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

        auto touchDown = [&] {
          CubicSpline::Node node{};
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
        liftOff.velocity *= scaling;
        touchDown.velocity *= scaling;
        double midHeight = scaling * (terrainHeight + settings_.swingHeight);

        footTrajectory.emplace_back(liftOff, midHeight, touchDown);
        footTrajectoryEvents.push_back(touchDown.time);

        std::cout << "\tswing\n";
        std::cout << "\tstart: \t t:" << liftOff.time << " h: " << liftOff.position << " v: " << liftOff.velocity << "\n";
        std::cout << "\tmid:   \t h: " << midHeight << " scaling: " << scaling << "\n";
        std::cout << "\tend:   \t t:" << touchDown.time << " h: " << touchDown.position << " v: " << touchDown.velocity << "\n\n";
      }
    }
  }
}

void SwingTrajectoryPlanner::updateErrorTrajectories(scalar_t initTime,
                                                     const std::array<vector3_t, NUM_CONTACT_POINTS>& currentFeetPositions,
                                                     const ocs2::ModeSchedule& modeSchedule) {
  const auto index = ocs2::lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);
  initTime_ = initTime;
  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    initialErrors_[leg] = currentFeetPositions[leg].z() - feetHeightTrajectories_[leg][index].position(initTime);
  }
}

std::vector<SwingTrajectoryPlanner::FootPhase> SwingTrajectoryPlanner::extractFootPhases(const std::vector<scalar_t>& eventTimes,
                                                                                         const std::vector<bool>& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<SwingTrajectoryPlanner::FootPhase> footPhases;
  int currentPhase = 0;
  while (currentPhase < numPhases) {
    // Register start of the phase
    FootPhase currentFootPhase{};
    currentFootPhase.type = (contactFlags[currentPhase]) ? FootPhaseType::Stance : FootPhaseType::Swing;
    currentFootPhase.startTime = (currentPhase == 0) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

    // Find when the phase ends
    while (currentPhase + 1 < numPhases && contactFlags[currentPhase] == contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the phase
    currentFootPhase.endTime = (currentPhase + 1 == numPhases) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

    // Add to phases
    footPhases.push_back(std::move(currentFootPhase));
    ++currentPhase;
  }
  return footPhases;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::array<std::vector<bool>, NUM_CONTACT_POINTS> SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) {
  const size_t numPhases = phaseIDsStock.size();

  std::array<std::vector<bool>, NUM_CONTACT_POINTS> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    std::array<bool, NUM_CONTACT_POINTS> contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
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

}  // namespace switched_model
