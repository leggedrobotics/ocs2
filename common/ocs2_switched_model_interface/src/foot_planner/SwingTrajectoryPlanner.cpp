//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const ComModelBase<scalar_t>& comModel,
                                               const KinematicsModelBase<scalar_t>& kinematicsModel)
    : settings_(std::move(settings)), comModel_(comModel.clone()), kinematicsModel_(kinematicsModel.clone()) {}

void SwingTrajectoryPlanner::update(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                                    const ocs2::TargetTrajectories& targetTrajectories,
                                    const feet_array_t<std::vector<ContactTiming>>& contactTimingsPerLeg,
                                    const TerrainModel& terrainModel) {
  const auto basePose = comModel_->calculateBasePose(getComPose(currentState));
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    const auto& contactTimings = contactTimingsPerLeg[leg];

    // Update last contacts if this leg is ever in contact
    if (!contactTimings.empty() && startsWithStancePhase(contactTimings)) {
      updateLastContact(leg, contactTimings.front().end, feetPositions[leg], terrainModel);
    }

    // Nominal footholds / terrain planes
    nominalFootholdsPerLeg_[leg] = selectNominalFootholdTerrain(leg, contactTimings, targetTrajectories, finalTime, terrainModel);

    // Create swing trajectories
    std::tie(feetNormalTrajectoriesEvents_[leg], feetNormalTrajectories_[leg]) = generateSwingTrajectories(leg, contactTimings, finalTime);
  }
}

const FootPhase& SwingTrajectoryPlanner::getFootPhase(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetNormalTrajectoriesEvents_[leg], time);
  return *feetNormalTrajectories_[leg][index];
}

auto SwingTrajectoryPlanner::generateSwingTrajectories(int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const
    -> std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> {
  std::vector<scalar_t> eventTimes;
  std::vector<std::unique_ptr<FootPhase>> footPhases;

  // First swing phase
  if (startsWithSwingPhase(contactTimings)) {
    SwingPhase::SwingEvent liftOff{lastContacts_[leg].first, settings_.liftOffVelocity, &lastContacts_[leg].second};
    SwingPhase::SwingEvent touchDown = [&] {
      if (touchesDownAtLeastOnce(contactTimings)) {
        return SwingPhase::SwingEvent{contactTimings.front().start, settings_.touchDownVelocity, &nominalFootholdsPerLeg_[leg].front()};
      } else {
        return SwingPhase::SwingEvent{finalTime + settings_.touchdownAfterHorizon, 0.0, nullptr};
      }
    }();
    const scalar_t scaling = getSwingMotionScaling(liftOff.time, touchDown.time);
    liftOff.velocity *= scaling;
    touchDown.velocity *= scaling;
    footPhases.emplace_back(new SwingPhase(liftOff, scaling * settings_.swingHeight, touchDown, settings_.errorGain));
  }

  // Loop through contact phases
  for (int i = 0; i < contactTimings.size(); ++i) {
    const auto& currentContactTiming = contactTimings[i];
    const TerrainPlane& nominalFoothold = nominalFootholdsPerLeg_[leg][i];

    // If phase starts after the horizon, we don't need to plan for it
    if (currentContactTiming.start > finalTime) {
      break;
    }

    // generate contact phase
    if (hasStartTime(currentContactTiming)) {
      eventTimes.push_back(currentContactTiming.start);
    }
    footPhases.emplace_back(new StancePhase(nominalFoothold, settings_.errorGain));

    // generate swing phase afterwards if the current contact is finite and ends before the horizon
    if (hasEndTime(currentContactTiming) && currentContactTiming.end < finalTime) {
      SwingPhase::SwingEvent liftOff{currentContactTiming.end, settings_.liftOffVelocity, &nominalFoothold};
      SwingPhase::SwingEvent touchDown = [&] {
        const bool nextContactExists = (i + 1) < contactTimings.size();
        if (nextContactExists) {
          return SwingPhase::SwingEvent{contactTimings[i + 1].start, settings_.touchDownVelocity, &nominalFootholdsPerLeg_[leg][i + 1]};
        } else {
          return SwingPhase::SwingEvent{finalTime + settings_.touchdownAfterHorizon, 0.0, nullptr};
        }
      }();
      const scalar_t scaling = getSwingMotionScaling(liftOff.time, touchDown.time);
      liftOff.velocity *= scaling;
      touchDown.velocity *= scaling;
      eventTimes.push_back(currentContactTiming.end);
      footPhases.emplace_back(new SwingPhase(liftOff, scaling * settings_.swingHeight, touchDown, settings_.errorGain));
    }
  }

  return std::make_pair(eventTimes, std::move(footPhases));
}

scalar_t SwingTrajectoryPlanner::getSwingMotionScaling(scalar_t liftoffTime, scalar_t touchDownTime) const {
  if (std::isnan(liftoffTime) || std::isnan(touchDownTime)) {
    return 1.0;
  } else {
    return std::min(1.0, (touchDownTime - liftoffTime) / settings_.swingTimeScale);
  }
}

std::vector<TerrainPlane> SwingTrajectoryPlanner::selectNominalFootholdTerrain(int leg, const std::vector<ContactTiming>& contactTimings,
                                                                               const ocs2::TargetTrajectories& targetTrajectories,
                                                                               scalar_t finalTime, const TerrainModel& terrainModel) const {
  std::vector<TerrainPlane> nominalFootholdTerrain;

  // Nominal foothold is equal to current foothold for legs in contact
  if (startsWithStancePhase(contactTimings)) {
    nominalFootholdTerrain.push_back(lastContacts_[leg].second);
  }

  // For future contact phases, use TargetTrajectories at halve the contact phase
  for (const auto& contactPhase : contactTimings) {
    if (hasStartTime(contactPhase)) {
      const auto middleContactTime = [&] {
        if (hasEndTime(contactPhase)) {
          return 0.5 * (contactPhase.start + contactPhase.end);
        } else {
          return 0.5 * (contactPhase.start + std::max(finalTime, contactPhase.start));
        }
      }();

      // Compute foot position from cost desired trajectory
      vector_t state = targetTrajectories.getDesiredState(middleContactTime);
      const base_coordinate_t middleContactDesiredComPose = state.head<BASE_COORDINATE_SIZE>();
      const joint_coordinate_t desiredJointPositions = state.segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE);
      const auto desiredBasePose = comModel_->calculateBasePose(middleContactDesiredComPose);
      const auto nominalFootholdPositionInWorld = kinematicsModel_->footPositionInOriginFrame(leg, desiredBasePose, desiredJointPositions);

      nominalFootholdTerrain.push_back(terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(nominalFootholdPositionInWorld));
    }
  }

  return nominalFootholdTerrain;
}

void SwingTrajectoryPlanner::updateLastContact(int leg, scalar_t expectedLiftOff, const vector3_t& currentFootPosition,
                                               const TerrainModel& terrainModel) {
  // Get orientation from terrain model, position from the kinematics
  auto lastContactTerrain = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(currentFootPosition);
  lastContactTerrain.positionInWorld = currentFootPosition;
  lastContacts_[leg] = {expectedLiftOff, lastContactTerrain};
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
