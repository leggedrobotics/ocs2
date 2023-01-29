//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"
#include "ocs2_switched_model_interface/foot_planner/KinematicFootPlacementPenalty.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings,
                                               const KinematicsModelBase<scalar_t>& kinematicsModel,
                                               const InverseKinematicsModelBase* inverseKinematicsModelPtr)
    : settings_(std::move(settings)),
      kinematicsModel_(kinematicsModel.clone()),
      inverseKinematicsModelPtr_(nullptr),
      terrainModel_(nullptr) {
  if (inverseKinematicsModelPtr != nullptr) {
    inverseKinematicsModelPtr_.reset(inverseKinematicsModelPtr->clone());
  }
}

void SwingTrajectoryPlanner::updateTerrain(std::unique_ptr<TerrainModel> terrainModel) {
  terrainModel_ = std::move(terrainModel);
}

const SignedDistanceField* SwingTrajectoryPlanner::getSignedDistanceField() const {
  if (terrainModel_) {
    return terrainModel_->getSignedDistanceField();
  } else {
    return nullptr;
  }
}

void SwingTrajectoryPlanner::updateSwingMotions(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                                                const ocs2::TargetTrajectories& targetTrajectories,
                                                const ocs2::ModeSchedule& modeSchedule) {
  if (!terrainModel_) {
    throw std::runtime_error("[SwingTrajectoryPlanner] terrain cannot be null. Update the terrain before planning swing motions");
  }

  // Need a copy to
  // 1. possibly overwrite joint references later (adapted with inverse kinematics)
  // 2. ensure a maximum interval between references points.
  // 3. unsure we have samples at start and end of the MPC horizon.
  subsampleReferenceTrajectory(targetTrajectories, initTime, finalTime);

  const feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg = extractContactTimingsPerLeg(modeSchedule);

  const auto basePose = getBasePose(currentState);
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    const auto& contactTimings = contactTimingsPerLeg[leg];

    // Update last contacts
    if (!contactTimings.empty()) {
      if (startsWithStancePhase(contactTimings)) {
        // If currently in contact -> update expected liftoff.
        if (hasEndTime(contactTimings.front())) {
          updateLastContact(leg, contactTimings.front().end, feetPositions[leg], *terrainModel_);
        } else {  // Expected liftoff unknown, set to end of horizon
          updateLastContact(leg, finalTime, feetPositions[leg], *terrainModel_);
        }
      } else {
        // If currently in swing -> verify that liftoff was before the horizon. If not, assume liftoff happened exactly at initTime
        if (lastContacts_[leg].first > initTime) {
          updateLastContact(leg, initTime, feetPositions[leg], *terrainModel_);
        }
      }
    }

    // Select heuristic footholds.
    heuristicFootholdsPerLeg_[leg] = selectHeuristicFootholds(leg, contactTimings, targetTrajectories, initTime, currentState, finalTime);

    // Select terrain constraints based on the heuristic footholds.
    nominalFootholdsPerLeg_[leg] = selectNominalFootholdTerrain(leg, contactTimings, heuristicFootholdsPerLeg_[leg], targetTrajectories,
                                                                initTime, currentState, finalTime, *terrainModel_);

    // Create swing trajectories
    if (settings_.swingTrajectoryFromReference) {
      std::tie(feetNormalTrajectoriesEvents_[leg], feetNormalTrajectories_[leg]) =
          extractSwingTrajectoriesFromReference(leg, contactTimings, finalTime);
    } else {
      std::tie(feetNormalTrajectoriesEvents_[leg], feetNormalTrajectories_[leg]) =
          generateSwingTrajectories(leg, contactTimings, finalTime);
    }
  }

  if (inverseKinematicsModelPtr_ && !settings_.swingTrajectoryFromReference) {
    adaptJointReferencesWithInverseKinematics(finalTime);
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
        return SwingPhase::SwingEvent{contactTimings.front().start, settings_.touchDownVelocity,
                                      &nominalFootholdsPerLeg_[leg].front().plane};
      } else {
        return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon, 0.0, nullptr};
      }
    }();

    SwingPhase::SwingProfile swingProfile = getDefaultSwingProfile();
    applySwingMotionScaling(liftOff, touchDown, swingProfile);

    footPhases.emplace_back(new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_.get()));
  }

  // Loop through contact phases
  for (int i = 0; i < contactTimings.size(); ++i) {
    const auto& currentContactTiming = contactTimings[i];
    const ConvexTerrain& nominalFoothold = nominalFootholdsPerLeg_[leg][i];

    // If phase starts after the horizon, we don't need to plan for it
    if (currentContactTiming.start > finalTime) {
      break;
    }

    // generate contact phase
    if (hasStartTime(currentContactTiming)) {
      eventTimes.push_back(currentContactTiming.start);
    }
    footPhases.emplace_back(new StancePhase(nominalFoothold, settings_.terrainMargin));

    // If contact phase extends beyond the horizon, we can stop planning.
    if (!hasEndTime(currentContactTiming) || currentContactTiming.end > finalTime) {
      break;
    }

    // generate swing phase afterwards
    SwingPhase::SwingEvent liftOff{currentContactTiming.end, settings_.liftOffVelocity, &nominalFoothold.plane};
    SwingPhase::SwingEvent touchDown = [&] {
      const bool nextContactExists = (i + 1) < contactTimings.size();
      if (nextContactExists) {
        return SwingPhase::SwingEvent{contactTimings[i + 1].start, settings_.touchDownVelocity, &nominalFootholdsPerLeg_[leg][i + 1].plane};
      } else {
        return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon, 0.0, nullptr};
      }
    }();

    SwingPhase::SwingProfile swingProfile = getDefaultSwingProfile();
    applySwingMotionScaling(liftOff, touchDown, swingProfile);

    eventTimes.push_back(currentContactTiming.end);
    footPhases.emplace_back(new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_.get()));
  }

  return std::make_pair(eventTimes, std::move(footPhases));
}

std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> SwingTrajectoryPlanner::extractSwingTrajectoriesFromReference(
    int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const {
  std::vector<scalar_t> eventTimes;
  std::vector<std::unique_ptr<FootPhase>> footPhases;

  // First swing phase
  if (startsWithSwingPhase(contactTimings)) {
    scalar_t liftOffTime{lastContacts_[leg].first};
    scalar_t touchDownTime = [&] {
      if (touchesDownAtLeastOnce(contactTimings)) {
        return contactTimings.front().start;
      } else {
        return finalTime + settings_.referenceExtensionAfterHorizon;
      }
    }();

    footPhases.push_back(extractExternalSwingPhase(leg, liftOffTime, touchDownTime));
  }

  // Loop through contact phases
  for (int i = 0; i < contactTimings.size(); ++i) {
    const auto& currentContactTiming = contactTimings[i];
    const ConvexTerrain& nominalFoothold = nominalFootholdsPerLeg_[leg][i];

    // If phase starts after the horizon, we don't need to plan for it
    if (currentContactTiming.start > finalTime) {
      break;
    }

    // generate contact phase
    if (hasStartTime(currentContactTiming)) {
      eventTimes.push_back(currentContactTiming.start);
    }
    footPhases.emplace_back(new StancePhase(nominalFoothold, settings_.terrainMargin));

    // If contact phase extends beyond the horizon, we can stop planning.
    if (!hasEndTime(currentContactTiming) || currentContactTiming.end > finalTime) {
      break;
    }

    // generate swing phase afterwards
    scalar_t liftOffTime{currentContactTiming.end};
    scalar_t touchDownTime = [&] {
      const bool nextContactExists = (i + 1) < contactTimings.size();
      if (nextContactExists) {
        return contactTimings[i + 1].start;
      } else {
        return finalTime + settings_.referenceExtensionAfterHorizon;
      }
    }();

    eventTimes.push_back(currentContactTiming.end);
    footPhases.push_back(extractExternalSwingPhase(leg, liftOffTime, touchDownTime));
  }

  return std::make_pair(eventTimes, std::move(footPhases));
}

void SwingTrajectoryPlanner::applySwingMotionScaling(SwingPhase::SwingEvent& liftOff, SwingPhase::SwingEvent& touchDown,
                                                     SwingPhase::SwingProfile& swingProfile) const {
  const scalar_t scaling = [&]() {
    if (std::isnan(liftOff.time) || std::isnan(touchDown.time)) {
      return 1.0;
    } else {
      return std::min(1.0, (touchDown.time - liftOff.time) / settings_.swingTimeScale);
    }
  }();

  if (scaling < 1.0) {
    liftOff.velocity *= scaling;
    touchDown.velocity *= scaling;
    swingProfile.sdfMidswingMargin = scaling * settings_.sdfMidswingMargin;
    for (auto& node : swingProfile.nodes) {
      node.swingHeight *= scaling;
      node.normalVelocity *= scaling;
    }
  }
}

std::unique_ptr<ExternalSwingPhase> SwingTrajectoryPlanner::extractExternalSwingPhase(int leg, scalar_t liftOffTime,
                                                                                      scalar_t touchDownTime) const {
  std::vector<scalar_t> time;
  std::vector<vector3_t> positions;
  std::vector<vector3_t> velocities;

  const auto liftoffIndex = ocs2::LinearInterpolation::timeSegment(liftOffTime, targetTrajectories_.timeTrajectory);
  const auto touchdownIndex = ocs2::LinearInterpolation::timeSegment(touchDownTime, targetTrajectories_.timeTrajectory);

  // liftoff
  if (liftOffTime < targetTrajectories_.timeTrajectory[liftoffIndex.first + 1]) {
    const vector_t state = ocs2::LinearInterpolation::interpolate(liftoffIndex, targetTrajectories_.stateTrajectory);
    const vector_t input = ocs2::LinearInterpolation::interpolate(liftoffIndex, targetTrajectories_.inputTrajectory);
    time.push_back(liftOffTime);
    positions.push_back(kinematicsModel_->footPositionInOriginFrame(leg, getBasePose(state), getJointPositions(state)));
    velocities.push_back(kinematicsModel_->footVelocityInOriginFrame(leg, getBasePose(state), getBaseLocalVelocities(state),
                                                                     getJointPositions(state), getJointVelocities(input)));
  }

  // intermediate
  for (int k = liftoffIndex.first + 1; k < touchdownIndex.first; ++k) {
    const auto& state = targetTrajectories_.stateTrajectory[k];
    time.push_back(targetTrajectories_.timeTrajectory[k]);
    positions.push_back(kinematicsModel_->footPositionInOriginFrame(leg, getBasePose(state), getJointPositions(state)));
    velocities.push_back(kinematicsModel_->footVelocityInOriginFrame(leg, getBasePose(state), getBaseLocalVelocities(state),
                                                                     getJointPositions(state),
                                                                     getJointVelocities(targetTrajectories_.inputTrajectory[k])));
  }

  // touchdown
  if (time.back() < touchDownTime) {
    const vector_t state = ocs2::LinearInterpolation::interpolate(touchdownIndex, targetTrajectories_.stateTrajectory);
    const vector_t input = ocs2::LinearInterpolation::interpolate(touchdownIndex, targetTrajectories_.inputTrajectory);
    time.push_back(touchDownTime);
    positions.push_back(kinematicsModel_->footPositionInOriginFrame(leg, getBasePose(state), getJointPositions(state)));
    velocities.push_back(kinematicsModel_->footVelocityInOriginFrame(leg, getBasePose(state), getBaseLocalVelocities(state),
                                                                     getJointPositions(state), getJointVelocities(input)));
  }

  return std::unique_ptr<ExternalSwingPhase>(new ExternalSwingPhase(move(time), move(positions), move(velocities)));
}

std::vector<vector3_t> SwingTrajectoryPlanner::selectHeuristicFootholds(int leg, const std::vector<ContactTiming>& contactTimings,
                                                                        const ocs2::TargetTrajectories& targetTrajectories,
                                                                        scalar_t initTime, const comkino_state_t& currentState,
                                                                        scalar_t finalTime) const {
  // Zmp preparation : measured state
  const auto initBasePose = getBasePose(currentState);
  const auto initBaseOrientation = getOrientation(initBasePose);
  const auto initBaseTwistInBase = getBaseLocalVelocities(currentState);
  const auto initBaseLinearVelocityInWorld = rotateVectorBaseToOrigin(getLinearVelocity(initBaseTwistInBase), initBaseOrientation);

  // Zmp preparation : desired state
  const vector_t initDesiredState = targetTrajectories.getDesiredState(initTime);
  const base_coordinate_t initDesiredBasePose = initDesiredState.head<BASE_COORDINATE_SIZE>();
  const auto initDesiredOrientation = getOrientation(initDesiredBasePose);
  const base_coordinate_t initDesiredBaseTwistInBase = initDesiredState.segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
  const auto initDesiredBaseLinearVelocityInWorld =
      rotateVectorBaseToOrigin(getLinearVelocity(initDesiredBaseTwistInBase), initDesiredOrientation);

  // Compute zmp / inverted pendulum foot placement offset: delta p = sqrt(h / g) * (v - v_des)
  scalar_t pendulumFrequency = std::sqrt(settings_.invertedPendulumHeight / 9.81);
  scalar_t zmpX = pendulumFrequency * (initBaseLinearVelocityInWorld.x() - initDesiredBaseLinearVelocityInWorld.x());
  scalar_t zmpY = pendulumFrequency * (initBaseLinearVelocityInWorld.y() - initDesiredBaseLinearVelocityInWorld.y());
  const vector3_t zmpReactiveOffset = {zmpX, zmpY, 0.0};

  // Heuristic footholds to fill
  std::vector<vector3_t> heuristicFootholds;

  // Heuristic foothold is equal to current foothold for legs in contact
  if (startsWithStancePhase(contactTimings)) {
    heuristicFootholds.push_back(lastContacts_[leg].second.positionInWorld);
  }

  // For future contact phases, use TargetTrajectories at halve the contact phase
  int contactCount = 0;
  for (const auto& contactPhase : contactTimings) {
    if (hasStartTime(contactPhase)) {
      const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
      const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

      // Compute foot position from cost desired trajectory
      const vector_t state = targetTrajectories.getDesiredState(middleContactTime);
      const auto desiredBasePose = getBasePose(state);
      const auto desiredJointPositions = getJointPositions(state);
      vector3_t referenceFootholdPositionInWorld = kinematicsModel_->footPositionInOriginFrame(leg, desiredBasePose, desiredJointPositions);

      // Add ZMP offset to the first upcoming foothold.
      if (contactCount == 0) {
        referenceFootholdPositionInWorld += zmpReactiveOffset;
      }

      // One foothold added per contactPhase
      heuristicFootholds.push_back(referenceFootholdPositionInWorld);

      // Can stop for this leg if we have processed one contact phase after (or extending across) the horizon
      if (contactEndTime > finalTime) {
        break;
      }
    }
    ++contactCount;
  }

  return heuristicFootholds;
}

std::vector<ConvexTerrain> SwingTrajectoryPlanner::selectNominalFootholdTerrain(int leg, const std::vector<ContactTiming>& contactTimings,
                                                                                const std::vector<vector3_t>& heuristicFootholds,
                                                                                const ocs2::TargetTrajectories& targetTrajectories,
                                                                                scalar_t initTime, const comkino_state_t& currentState,
                                                                                scalar_t finalTime,
                                                                                const TerrainModel& terrainModel) const {
  // Will increment the heuristic each time after selecting a nominalFootholdTerrain
  auto heuristicFootholdIt = heuristicFootholds.cbegin();
  std::vector<ConvexTerrain> nominalFootholdTerrain;

  // Nominal foothold is equal to current foothold for legs in contact
  if (startsWithStancePhase(contactTimings)) {
    ConvexTerrain convexTerrain;
    convexTerrain.plane = lastContacts_[leg].second;
    nominalFootholdTerrain.push_back(convexTerrain);
    ++heuristicFootholdIt;  // Skip this heuristic. Use lastContact directly
  }

  // For future contact phases
  for (const auto& contactPhase : contactTimings) {
    if (hasStartTime(contactPhase)) {
      const scalar_t timeTillContact = contactPhase.start - initTime;
      const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
      const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

      // Get previous foothold if there was one at this time
      const FootPhase* previousIterationContact = getFootPhaseIfInContact(leg, middleContactTime);

      if (timeTillContact < settings_.previousFootholdTimeDeadzone && previousIterationContact != nullptr) {
        // Simply copy the information out of the previous iteration
        nominalFootholdTerrain.push_back(*previousIterationContact->nominalFootholdConstraint());
        ++heuristicFootholdIt;  // Skip this heuristic. Using the previous terrain instead
      } else {
        // Select the terrain base on the heuristic
        vector3_t referenceFootholdPositionInWorld = *heuristicFootholdIt;

        // Filter w.r.t. previous foothold
        if (previousIterationContact != nullptr) {
          referenceFootholdPositionInWorld =
              filterFoothold(referenceFootholdPositionInWorld, previousIterationContact->nominalFootholdLocation());
        }

        // Kinematic penalty
        const base_coordinate_t basePoseAtTouchdown = getBasePose(targetTrajectories.getDesiredState(contactPhase.start));
        const auto hipPositionInWorldTouchdown = kinematicsModel_->legRootInOriginFrame(leg, basePoseAtTouchdown);
        const auto hipOrientationInWorldTouchdown = kinematicsModel_->orientationLegRootToOriginFrame(leg, basePoseAtTouchdown);
        const base_coordinate_t basePoseAtLiftoff = getBasePose(targetTrajectories.getDesiredState(contactEndTime));
        const auto hipPositionInWorldLiftoff = kinematicsModel_->legRootInOriginFrame(leg, basePoseAtLiftoff);
        const auto hipOrientationInWorldLiftoff = kinematicsModel_->orientationLegRootToOriginFrame(leg, basePoseAtLiftoff);
        ApproximateKinematicsConfig config;
        config.kinematicPenaltyWeight = settings_.legOverExtensionPenalty;
        config.maxLegExtension = settings_.nominalLegExtension;
        auto scoringFunction = [&](const vector3_t& footPositionInWorld) {
          return computeKinematicPenalty(footPositionInWorld, hipPositionInWorldTouchdown, hipOrientationInWorldTouchdown, config) +
                 computeKinematicPenalty(footPositionInWorld, hipPositionInWorldLiftoff, hipOrientationInWorldLiftoff, config);
        };

        if (contactPhase.start < finalTime) {
          ConvexTerrain convexTerrain = terrainModel.getConvexTerrainAtPositionInWorld(referenceFootholdPositionInWorld, scoringFunction);
          nominalFootholdTerrain.push_back(convexTerrain);
          ++heuristicFootholdIt;
        } else {  // After the horizon -> we are only interested in the position and orientation
          ConvexTerrain convexTerrain;
          convexTerrain.plane =
              terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(referenceFootholdPositionInWorld, scoringFunction);
          nominalFootholdTerrain.push_back(convexTerrain);
          ++heuristicFootholdIt;
        }
      }

      // Can stop for this leg if we have processed one contact phase after (or extending across) the horizon
      if (contactEndTime > finalTime) {
        break;
      }
    }
  }

  return nominalFootholdTerrain;
}

void SwingTrajectoryPlanner::subsampleReferenceTrajectory(const ocs2::TargetTrajectories& targetTrajectories, scalar_t initTime,
                                                          scalar_t finalTime) {
  if (targetTrajectories.empty()) {
    throw std::runtime_error("[SwingTrajectoryPlanner] provided target trajectory cannot be empty.");
  }

  targetTrajectories_.clear();

  // Add first reference
  {
    const auto initInterpIndex = ocs2::LinearInterpolation::timeSegment(initTime, targetTrajectories.timeTrajectory);
    targetTrajectories_.timeTrajectory.push_back(initTime);
    targetTrajectories_.stateTrajectory.push_back(
        ocs2::LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.stateTrajectory));
    targetTrajectories_.inputTrajectory.push_back(
        ocs2::LinearInterpolation::interpolate(initInterpIndex, targetTrajectories.inputTrajectory));
  }

  for (int k = 0; k < targetTrajectories.timeTrajectory.size(); ++k) {
    if (targetTrajectories.timeTrajectory[k] < initTime) {
      continue;  // Drop all samples before init time
    } else if (targetTrajectories.timeTrajectory[k] > finalTime) {
      // Add final time sample. Samples after final time are also kept for touchdowns after the horizon.
      const auto finalInterpIndex = ocs2::LinearInterpolation::timeSegment(finalTime, targetTrajectories.timeTrajectory);
      targetTrajectories_.timeTrajectory.push_back(finalTime);
      targetTrajectories_.stateTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.stateTrajectory));
      targetTrajectories_.inputTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(finalInterpIndex, targetTrajectories.inputTrajectory));
    }

    // Check if we need to add extra intermediate samples
    while (targetTrajectories_.timeTrajectory.back() + settings_.maximumReferenceSampleTime < targetTrajectories.timeTrajectory[k]) {
      const scalar_t t = targetTrajectories_.timeTrajectory.back() + settings_.maximumReferenceSampleTime;
      const auto interpIndex = ocs2::LinearInterpolation::timeSegment(t, targetTrajectories.timeTrajectory);

      targetTrajectories_.timeTrajectory.push_back(t);
      targetTrajectories_.stateTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(interpIndex, targetTrajectories.stateTrajectory));
      targetTrajectories_.inputTrajectory.push_back(
          ocs2::LinearInterpolation::interpolate(interpIndex, targetTrajectories.inputTrajectory));
    }

    // Add the original reference sample
    targetTrajectories_.timeTrajectory.push_back(targetTrajectories.timeTrajectory[k]);
    targetTrajectories_.stateTrajectory.push_back(targetTrajectories.stateTrajectory[k]);
    targetTrajectories_.inputTrajectory.push_back(targetTrajectories.inputTrajectory[k]);
  }
}

void SwingTrajectoryPlanner::adaptJointReferencesWithInverseKinematics(scalar_t finalTime) {
  const scalar_t damping = 0.01;  // Quite some damping on the IK to get well conditions references.

  for (int k = 0; k < targetTrajectories_.timeTrajectory.size(); ++k) {
    const scalar_t t = targetTrajectories_.timeTrajectory[k];

    const base_coordinate_t basePose = getBasePose(comkino_state_t(targetTrajectories_.stateTrajectory[k]));
    const vector3_t basePositionInWorld = getPositionInOrigin(basePose);
    const vector3_t eulerXYZ = getOrientation(basePose);

    for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      const auto& footPhase = this->getFootPhase(leg, t);

      // Joint positions
      const vector3_t positionBaseToFootInWorldFrame = footPhase.getPositionInWorld(t) - basePositionInWorld;
      const vector3_t positionBaseToFootInBaseFrame = rotateVectorOriginToBase(positionBaseToFootInWorldFrame, eulerXYZ);

      const size_t stateOffset = 2 * BASE_COORDINATE_SIZE + 3 * leg;
      targetTrajectories_.stateTrajectory[k].segment(stateOffset, 3) =
          inverseKinematicsModelPtr_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(leg, positionBaseToFootInBaseFrame);

      // Joint velocities
      auto jointPositions = getJointPositions(targetTrajectories_.stateTrajectory[k]);
      auto baseTwistInBaseFrame = getBaseLocalVelocities(targetTrajectories_.stateTrajectory[k]);

      const vector3_t b_baseToFoot = kinematicsModel_->positionBaseToFootInBaseFrame(leg, jointPositions);
      const vector3_t footVelocityInBaseFrame = rotateVectorOriginToBase(footPhase.getVelocityInWorld(t), eulerXYZ);
      const vector3_t footRelativeVelocityInBaseFrame =
          footVelocityInBaseFrame - getLinearVelocity(baseTwistInBaseFrame) - getAngularVelocity(baseTwistInBaseFrame).cross(b_baseToFoot);

      const size_t inputOffset = 3 * NUM_CONTACT_POINTS + 3 * leg;
      targetTrajectories_.inputTrajectory[k].segment(inputOffset, 3) =
          inverseKinematicsModelPtr_->getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
              leg, footRelativeVelocityInBaseFrame, kinematicsModel_->baseToFootJacobianBlockInBaseFrame(leg, jointPositions), damping);
    }

    // Can stop adaptation as soon as we have processed a point beyond the horizon.
    if (t > finalTime) {
      break;
    }
  }
}

void SwingTrajectoryPlanner::updateLastContact(int leg, scalar_t expectedLiftOff, const vector3_t& currentFootPosition,
                                               const TerrainModel& terrainModel) {
  // Get orientation from terrain model, position from the kinematics
  auto lastContactTerrain = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(currentFootPosition);
  lastContactTerrain.positionInWorld = currentFootPosition;
  lastContacts_[leg] = {expectedLiftOff, lastContactTerrain};
}

SwingPhase::SwingProfile SwingTrajectoryPlanner::getDefaultSwingProfile() const {
  SwingPhase::SwingProfile defaultSwingProfile;
  defaultSwingProfile.sdfMidswingMargin = settings_.sdfMidswingMargin;
  defaultSwingProfile.maxSwingHeightAdaptation = 2.0 * settings_.swingHeight;

  SwingPhase::SwingProfile::Node midPoint;
  midPoint.phase = 0.5;
  midPoint.swingHeight = settings_.swingHeight;
  midPoint.normalVelocity = 0.0;
  midPoint.tangentialProgress = 0.6;
  midPoint.tangentialVelocityFactor = 2.0;
  defaultSwingProfile.nodes.push_back(midPoint);
  return defaultSwingProfile;
}

scalar_t SwingTrajectoryPlanner::getContactEndTime(const ContactTiming& contactPhase, scalar_t finalTime) const {
  return hasEndTime(contactPhase) ? contactPhase.end : std::max(finalTime + settings_.referenceExtensionAfterHorizon, contactPhase.start);
}

const FootPhase* SwingTrajectoryPlanner::getFootPhaseIfInContact(size_t leg, scalar_t time) const {
  const FootPhase* previousIterationContact = nullptr;
  if (!feetNormalTrajectories_[leg].empty()) {
    const auto& footPhase = getFootPhase(leg, time);
    if (footPhase.contactFlag()) {
      previousIterationContact = &footPhase;
    }
  }
  return previousIterationContact;
}

vector3_t SwingTrajectoryPlanner::filterFoothold(const vector3_t& newFoothold, const vector3_t& previousFoothold) const {
  // Apply Position deadzone and low pass filter
  if ((newFoothold - previousFoothold).norm() < settings_.previousFootholdDeadzone) {
    return previousFoothold;
  } else {
    // low pass filter
    const scalar_t lambda = settings_.previousFootholdFactor;
    return lambda * previousFoothold + (1.0 - lambda) * newFoothold;
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
  ocs2::loadData::loadPtreeValue(pt, settings.errorGain, prefix + "errorGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingTimeScale, prefix + "swingTimeScale", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.sdfMidswingMargin, prefix + "sdfMidswingMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.terrainMargin, prefix + "terrainMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdFactor, prefix + "previousFootholdFactor", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdDeadzone, prefix + "previousFootholdDeadzone", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdTimeDeadzone, prefix + "previousFootholdTimeDeadzone", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.invertedPendulumHeight, prefix + "invertedPendulumHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.nominalLegExtension, prefix + "nominalLegExtension", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.legOverExtensionPenalty, prefix + "legOverExtensionPenalty", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.referenceExtensionAfterHorizon, prefix + "referenceExtensionAfterHorizon", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.maximumReferenceSampleTime, prefix + "maximumReferenceSampleTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingTrajectoryFromReference, prefix + "swingTrajectoryFromReference", verbose);

  if (verbose) {
    std::cerr << " #### ==================================================" << std::endl;
  }

  return settings;
}

}  // namespace switched_model
