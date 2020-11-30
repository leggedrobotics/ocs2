/*
 * GaitAdaptation.cpp
 *
 *  Created on: Jun 4, 2020
 *      Author: Marko Bjelonic
 */

// anymal ctrl ocs2
#include "ocs2_switched_model_interface/logic/GaitAdaptation.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/logic/SingleLegLogic.h"

namespace switched_model {

GaitAdaptation::GaitAdaptation(const GaitAdaptationSettings& settings, const contact_flag_t& measuredContactFlags)
    : timeUntilNextTouchDownPerLeg_(constantFeetArray<scalar_t>(0.0)),
      timeUntilNextLiftOffPerLeg_(constantFeetArray<scalar_t>(0.0)),
      settings_(settings) {}

void GaitAdaptation::advance(GaitSchedule& gaitSchedule, const contact_flag_t& measuredContactFlags, scalar_t dt) {
  // Update internal knowledge of next touchdown / liftoff
  advanceSwingEvents(gaitSchedule);

  // Decide on a strategy per leg and schedule adaptation
  const auto& scheduledAdaptation = advanceLegStrategies(modeNumber2StanceLeg(gaitSchedule.getCurrentMode()), measuredContactFlags);

  // Apply all scheduled
  applyAdaptation(gaitSchedule, scheduledAdaptation);
}

void GaitAdaptation::advanceSwingEvents(const GaitSchedule& gaitSchedule) {
  // Only interested in events within the specified windows
  const auto checkHorizon = settings_.earlyTouchDownTimeWindow;

  // Extract contact timings from the gait schedule
  const auto modeSchedule = gaitSchedule.getModeSchedule(checkHorizon);
  const auto contactTimingsPerLeg = switched_model::extractContactTimingsPerLeg(modeSchedule);
  const auto currentTime = gaitSchedule.getCurrentTime();

  for (size_t leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    timeUntilNextTouchDownPerLeg_[leg] = getTimeOfNextTouchDown(currentTime, contactTimingsPerLeg[leg]) - currentTime;
    timeUntilNextLiftOffPerLeg_[leg] = getTimeOfNextLiftOff(currentTime, contactTimingsPerLeg[leg]) - currentTime;
  }
}

auto GaitAdaptation::advanceLegStrategies(const contact_flag_t& desiredContactFlags, const contact_flag_t& measuredContactFlags)
    -> feet_array_t<ScheduledAdaptation> {
  auto adaptations = constantFeetArray<ScheduledAdaptation>(ScheduledAdaptation::None);

  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    bool desiredContact = desiredContactFlags[leg];
    bool measuredContact = measuredContactFlags[leg];

    if (desiredContact) {
      if (measuredContact) {
        adaptations[leg] = desiredContactMeasuredContact(leg);
      } else {
        adaptations[leg] = desiredContactMeasuredMotion(leg);
      }
    } else {  // motion is desired
      if (measuredContact) {
        adaptations[leg] = desiredMotionMeasuredContact(leg);
      } else {
        adaptations[leg] = desiredMotionMeasuredMotion(leg);
      }
    }
  }

  return adaptations;
}

auto GaitAdaptation::desiredContactMeasuredContact(size_t leg) -> ScheduledAdaptation {
  return ScheduledAdaptation::None;
}

auto GaitAdaptation::desiredContactMeasuredMotion(size_t leg) -> ScheduledAdaptation {
  return ScheduledAdaptation::None;
}

auto GaitAdaptation::desiredMotionMeasuredContact(size_t leg) -> ScheduledAdaptation {
  if (!std::isnan(timeUntilNextTouchDownPerLeg_[leg]) && timeUntilNextTouchDownPerLeg_[leg] < settings_.earlyTouchDownTimeWindow) {
    // Touchdown was planned to be soon -> Take the contact early
    return ScheduledAdaptation::EarlyContact;
  } else {
    return ScheduledAdaptation::None;
  }
}

auto GaitAdaptation::desiredMotionMeasuredMotion(size_t leg) -> ScheduledAdaptation {
  return ScheduledAdaptation::None;
}

void GaitAdaptation::applyAdaptation(GaitSchedule& gaitSchedule, const feet_array_t<ScheduledAdaptation>& scheduledAdaptations) {
  // Early contact
  if (std::any_of(scheduledAdaptations.begin(), scheduledAdaptations.end(), isEarlyContact)) {
    const auto earlyTouchDownPerLeg = applyPerLeg(isEarlyContact, scheduledAdaptations);
    gaitSchedule.adaptCurrentGait(earlyTouchDownAdaptation(earlyTouchDownPerLeg));
  }
}

std::function<void(scalar_t& currentPhase, switched_model::Gait& currentGait, scalar_t currTime, switched_model::Gait& nextGait)>
earlyTouchDownAdaptation(const switched_model::feet_array_t<bool>& earlyTouchDownPerLeg) {
  return [=](scalar_t& currentPhase, Gait& currentGait, scalar_t currTime, Gait& nextGait) {
    const int currentModeIndex = getModeIndexFromPhase(currentPhase, currentGait);
    for (unsigned int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      if (earlyTouchDownPerLeg[leg]) {
        const int modeIndexOfNextContact = getModeIndexOfNextContactStateOfLeg(true, currentModeIndex, leg, currentGait);

        if (modeIndexOfNextContact < currentGait.modeSequence.size()) {  // update current gait
          setContactStateOfLegBetweenModes(true, currentModeIndex, modeIndexOfNextContact, leg, currentGait);
        } else {  // update current gait till end, and next gait
          setContactStateOfLegBetweenModes(true, currentModeIndex, modeIndexOfNextContact - 1, leg, currentGait);

          int modeIndexOfNextContactOfNextGait = getModeIndexOfNextContactStateOfLeg(true, 0, leg, nextGait);
          if (modeIndexOfNextContactOfNextGait == nextGait.modeSequence.size()) {
            --modeIndexOfNextContactOfNextGait;
          }
          setContactStateOfLegBetweenModes(true, 0, modeIndexOfNextContactOfNextGait, leg, nextGait);
        }
      }
    }
  };
}

int getModeIndexOfNextContactStateOfLeg(bool contact, int startModeIdx, size_t leg, const Gait& gait) {
  int modeIndex = startModeIdx;
  while (modeIndex < gait.modeSequence.size()) {
    size_t currentMode = gait.modeSequence[modeIndex];
    if (modeNumber2StanceLeg(currentMode)[leg] == contact) {
      break;
    } else {
      ++modeIndex;
    }
  }
  return modeIndex;
}

void setContactStateOfLegBetweenModes(bool contact, int startModeIdx, int lastModeIdx, size_t leg, Gait& gait) {
  for (int modeIndex = startModeIdx; modeIndex <= lastModeIdx; ++modeIndex) {
    auto stanceLegs = switched_model::modeNumber2StanceLeg(gait.modeSequence[modeIndex]);
    stanceLegs[leg] = contact;
    gait.modeSequence[modeIndex] = switched_model::stanceLeg2ModeNumber(stanceLegs);
  }
}

}  // namespace switched_model
