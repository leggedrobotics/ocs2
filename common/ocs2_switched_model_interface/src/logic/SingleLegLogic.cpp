//
// Created by rgrandia on 27.04.20.
//

#include "ocs2_switched_model_interface/logic/SingleLegLogic.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

feet_array_t<std::vector<ContactTiming>> extractContactTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg = extractContactFlags(modeSchedule.modeSequence);

  // Extract timings per leg
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    contactTimingsPerLeg[leg] = extractContactTimings(modeSchedule.eventTimes, contactSequencePerLeg[leg]);
  }

  return contactTimingsPerLeg;
}

feet_array_t<scalar_t> getTimeUntilNextLiftOffPerLeg(scalar_t currentTime, const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<scalar_t> timeUntilNextLiftOffPerLeg;

  // Convert mode sequence to a contact timing vector per leg.
  const auto& contactTimingsPerLeg = extractContactTimingsPerLeg(modeSchedule);

  // Extract timings per leg
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    if (contactTimingsPerLeg[leg].empty()) {
      timeUntilNextLiftOffPerLeg[leg] = -1.0;
    } else if (std::isnan(contactTimingsPerLeg[leg].front().end)) {
      timeUntilNextLiftOffPerLeg[leg] = -1.0;
    } else {
      timeUntilNextLiftOffPerLeg[leg] = contactTimingsPerLeg[leg].front().end - currentTime;
    }
  }

  return timeUntilNextLiftOffPerLeg;
}

feet_array_t<scalar_t> getTimeUntilNextTouchDownPerLeg(scalar_t currentTime, const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<scalar_t> timeUntilNextTouchDownPerLeg;

  // Convert mode sequence to a contact timing vector per leg.
  auto contactTimingsPerLeg = extractContactTimingsPerLeg(modeSchedule);

  // Convert mode sequence to a contact flag vector per leg
  const auto& contactSequencePerLeg = extractContactFlags(modeSchedule.modeSequence);

  // Extract timings per leg
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    if (contactSequencePerLeg[leg].empty()) {
      timeUntilNextTouchDownPerLeg[leg] = -1.0;
      break;
    }
    if (contactSequencePerLeg[leg].front()) {
      contactTimingsPerLeg[leg].erase(contactTimingsPerLeg[leg].begin());
    }

    if (contactTimingsPerLeg[leg].empty()) {
      timeUntilNextTouchDownPerLeg[leg] = -1.0;
    } else if (std::isnan(contactTimingsPerLeg[leg].front().start)) {
      timeUntilNextTouchDownPerLeg[leg] = -1.0;
    } else {
      timeUntilNextTouchDownPerLeg[leg] = contactTimingsPerLeg[leg].front().start - currentTime;
    }
  }

  return timeUntilNextTouchDownPerLeg;
}

feet_array_t<scalar_t> getCurrentSwingPhasePerLeg(scalar_t currentTime, const feet_array_t<scalar_t>& swingDurationsOfCurrentGait,
                                                  const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<scalar_t> swingPhasePerLeg;

  // Convert mode sequence to a contact timing vector per leg.
  const auto& contactTimingsPerLeg = extractContactTimingsPerLeg(modeSchedule);

  for (int leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    if (startsWithStancePhase(contactTimingsPerLeg[leg]) || hasStartTime(contactTimingsPerLeg[leg].front()) ||
        swingDurationsOfCurrentGait[leg] <= 0)) {
      swingPhasePerLeg[leg] = -1.0;
    }
    else {
      swingPhasePerLeg[leg] = 1.0 - (contactTimingsPerLeg[leg].front().start - currentTime) / swingDurationsOfCurrentGait[leg];
    }
  }

  return swingPhasePerLeg;
}

std::vector<ContactTiming> extractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<ContactTiming> contactTimings;
  int currentPhase = 0;

  while (currentPhase < numPhases) {
    // Search where contact phase starts
    while (currentPhase < numPhases && !contactFlags[currentPhase]) {
      ++currentPhase;
    }
    if (currentPhase >= numPhases) {
      break;  // No more contact phases
    }

    // Register start of the contact phase
    const scalar_t startTime = (currentPhase == 0) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

    // Find when the contact phase ends
    while (currentPhase + 1 < numPhases && contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the contact phase
    const scalar_t endTime = (currentPhase + 1 >= numPhases) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

    // Add to phases
    contactTimings.push_back({startTime, endTime});
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