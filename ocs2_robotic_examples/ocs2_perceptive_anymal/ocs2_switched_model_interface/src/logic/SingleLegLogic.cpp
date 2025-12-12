//
// Created by rgrandia on 27.04.20.
//

#include "ocs2_switched_model_interface/logic/SingleLegLogic.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

feet_array_t<std::vector<ContactTiming>> extractContactTimingsPerLeg(
    const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg =
      extractContactFlags(modeSchedule.modeSequence);

  // Extract timings per leg
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    contactTimingsPerLeg[leg] = extractContactTimings(
        modeSchedule.eventTimes, contactSequencePerLeg[leg]);
  }

  return contactTimingsPerLeg;
}

scalar_t getTimeOfNextLiftOff(
    scalar_t currentTime, const std::vector<ContactTiming>& contactTimings) {
  for (const auto& contactPhase : contactTimings) {
    if (hasEndTime(contactPhase) && contactPhase.end > currentTime) {
      return contactPhase.end;
    }
  }
  return timingNaN();
}

scalar_t getTimeOfNextTouchDown(
    scalar_t currentTime, const std::vector<ContactTiming>& contactTimings) {
  for (const auto& contactPhase : contactTimings) {
    if (hasStartTime(contactPhase) && contactPhase.start > currentTime) {
      return contactPhase.start;
    }
  }
  return timingNaN();
}

std::vector<ContactTiming> extractContactTimings(
    const std::vector<scalar_t>& eventTimes,
    const std::vector<bool>& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<ContactTiming> contactTimings;
  contactTimings.reserve(1 + eventTimes.size() / 2);  // Approximate upper bound
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
    const scalar_t startTime = (currentPhase == 0)
                                   ? std::numeric_limits<scalar_t>::quiet_NaN()
                                   : eventTimes[currentPhase - 1];

    // Find when the contact phase ends
    while (currentPhase + 1 < numPhases && contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the contact phase
    const scalar_t endTime = (currentPhase + 1 >= numPhases)
                                 ? std::numeric_limits<scalar_t>::quiet_NaN()
                                 : eventTimes[currentPhase];

    // Add to phases
    contactTimings.push_back({startTime, endTime});
    ++currentPhase;
  }
  return contactTimings;
}

feet_array_t<std::vector<bool>> extractContactFlags(
    const std::vector<size_t>& modeSequence) {
  const size_t numPhases = modeSequence.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(),
            std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(modeSequence[i]);
    for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

}  // namespace switched_model