//
// Created by rgrandia on 27.04.20.
//

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

struct ContactTiming {
  scalar_t start;
  scalar_t end;
};

inline scalar_t timingNaN() {
  return std::numeric_limits<scalar_t>::quiet_NaN();
}

inline bool hasStartTime(const ContactTiming& timing) {
  return !std::isnan(timing.start);
}
inline bool hasEndTime(const ContactTiming& timing) {
  return !std::isnan(timing.end);
}
inline bool startsWithSwingPhase(const std::vector<ContactTiming>& timings) {
  return timings.empty() || hasStartTime(timings.front());
}
inline bool startsWithStancePhase(const std::vector<ContactTiming>& timings) {
  return !startsWithSwingPhase(timings);
}
inline bool endsWithSwingPhase(const std::vector<ContactTiming>& timings) {
  return timings.empty() || hasEndTime(timings.back());
}
inline bool endsWithStancePhase(const std::vector<ContactTiming>& timings) {
  return !endsWithSwingPhase(timings);
}

inline bool touchesDownAtLeastOnce(const std::vector<ContactTiming>& timings) {
  return std::any_of(timings.begin(), timings.end(), [](const ContactTiming& timing) { return hasStartTime(timing); });
}

inline bool liftsOffAtLeastOnce(const std::vector<ContactTiming>& timings) {
  return !timings.empty() && hasEndTime(timings.front());
}

/** Extracts the contact timings for all legs from a modeSchedule */
feet_array_t<std::vector<ContactTiming>> extractContactTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule);

/** Returns time of the next lift off. Returns nan if leg is not lifting off */
scalar_t getTimeOfNextLiftOff(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/** Returns time of the  touch down for all legs from a modeschedule. Returns nan if leg does not touch down */
scalar_t getTimeOfNextTouchDown(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/**
 * Get {startTime, endTime} for all contact phases. Swingphases are always implied in between: endTime[i] < startTime[i+1]
 * times are NaN if they cannot be identified at the boundaries
 * Vector is empty if there are no contact phases
 */
std::vector<ContactTiming> extractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

/**
 * Extracts for each leg the contact sequence over the motion phase sequence.
 * @param modeSequence : Sequence of contact modes.
 * @return Sequence of contact flags per leg.
 */
feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& modeSequence);

}  // namespace switched_model
