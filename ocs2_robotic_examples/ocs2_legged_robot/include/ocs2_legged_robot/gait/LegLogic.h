#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_legged_robot/common/Types.h"

namespace ocs2 {
namespace legged_robot {

struct LegPhase {
  scalar_t phase;
  scalar_t duration;
};

struct ContactTiming {
  scalar_t start;
  scalar_t end;
};

struct SwingTiming {
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

inline bool hasStartTime(const SwingTiming& timing) {
  return !std::isnan(timing.start);
}
inline bool hasEndTime(const SwingTiming& timing) {
  return !std::isnan(timing.end);
}

inline bool startsWithSwingPhase(const std::vector<ContactTiming>& timings) {
  return timings.empty() || hasStartTime(timings.front());
}
inline bool startsWithContactPhase(const std::vector<ContactTiming>& timings) {
  return !startsWithSwingPhase(timings);
}
inline bool endsWithSwingPhase(const std::vector<ContactTiming>& timings) {
  return timings.empty() || hasEndTime(timings.back());
}
inline bool endsWithContactPhase(const std::vector<ContactTiming>& timings) {
  return !endsWithSwingPhase(timings);
}

inline bool startsWithContactPhase(const std::vector<SwingTiming>& timings) {
  return timings.empty() || hasStartTime(timings.front());
}
inline bool startsWithSwingPhase(const std::vector<SwingTiming>& timings) {
  return !startsWithContactPhase(timings);
}
inline bool endsWithContactPhase(const std::vector<SwingTiming>& timings) {
  return timings.empty() || hasEndTime(timings.back());
}
inline bool endsWithSwingPhase(const std::vector<SwingTiming>& timings) {
  return !endsWithContactPhase(timings);
}

inline bool touchesDownAtLeastOnce(const std::vector<ContactTiming>& timings) {
  return std::any_of(timings.begin(), timings.end(), [](const ContactTiming& timing) { return hasStartTime(timing); });
}

inline bool liftsOffAtLeastOnce(const std::vector<ContactTiming>& timings) {
  return !timings.empty() && hasEndTime(timings.front());
}

inline bool touchesDownAtLeastOnce(const std::vector<SwingTiming>& timings) {
  return !timings.empty() && hasEndTime(timings.front());
}

inline bool liftsOffAtLeastOnce(const std::vector<SwingTiming>& timings) {
  return std::any_of(timings.begin(), timings.end(), [](const SwingTiming& timing) { return hasStartTime(timing); });
}

/**
 * @brief Get the contact phase for all legs.
 * If leg in contact, returns a value between 0.0 (at start of contact phase) and 1.0 (at end of contact phase).
 * If leg not in contact (i.e. in swing), returns -1.0.
 * If mode schedule starts with contact phase, returns 1.0 during this phase.
 * If mode schedule ends with contact phase, returns 0.0 during this phase.
 * @param [in] time : Query time.
 * @param [in] modeSchedule : Mode schedule.
 * @return Contact phases for all legs.
 */
feet_array_t<LegPhase> getContactPhasePerLeg(scalar_t time, const ocs2::ModeSchedule& modeSchedule);

/**
 * @brief Get the swing phase for all legs.
 * If leg in swing, returns a value between 0.0 (at start of swing phase) and 1.0 (at end of swing phase).
 * If leg not in swing (i.e. in contact), returns -1.0.
 * If mode schedule starts with swing phase, returns 1.0 during this phase.
 * If mode schedule ends with swing phase, returns 0.0 during this phase.
 * @param [in] time : Query time.
 * @param [in] modeSchedule : Mode schedule.
 * @return Swing phases for all legs.
 */
feet_array_t<LegPhase> getSwingPhasePerLeg(scalar_t time, const ocs2::ModeSchedule& modeSchedule);

/** Extracts the contact timings for all legs from a modeSchedule */
feet_array_t<std::vector<ContactTiming>> extractContactTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule);

/** Extracts the swing timings for all legs from a modeSchedule */
feet_array_t<std::vector<SwingTiming>> extractSwingTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule);

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
 * Get {startTime, endTime} for all swing phases. Contact phases are always implied in between: endTime[i] < startTime[i+1]
 * times are NaN if they cannot be identified at the boundaries
 * Vector is empty if there are no swing phases
 */
std::vector<SwingTiming> extractSwingTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

/**
 * Extracts for each leg the contact sequence over the motion phase sequence.
 * @param modeSequence : Sequence of contact modes.
 * @return Sequence of contact flags per leg.
 */
feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& modeSequence);

}  // namespace legged_robot
}  // namespace ocs2
