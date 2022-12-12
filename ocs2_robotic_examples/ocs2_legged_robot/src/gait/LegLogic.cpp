/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_legged_robot/gait/LegLogic.h"

#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"

namespace {

inline ocs2::scalar_t timingNaN() {
  return std::numeric_limits<ocs2::scalar_t>::quiet_NaN();
}

inline bool hasStartTime(const ocs2::legged_robot::ContactTiming& timing) {
  return !std::isnan(timing.start);
}
inline bool hasEndTime(const ocs2::legged_robot::ContactTiming& timing) {
  return !std::isnan(timing.end);
}

inline bool hasStartTime(const ocs2::legged_robot::SwingTiming& timing) {
  return !std::isnan(timing.start);
}
inline bool hasEndTime(const ocs2::legged_robot::SwingTiming& timing) {
  return !std::isnan(timing.end);
}

inline bool startsWithSwingPhase(const std::vector<ocs2::legged_robot::ContactTiming>& timings) {
  return timings.empty() || hasStartTime(timings.front());
}
inline bool startsWithContactPhase(const std::vector<ocs2::legged_robot::ContactTiming>& timings) {
  return !startsWithSwingPhase(timings);
}
inline bool endsWithSwingPhase(const std::vector<ocs2::legged_robot::ContactTiming>& timings) {
  return timings.empty() || hasEndTime(timings.back());
}
inline bool endsWithContactPhase(const std::vector<ocs2::legged_robot::ContactTiming>& timings) {
  return !endsWithSwingPhase(timings);
}

inline bool startsWithContactPhase(const std::vector<ocs2::legged_robot::SwingTiming>& timings) {
  return timings.empty() || hasStartTime(timings.front());
}
inline bool startsWithSwingPhase(const std::vector<ocs2::legged_robot::SwingTiming>& timings) {
  return !startsWithContactPhase(timings);
}
inline bool endsWithContactPhase(const std::vector<ocs2::legged_robot::SwingTiming>& timings) {
  return timings.empty() || hasEndTime(timings.back());
}
inline bool endsWithSwingPhase(const std::vector<ocs2::legged_robot::SwingTiming>& timings) {
  return !endsWithContactPhase(timings);
}

inline bool touchesDownAtLeastOnce(const std::vector<ocs2::legged_robot::ContactTiming>& timings) {
  return std::any_of(timings.begin(), timings.end(), [](const ocs2::legged_robot::ContactTiming& timing) { return hasStartTime(timing); });
}

inline bool liftsOffAtLeastOnce(const std::vector<ocs2::legged_robot::ContactTiming>& timings) {
  return !timings.empty() && hasEndTime(timings.front());
}

inline bool touchesDownAtLeastOnce(const std::vector<ocs2::legged_robot::SwingTiming>& timings) {
  return !timings.empty() && hasEndTime(timings.front());
}

inline bool liftsOffAtLeastOnce(const std::vector<ocs2::legged_robot::SwingTiming>& timings) {
  return std::any_of(timings.begin(), timings.end(), [](const ocs2::legged_robot::SwingTiming& timing) { return hasStartTime(timing); });
}

}  // anonymous namespace

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<LegPhase> getContactPhasePerLeg(scalar_t time, const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<LegPhase> contactPhasePerLeg;

  // Convert mode sequence to a contact timing vector per leg
  const auto contactTimingsPerLeg = extractContactTimingsPerLeg(modeSchedule);

  // Extract contact phases per leg
  for (size_t leg = 0; leg < contactPhasePerLeg.size(); ++leg) {
    if (contactTimingsPerLeg[leg].empty()) {
      // Leg is always in swing phase
      contactPhasePerLeg[leg].phase = -1.0;
      contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
    } else if (startsWithContactPhase(contactTimingsPerLeg[leg]) && (time <= contactTimingsPerLeg[leg].front().end)) {
      // It is assumed that contact phase started at minus infinity, so current time will be always close to ContactTiming.end
      contactPhasePerLeg[leg].phase = 1.0;
      contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else if (endsWithContactPhase(contactTimingsPerLeg[leg]) && (time >= contactTimingsPerLeg[leg].back().start)) {
      // It is assumed that contact phase ends at infinity, so current time will be always close to ContactTiming.start
      contactPhasePerLeg[leg].phase = 0.0;
      contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else {
      // Check if leg is in contact interval at current time
      auto it = std::find_if(contactTimingsPerLeg[leg].begin(), contactTimingsPerLeg[leg].end(),
                             [time](ContactTiming timing) { return (timing.start <= time) && (time <= timing.end); });
      if (it == contactTimingsPerLeg[leg].end()) {
        // Leg is not in contact for current time
        contactPhasePerLeg[leg].phase = -1.0;
        contactPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
      } else {
        // Leg is in contact for current time
        const auto& currentContactTiming = *it;
        contactPhasePerLeg[leg].phase = (time - currentContactTiming.start) / (currentContactTiming.end - currentContactTiming.start);
        contactPhasePerLeg[leg].duration = currentContactTiming.end - currentContactTiming.start;
      }
    }
  }

  return contactPhasePerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<LegPhase> getSwingPhasePerLeg(scalar_t time, const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<LegPhase> swingPhasePerLeg;

  // Convert mode sequence to a swing timing vector per leg
  const auto swingTimingsPerLeg = extractSwingTimingsPerLeg(modeSchedule);

  // Extract swing phases per leg
  for (size_t leg = 0; leg < swingPhasePerLeg.size(); ++leg) {
    if (swingTimingsPerLeg[leg].empty()) {
      // Leg is always in contact phase
      swingPhasePerLeg[leg].phase = -1.0;
      swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
    } else if (startsWithSwingPhase(swingTimingsPerLeg[leg]) && (time <= swingTimingsPerLeg[leg].front().end)) {
      // It is assumed that swing phase started at minus infinity, so current time will be always close to SwingTiming.end
      swingPhasePerLeg[leg].phase = 1.0;
      swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else if (endsWithSwingPhase(swingTimingsPerLeg[leg]) && (time >= swingTimingsPerLeg[leg].back().start)) {
      // It is assumed that swing phase ends at infinity, so current time will be always close to SwingTiming.start
      swingPhasePerLeg[leg].phase = 0.0;
      swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::infinity();
    } else {
      // Check if leg is in swing interval at current time
      auto it = std::find_if(swingTimingsPerLeg[leg].begin(), swingTimingsPerLeg[leg].end(),
                             [time](SwingTiming timing) { return (timing.start <= time) && (time <= timing.end); });
      if (it == swingTimingsPerLeg[leg].end()) {
        // Leg is not swinging for current time
        swingPhasePerLeg[leg].phase = scalar_t(-1.0);
        swingPhasePerLeg[leg].duration = std::numeric_limits<scalar_t>::quiet_NaN();
      } else {
        // Leg is swinging for current time
        const auto& currentSwingTiming = *it;
        swingPhasePerLeg[leg].phase = (time - currentSwingTiming.start) / (currentSwingTiming.end - currentSwingTiming.start);
        swingPhasePerLeg[leg].duration = currentSwingTiming.end - currentSwingTiming.start;
      }
    }
  }

  return swingPhasePerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<ContactTiming>> extractContactTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg = extractContactFlags(modeSchedule.modeSequence);

  // Extract timings per leg
  for (size_t leg = 0; leg < contactTimingsPerLeg.size(); ++leg) {
    contactTimingsPerLeg[leg] = extractContactTimings(modeSchedule.eventTimes, contactSequencePerLeg[leg]);
  }

  return contactTimingsPerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<SwingTiming>> extractSwingTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule) {
  feet_array_t<std::vector<SwingTiming>> swingTimingsPerLeg;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg = extractContactFlags(modeSchedule.modeSequence);

  // Extract timings per leg
  for (size_t leg = 0; leg < swingTimingsPerLeg.size(); ++leg) {
    swingTimingsPerLeg[leg] = extractSwingTimings(modeSchedule.eventTimes, contactSequencePerLeg[leg]);
  }

  return swingTimingsPerLeg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t getTimeOfNextLiftOff(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings) {
  for (const auto& contactPhase : contactTimings) {
    if (hasEndTime(contactPhase) && contactPhase.end > currentTime) {
      return contactPhase.end;
    }
  }
  return timingNaN();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t getTimeOfNextTouchDown(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings) {
  for (const auto& contactPhase : contactTimings) {
    if (hasStartTime(contactPhase) && contactPhase.start > currentTime) {
      return contactPhase.start;
    }
  }
  return timingNaN();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<ContactTiming> extractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags) {
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<SwingTiming> extractSwingTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<SwingTiming> swingTimings;
  swingTimings.reserve(1 + eventTimes.size() / 2);  // Approximate upper bound
  int currentPhase = 0;

  while (currentPhase < numPhases) {
    // Search where swing phase starts
    while (currentPhase < numPhases && contactFlags[currentPhase]) {
      ++currentPhase;
    }
    if (currentPhase >= numPhases) {
      break;  // No more swing phases
    }

    // Register start of the swing phase
    const scalar_t startTime = (currentPhase == 0) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

    // Find when the swing phase ends
    while (currentPhase + 1 < numPhases && !contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the contact phase
    const scalar_t endTime = (currentPhase + 1 >= numPhases) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

    // Add to phases
    swingTimings.push_back({startTime, endTime});
    ++currentPhase;
  }
  return swingTimings;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& modeSequence) {
  const size_t numPhases = modeSequence.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(modeSequence[i]);
    for (size_t j = 0; j < contactFlagStock.size(); j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

}  // namespace legged_robot
}  // namespace ocs2
