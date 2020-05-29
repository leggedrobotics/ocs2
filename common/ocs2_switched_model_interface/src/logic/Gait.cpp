//
// Created by rgrandia on 15.03.20.
//

#include "ocs2_switched_model_interface/logic/Gait.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

#include <ocs2_core/misc/Display.h>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace switched_model {

bool isValidGait(const Gait& gait) {
  bool validGait = true;
  validGait &= gait.duration > 0;
  validGait &= std::all_of(gait.eventPhases.begin(), gait.eventPhases.end(), [](scalar_t phase) { return 0 < phase && phase < 1; });
  validGait &= std::is_sorted(gait.eventPhases.begin(), gait.eventPhases.end());
  validGait &= gait.eventPhases.size() + 1 == gait.modeSequence.size();
  return validGait;
}

bool isValidPhase(scalar_t phase) {
  return phase >= 0 && phase < 1.0;
}

scalar_t wrapPhase(scalar_t phase) {
  phase = std::fmod(phase, 1.0);
  if (phase < 0) {
    phase += 1.0;
  }
  return phase;
}

int getCurrentModeIndex(scalar_t phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  auto firstLargerValueIterator = std::upper_bound(gait.eventPhases.begin(), gait.eventPhases.end(), phase);
  return static_cast<int>(firstLargerValueIterator - gait.eventPhases.begin());
}

size_t getModeFromPhase(scalar_t phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  return gait.modeSequence[getCurrentModeIndex(phase, gait)];
}

scalar_t timeLeftInGait(scalar_t phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  return (1.0 - phase) * gait.duration;
}

scalar_t timeLeftInMode(scalar_t phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  int modeIndex = getCurrentModeIndex(phase, gait);
  if (modeIndex < gait.eventPhases.size()) {
    return (gait.eventPhases[modeIndex] - phase) * gait.duration;
  } else {
    return timeLeftInGait(phase, gait);
  }
}

feet_array_t<scalar_t> getCurrentSwingPhasePerLeg(scalar_t phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  feet_array_t<scalar_t> swingPhasePerLeg;

  int modeIndex = getCurrentModeIndex(phase, gait);
  const auto& stanceLegs = modeNumber2StanceLeg(gait.modeSequence[modeIndex]);

  for (int leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    if (stanceLegs[leg]) {
      swingPhasePerLeg[leg] = -1.0;
    } else {
      scalar_t liftOffPhase;
      scalar_t touchDownPhase;
      if (modeIndex < gait.eventPhases.size()) {
        // Get touchdown index.
        int touchDownIndex = modeIndex;
        while (touchDownIndex < gait.eventPhases.size()) {
          if (modeNumber2StanceLeg(gait.modeSequence[touchDownIndex + 1])[leg]) {
            break;
          } else {
            touchDownIndex++;
          }
        }
        liftOffPhase = gait.eventPhases[modeIndex - 1];
        if (touchDownIndex < gait.eventPhases.size()) {
          touchDownPhase = gait.eventPhases[touchDownIndex];
        } else {
          touchDownPhase = 1.0;
        }
      } else {
        liftOffPhase = gait.eventPhases.back();
        touchDownPhase = 1.0;
      }
      swingPhasePerLeg[leg] = (phase - liftOffPhase) / (touchDownPhase - liftOffPhase);
    }
  }
  return swingPhasePerLeg;
}

std::ostream& operator<<(std::ostream& stream, const Gait& gait) {
  stream << "Duration:       " << gait.duration << "\n";
  stream << "Event phases:  {" << ocs2::toDelimitedString(gait.eventPhases) << "}\n";
  stream << "Mode sequence: {" << ocs2::toDelimitedString(gait.modeSequence) << "}\n";
  return stream;
}

}  // namespace switched_model
