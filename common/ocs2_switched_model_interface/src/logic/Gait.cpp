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

int getModeIndexFromPhaseUntilNextTouchDownOfLeg(scalar_t phase, int leg, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  int modeIndex = getCurrentModeIndex(phase, gait);
  while (modeIndex < gait.modeSequence.size()) {
    size_t currentMode = gait.modeSequence[modeIndex];
    if (modeNumber2StanceLeg(currentMode)[leg]) {
      break;
    } else {
      ++modeIndex;
    }
  }
  if (modeIndex >= gait.modeSequence.size()) {
    modeIndex = -1;
  }
  return modeIndex;
}

int getModeIndexFromPhaseUntilLastTouchDownOfLeg(scalar_t phase, int leg, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  int modeIndex = getCurrentModeIndex(phase, gait);
  while (modeIndex >= 0) {
    size_t currentMode = gait.modeSequence[modeIndex];
    if (modeNumber2StanceLeg(currentMode)[leg]) {
      break;
    } else {
      --modeIndex;
    }
  }
  if (modeIndex < 0) {
    modeIndex = -1;
  }
  return modeIndex;
}

void setContactStateOfLegToContactBetweenModes(int startModeId, int lastModeId, int leg, Gait& gait) {
  assert(startModeId < gait.modeSequence.size());
  assert(lastModeId < gait.modeSequence.size());
  for (int modeIndex = startModeId; modeIndex <= lastModeId; ++modeIndex) {
    auto stanceLegs = switched_model::modeNumber2StanceLeg(gait.modeSequence[modeIndex]);
    stanceLegs[leg] = true;
    gait.modeSequence[modeIndex] = switched_model::stanceLeg2ModeNumber(stanceLegs);
  }
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

  const int currentModeIndex = getCurrentModeIndex(phase, gait);
  const auto& stanceLegs = modeNumber2StanceLeg(gait.modeSequence[currentModeIndex]);

  for (int leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    if (stanceLegs[leg]) {
      swingPhasePerLeg[leg] = -1.0;
    } else {
      const int modeIndexUntilNextTouchDown = getModeIndexFromPhaseUntilNextTouchDownOfLeg(phase, leg, gait);
      const int modeIndexUntilLastTouchDown = getModeIndexFromPhaseUntilLastTouchDownOfLeg(phase, leg, gait);
      scalar_t liftOffPhase;
      if (modeIndexUntilLastTouchDown < 0) {
        liftOffPhase = 0.0;
      } else {
        liftOffPhase = gait.eventPhases[modeIndexUntilLastTouchDown];
      }
      scalar_t touchDownPhase;
      if (modeIndexUntilNextTouchDown >= gait.modeSequence.size() || modeIndexUntilNextTouchDown == -1) {
        touchDownPhase = 1.0;
      } else {
        touchDownPhase = gait.eventPhases[modeIndexUntilNextTouchDown - 1];
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
