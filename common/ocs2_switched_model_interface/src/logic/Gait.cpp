//
// Created by rgrandia on 15.03.20.
//

#include "ocs2_switched_model_interface/logic/Gait.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace switched_model {

bool isValidGait(const Gait& gait) {
  bool validGait = true;
  validGait &= gait.duration > 0;
  validGait &= std::all_of(gait.eventPhases.begin(), gait.eventPhases.end(), [](double phase) { return 0 < phase && phase < 1; });
  validGait &= std::is_sorted(gait.eventPhases.begin(), gait.eventPhases.end());
  validGait &= gait.eventPhases.size() + 1 == gait.modeSequence.size();
  return validGait;
}

bool isValidPhase(double phase) {
  return phase >= 0 && phase < 1.0;
}

double wrapPhase(double phase) {
  phase = std::fmod(phase, 1.0);
  if (phase < 0) {
    phase += 1.0;
  }
  return phase;
}

int getCurrentModeIndex(double phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  auto firstLargerValueIterator = std::upper_bound(gait.eventPhases.begin(), gait.eventPhases.end(), phase);
  return static_cast<int>(firstLargerValueIterator - gait.eventPhases.begin());
}

size_t getModeFromPhase(double phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  return gait.modeSequence[getCurrentModeIndex(phase, gait)];
}

double timeLeftInGait(double phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  return (1.0 - phase) * gait.duration;
}

double timeLeftInMode(double phase, const Gait& gait) {
  assert(isValidPhase(phase));
  assert(isValidGait(gait));
  int modeIndex = getCurrentModeIndex(phase, gait);
  if (modeIndex < gait.eventPhases.size()) {
    return (gait.eventPhases[modeIndex] - phase) * gait.duration;
  } else {
    return timeLeftInGait(phase, gait);
  }
}

}  // namespace switched_model
