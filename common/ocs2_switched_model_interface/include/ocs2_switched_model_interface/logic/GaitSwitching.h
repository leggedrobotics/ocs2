//
// Created by rgrandia on 15.03.20.
//

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/Gait.h"

namespace switched_model {

/**
 * Selects the next gait from iterators. Repeating the final gait when the final gait is reached.
 * @tparam GaitIt : Gait iterator
 * @param currentGait : iterator pointing to the current gait
 * @param pastTheEndGait : iterator pointing part the end of the last gait to be considered. (e.g. obtained from vector::end())
 * @return next gait in the sequence
 */
template <typename GaitIt>
GaitIt nextGait(GaitIt currentGait, GaitIt pastTheEndGait) {
  GaitIt nextGait = currentGait + 1;
  if (nextGait == pastTheEndGait) {
    return currentGait;
  } else {
    return nextGait;
  }
}

/**
 * Recursively progresses to the next gait in a sequence while keeping track of the phase variable.
 * When the final gait is reached, it is repeated as long as required by the specified dt.
 *
 * @tparam GaitIt : Gait iterator
 * @param phase : phase variable in the current gait.
 * @param dt : time to progress in the gait sequence
 * @param currentGait : iterator pointing to the current gait
 * @param pastTheEndGait : iterator pointing part the end of the last gait to be considered. (e.g. obtained from vector::end())
 * @return {phase in new gait, iterator to newly active gait.
 */
template <typename GaitIt>
std::pair<scalar_t, GaitIt> advancePhase(scalar_t phase, scalar_t dt, GaitIt currentGait, GaitIt pastTheEndGait) {
  assert(isValidPhase(phase));
  assert(dt >= 0);

  // Phase change within current gait
  scalar_t dphase = dt / currentGait->duration;

  if (phase + dphase < 1.0) {  // Advance within current gait
    phase += dphase;
    return {phase, currentGait};
  } else {  // Advance to next gait
    const scalar_t dtRemaining = std::max(dt - timeLeftInGait(phase, *currentGait), 0.0);
    const GaitIt nexGait = nextGait(currentGait, pastTheEndGait);
    // Recurse by setting the phase to the beginning of the next phase
    return advancePhase(0.0, dtRemaining, nexGait, pastTheEndGait);
  }
}

template <typename GaitIt>
ocs2::ModeSchedule getModeSchedule(scalar_t phase, scalar_t t0, scalar_t timeHorizon, GaitIt currentGait, GaitIt pastTheEndGait) {
  assert(isValidPhase(phase));

  // Initialize with the current mode
  std::vector<scalar_t> evenTimes = {};
  std::vector<size_t> modeSchedule = {getModeFromPhase(phase, *currentGait)};

  const scalar_t tend = t0 + timeHorizon;
  scalar_t t = t0;
  while (t < tend) {
    scalar_t dt = timeLeftInMode(phase, *currentGait);
    t += dt;
    if (t < tend) {  // Next event is within horizon: Add the event time and advance the phase to that event
      std::tie(phase, currentGait) = advancePhase(phase, dt, currentGait, pastTheEndGait);
      size_t mode = getModeFromPhase(phase, *currentGait);
      if (mode != modeSchedule.back()) {  // Only add the mode if it is a switch w.r.t the last mode.
        evenTimes.push_back(t);
        modeSchedule.push_back(mode);
      }
    }
  }
  return {evenTimes, modeSchedule};
}

}  // namespace switched_model
