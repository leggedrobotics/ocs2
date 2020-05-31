//
// Created by rgrandia on 15.03.20.
//

#pragma once

#include <ostream>
#include <vector>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * A gait is a periodic mode schedule parameterized by a "phase" variable.
 *
 * The eventPhases only indicate switches of modes, i.e. phase = 0 and phase = 1 are not part of the eventPhases.
 * The number of modes is therefore number of phases + 1
 *
 * The conversion to time is regulated by a duration
 */
struct Gait {
  /** time for one gait cycle*/
  scalar_t duration;
  /** points in (0.0, 1.0) along the gait cycle where the contact mode changes, size N-1 */
  std::vector<scalar_t> eventPhases;
  /** sequence of contact modes, size: N */
  std::vector<size_t> modeSequence;
};

/**
 * isValidGait checks the following properties
 * - positive duration
 * - eventPhases are all in (0.0, 1.0)
 * - eventPhases are sorted
 * - the size of the modeSequences is 1 more than the eventPhases.
 */
bool isValidGait(const Gait& gait);

/** Get the current mode index. */
int getCurrentModeIndex(scalar_t phase, const Gait& gait);

/** Check is if the phase is in [0.0, 1.0) */
bool isValidPhase(scalar_t phase);

/** Wraps a phase to [0.0, 1.0) */
scalar_t wrapPhase(scalar_t phase);

/** The modes are selected with a closed-open interval: [ ) */
int getModeIndexFromPhase(scalar_t phase, const Gait& gait);

/** Gets the mode index until next touch down from the phase variable and returns -1 if there is no last touch down after the phase */
int getModeIndexFromPhaseUntilNextTouchDownOfLeg(scalar_t phase, int leg, const Gait& gait);

/** Gets the mode index until last touch down from the phase variable and returns -1 if there is no last touch down before the phase. */
int getModeIndexFromPhaseUntilLastTouchDownOfLeg(scalar_t phase, int leg, const Gait& gait);

/** Gets the active mode from the phase variable */
size_t getModeFromPhase(scalar_t phase, const Gait& gait);

/** Returns the time left in the gait based on the phase variable */
scalar_t timeLeftInGait(scalar_t phase, const Gait& gait);

/** Returns the time left in the current based on the phase variable */
scalar_t timeLeftInMode(scalar_t phase, const Gait& gait);

/** Returns current swing phase per leg [0.0, 1.0) or -1.0 if standing */
feet_array_t<scalar_t> getCurrentSwingPhasePerLeg(scalar_t phase, const Gait& gait);

/** Print gait */
std::ostream& operator<<(std::ostream& stream, const Gait& gait);

}  // namespace switched_model
