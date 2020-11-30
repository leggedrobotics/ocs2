/*
 * GaitAdaptation.h
 *
 *  Created on: Jun 4, 2020
 *      Author: Marko Bjelonic
 */

#pragma once

#include <ocs2_switched_model_interface/logic/Gait.h>

namespace switched_model {

/**
 * Removes the first swing phase for all legs flagged with earlyTouchdown
 */
std::function<void(scalar_t& currentPhase, switched_model::Gait& currentGait, scalar_t currTime, switched_model::Gait& nextGait)>
earlyTouchDownAdaptation(const switched_model::feet_array_t<bool>& earlyTouchDownPerLeg);

/** Gets the mode index of the next phase with the specified contact state, returns the size of the modesequence of no such contact state
 * was found */
int getModeIndexOfNextContactStateOfLeg(bool contact, int startModeIdx, size_t leg, const Gait& gait);

/** Adapts the mode sequence between the two mode ids of a specific leg to be with the specified flag. */
void setContactStateOfLegBetweenModes(bool contact, int startModeIdx, int lastModeIdx, size_t leg, Gait& gait);

}  // namespace switched_model
