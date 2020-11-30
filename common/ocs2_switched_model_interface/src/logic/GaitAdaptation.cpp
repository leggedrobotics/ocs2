/*
 * GaitAdaptation.cpp
 *
 *  Created on: Jun 4, 2020
 *      Author: Marko Bjelonic
 */

// anymal ctrl ocs2
#include "ocs2_switched_model_interface/logic/GaitAdaptation.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

std::function<void(scalar_t& currentPhase, switched_model::Gait& currentGait, scalar_t currTime, switched_model::Gait& nextGait)>
earlyTouchDownAdaptation(const switched_model::feet_array_t<bool>& earlyTouchDownPerLeg) {
  return [=](scalar_t& currentPhase, Gait& currentGait, scalar_t currTime, Gait& nextGait) {
    const int currentModeIndex = getModeIndexFromPhase(currentPhase, currentGait);
    for (unsigned int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      if (earlyTouchDownPerLeg[leg]) {
        const int modeIndexOfNextContact = getModeIndexOfNextContactStateOfLeg(true, currentModeIndex, leg, currentGait);

        if (modeIndexOfNextContact < currentGait.modeSequence.size()) {  // update current gait
          setContactStateOfLegBetweenModes(true, currentModeIndex, modeIndexOfNextContact, leg, currentGait);
        } else {  // update current gait till end, and next gait
          setContactStateOfLegBetweenModes(true, currentModeIndex, modeIndexOfNextContact - 1, leg, currentGait);

          int modeIndexOfNextContactOfNextGait = getModeIndexOfNextContactStateOfLeg(true, 0, leg, nextGait);
          if (modeIndexOfNextContactOfNextGait == nextGait.modeSequence.size()) {
            --modeIndexOfNextContactOfNextGait;
          }
          setContactStateOfLegBetweenModes(true, 0, modeIndexOfNextContactOfNextGait, leg, nextGait);
        }
      }
    }
  };
}

int getModeIndexOfNextContactStateOfLeg(bool contact, int startModeIdx, size_t leg, const Gait& gait) {
  int modeIndex = startModeIdx;
  while (modeIndex < gait.modeSequence.size()) {
    size_t currentMode = gait.modeSequence[modeIndex];
    if (modeNumber2StanceLeg(currentMode)[leg] == contact) {
      break;
    } else {
      ++modeIndex;
    }
  }
  return modeIndex;
}

void setContactStateOfLegBetweenModes(bool contact, int startModeIdx, int lastModeIdx, size_t leg, Gait& gait) {
  for (int modeIndex = startModeIdx; modeIndex <= lastModeIdx; ++modeIndex) {
    auto stanceLegs = switched_model::modeNumber2StanceLeg(gait.modeSequence[modeIndex]);
    stanceLegs[leg] = contact;
    gait.modeSequence[modeIndex] = switched_model::stanceLeg2ModeNumber(stanceLegs);
  }
}

}  // namespace switched_model
