/*
 * FeetPlannerBase.h
 *
 *  Created on: Mar 8, 2018
 *      Author: farbod
 */

#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto FeetPlannerBase::updateFootSchedule(size_t footIndex, const size_array_t& phaseIDsStock, const bool_array_t& contactFlagStock) const
    -> std::pair<int_array_t, int_array_t> {
  const size_t numPhases = phaseIDsStock.size();

  int_array_t startTimeIndexStock(numPhases, 0);
  int_array_t finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing feet
  for (size_t i = 0; i < numPhases; i++) {
    if (!contactFlagStock[i]) {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto FeetPlannerBase::extractContactFlags(const size_array_t& phaseIDsStock) const -> std::array<bool_array_t, NUM_CONTACT_POINTS> {
  const size_t numPhases = phaseIDsStock.size();

  std::array<bool_array_t, NUM_CONTACT_POINTS> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), bool_array_t(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    std::array<bool, NUM_CONTACT_POINTS> contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> FeetPlannerBase::findIndex(size_t index, const bool_array_t& contactFlagStock) const {
  const size_t numPhases = contactFlagStock.size();

  // skip if it is a stance leg
  if (contactFlagStock[index]) {
    return {0, 0};
  }

  // find the starting time
  int startTimesIndex = -1;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = ip - 1;
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}

}  // namespace switched_model
