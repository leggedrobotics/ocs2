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
auto FeetPlannerBase::updateFootSchedule(size_t footIndex, const size_array_t& phaseIDsStock, const bool_array_t& contactFlagStock)
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
auto FeetPlannerBase::extractContactFlags(const size_array_t& phaseIDsStock) -> std::array<bool_array_t, NUM_CONTACT_POINTS> {
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
std::pair<int, int> FeetPlannerBase::findIndex(size_t index, const bool_array_t& contactFlagStock) {
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int FeetPlannerBase::findTouchdownIndex(size_t index, const FeetPlannerBase::bool_array_t& contactFlagStock) {
  assert(!contactFlagStock.empty());
  assert(!contactFlagStock[index]);

  const int numPhases = contactFlagStock.size();

  for (int i = index; (i + 1) < numPhases; i++) {
    if (contactFlagStock[i + 1]) {
      return i;
    }
  }
  return numPhases - 1;
}

int FeetPlannerBase::findLiftoffIndex(size_t index, const FeetPlannerBase::bool_array_t& contactFlagStock) {
  assert(!contactFlagStock.empty());
  assert(contactFlagStock[index]);

  const int numPhases = contactFlagStock.size();

  for (int i = index; (i + 1) < numPhases; i++) {
    if (!contactFlagStock[i + 1]) {
      return i;
    }
  }
  return numPhases - 1;
}

std::vector<FeetPlannerBase::FootPhase> FeetPlannerBase::extractFootPhases(const std::vector<scalar_t>& eventTimes,
                                                                           const bool_array_t& contactFlags) {
  assert(eventTimes.size() + 1 == contactFlags.size());
  const int numPhases = contactFlags.size();

  std::vector<FeetPlannerBase::FootPhase> footPhases;
  int currentPhase = 0;
  while (currentPhase < numPhases) {
    // Register start of the phase
    FootPhase currentFootPhase;
    currentFootPhase.type = (contactFlags[currentPhase]) ? FootPhaseType::Stance : FootPhaseType::Swing;
    currentFootPhase.startTime = (currentPhase == 0) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

    // Find when the phase ends
    while (currentPhase + 1 < numPhases && contactFlags[currentPhase] == contactFlags[currentPhase + 1]) {
      ++currentPhase;
    }

    // Register end of the phase
    currentFootPhase.endTime = (currentPhase + 1 == numPhases) ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

    // Add to phases
    footPhases.push_back(std::move(currentFootPhase));
    ++currentPhase;
  }
  return footPhases;
}

std::vector<FeetPlannerBase::FootPhase> FeetPlannerBase::filterOutShortSwingPhases(const std::vector<FootPhase>& footPhases,
                                                                                   scalar_t minimumSwingduration) {
  std::vector<FeetPlannerBase::FootPhase> filteredPhases;

  // Keep the first phase
  filteredPhases.push_back(footPhases.front());

  // Filter intermediate phases
  for (int i = 1; (i + 1) < footPhases.size(); i++) {
    const auto& footPhase = footPhases[i];

    if (footPhase.endTime >= footPhase.startTime + minimumSwingduration || footPhase.type == FootPhaseType::Stance) {
      // phase is long enough or a stance phase -> keep it
      filteredPhases.push_back(footPhase);
    } else {
      // phase is too short. remove it and merge the stance phase before and after
      assert(filteredPhases.back().type == FootPhaseType::Stance);
      assert(footPhases[i + 1].type == FootPhaseType::Stance);
      filteredPhases.back().endTime = footPhases[i + 1].endTime;
      i++;  // skips past the next stance phase such that it is not added.
    }
  }

  // Keep the final phase
  filteredPhases.push_back(footPhases.back());

  return filteredPhases;
}

}  // namespace switched_model
