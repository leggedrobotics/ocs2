/*
 * FeetPlannerBase.h
 *
 *  Created on: Mar 8, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
void FeetPlannerBase<scalar_t, cpg_t>::getStartTimesIndices(std::array<int_array_t, 4>& startTimesIndices) const {
  startTimesIndices = startTimesIndices_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
void FeetPlannerBase<scalar_t, cpg_t>::getFinalTimesIndices(std::array<int_array_t, 4>& finalTimesIndices) const {
  finalTimesIndices = finalTimesIndices_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
void FeetPlannerBase<scalar_t, cpg_t>::updateFootSchedule(const size_t& footIndex, const size_array_t& phaseIDsStock,
                                                          const bool_array_t& contactFlagStock, int_array_t& startTimeIndexStock,
                                                          int_array_t& finalTimeIndexStock) const {
  const size_t numSubsystems = phaseIDsStock.size();

  startTimeIndexStock.resize(numSubsystems);
  finalTimeIndexStock.resize(numSubsystems);

  for (size_t i = 0; i < numSubsystems; i++) {
    // find the startTime and finalTime indices
    if (contactFlagStock[i] == false) {
      int startTimeIndex, finalTimeIndex;
      findIndex(i, contactFlagStock, startTimeIndex, finalTimeIndex);

      startTimeIndexStock[i] = startTimeIndex;
      finalTimeIndexStock[i] = finalTimeIndex;

    } else {
      startTimeIndexStock[i] = 0;
      finalTimeIndexStock[i] = 0;
    }

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
void FeetPlannerBase<scalar_t, cpg_t>::extractContactFlags(const size_array_t& phaseIDsStock,
                                                           std::array<bool_array_t, 4>& contactFlagStock) const {
  const size_t numPhases = phaseIDsStock.size();

  for (size_t j = 0; j < 4; j++) contactFlagStock[j].resize(numPhases);

  for (size_t i = 0; i < numPhases; i++)
    for (size_t j = 0; j < 4; j++) {
      std::array<bool, 4> contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
      contactFlagStock[j][i] = contactFlag[j];
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
void FeetPlannerBase<scalar_t, cpg_t>::findIndex(const size_t& index, const bool_array_t& contactFlagStock, int& startTimesIndex,
                                                 int& finalTimesIndex) const {
  const size_t numSubsystems = contactFlagStock.size();

  // skip if it is a stance leg
  if (contactFlagStock[index] == true) return;

  // find the starting time
  startTimesIndex = -1;
  for (int ip = index - 1; ip >= 0; ip--)
    if (contactFlagStock[ip] == true) {
      startTimesIndex = ip;
      break;
    }

  // find the final time
  finalTimesIndex = numSubsystems - 1;
  for (size_t ip = index + 1; ip < numSubsystems; ip++)
    if (contactFlagStock[ip] == true) {
      finalTimesIndex = ip - 1;
      break;
    }
}

}  // namespace switched_model
