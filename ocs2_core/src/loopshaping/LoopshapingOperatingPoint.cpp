//
// Created by rgrandia on 14.02.20.
//

#include <iostream>

#include <ocs2_core/loopshaping/LoopshapingOperatingPoint.h>

namespace ocs2 {

void LoopshapingOperatingPoint::getSystemOperatingTrajectories(const vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                                                               scalar_array_t& timeTrajectory, vector_array_t& stateTrajectory,
                                                               vector_array_t& inputTrajectory, bool concatOutput /* = false*/) {
  if (!concatOutput) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    inputTrajectory.clear();
  }

  scalar_array_t systemTimeTrajectory;
  vector_array_t systemStateTrajectory;
  vector_array_t systemInputTrajectory;
  vector_t initialSystemState = loopshapingDefinition_->getSystemState(initialState);

  // Get system operating point
  systembase_->getSystemOperatingTrajectories(initialSystemState, startTime, finalTime, systemTimeTrajectory, systemStateTrajectory,
                                              systemInputTrajectory, false);

  // Filter operating point
  vector_t equilibriumFilterState;
  vector_t equilibriumFilterInput;
  for (int k = 0; k < systemTimeTrajectory.size(); ++k) {
    loopshapingDefinition_->getFilterEquilibrium(systemInputTrajectory[k], equilibriumFilterState, equilibriumFilterInput);
    timeTrajectory.emplace_back(systemTimeTrajectory[k]);
    inputTrajectory.emplace_back(loopshapingDefinition_->concatenateSystemAndFilterInput(systemInputTrajectory[k], equilibriumFilterInput));
    stateTrajectory.emplace_back(loopshapingDefinition_->concatenateSystemAndFilterState(systemStateTrajectory[k], equilibriumFilterState));
  }
}

}  // namespace ocs2
