/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_oc/rollout/OperatingTrajectoriesRollout.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t OperatingTrajectoriesRollout::runImpl(time_interval_array_t timeIntervalArray, const vector_t& initState,
                                               ControllerBase* controller, scalar_array_t& timeTrajectory,
                                               size_array_t& postEventIndicesStock, vector_array_t& stateTrajectory,
                                               vector_array_t& inputTrajectory, std::vector<ModelDataBase>* modelDataTrajectoryPtr) {
  const int numSubsystems = timeIntervalArray.size();
  const int numEvents = numSubsystems - 1;

  // clearing the output trajectories
  timeTrajectory.clear();
  timeTrajectory.reserve(2 * numSubsystems);
  stateTrajectory.clear();
  stateTrajectory.reserve(2 * numSubsystems);
  inputTrajectory.clear();
  inputTrajectory.reserve(2 * numSubsystems);
  postEventIndicesStock.clear();
  postEventIndicesStock.reserve(numEvents);

  vector_t beginState = initState;
  scalar_t beginTime, endTime;
  for (int i = 0; i < numSubsystems; i++) {
    // get operating trajectories
    operatingTrajectoriesPtr_->getSystemOperatingTrajectories(beginState, timeIntervalArray[i].first, timeIntervalArray[i].second,
                                                              timeTrajectory, stateTrajectory, inputTrajectory, true);

    if (i < numEvents) {
      postEventIndicesStock.push_back(stateTrajectory.size());
      beginState = stateTrajectory.back();
    }

  }  // end of i loop

  // filling the model data
  if (modelDataTrajectoryPtr) {
    modelDataTrajectoryPtr->clear();
    modelDataTrajectoryPtr->reserve(timeTrajectory.size());
    for (int i = 0; i < timeTrajectory.size(); i++) {
      ModelDataBase modelDataTemp;
      modelDataTemp.time_ = timeTrajectory[i];
      modelDataTemp.stateDim_ = stateDim_;
      modelDataTemp.inputDim_ = inputDim_;
      modelDataTemp.dynamics_.setZero(stateDim_);
      modelDataTemp.dynamicsBias_.setZero(stateDim_);
      modelDataTrajectoryPtr->emplace_back(modelDataTemp);
    }  // end of i loop
  }

  return stateTrajectory.back();
}

}  // namespace ocs2
