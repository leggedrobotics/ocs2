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

#include "ocs2_ipm/IpmTrajectoryAdjustment.h"

#include <ocs2_oc/trajectory_adjustment/TrajectorySpreading.h>
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreadingHelperFunctions.h>

namespace ocs2 {
namespace {

MultiplierCollection toMultiplierCollection(vector_t&& stateIneq, vector_t&& stateInputIneq = vector_t()) {
  MultiplierCollection multiplierCollection;
  multiplierCollection.stateIneq.emplace_back(0.0, std::move(stateIneq));
  multiplierCollection.stateInputIneq.emplace_back(0.0, std::move(stateInputIneq));
  return multiplierCollection;
}

std::pair<vector_t, vector_t> fromMultiplierCollection(MultiplierCollection&& multiplierCollection) {
  return std::make_pair(std::move(multiplierCollection.stateIneq.front().lagrangian),
                        std::move(multiplierCollection.stateInputIneq.front().lagrangian));
}
}  // namespace
}  // namespace ocs2

namespace ocs2 {
namespace ipm {

DualSolution toDualSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& stateIneq, vector_array_t&& stateInputIneq) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  DualSolution dualSolution;
  const auto newTimeTrajectory = toInterpolationTime(time);
  dualSolution.postEventIndices = toPostEventIndices(time);

  dualSolution.preJumps.reserve(dualSolution.postEventIndices.size());
  dualSolution.intermediates.reserve(time.size());

  for (int i = 0; i < N; ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent) {
      dualSolution.preJumps.emplace_back(toMultiplierCollection(std::move(stateIneq[i])));
      dualSolution.intermediates.push_back(dualSolution.intermediates.back());  // no event at the initial node
    } else {
      dualSolution.intermediates.emplace_back(toMultiplierCollection(std::move(stateIneq[i]), std::move(stateInputIneq[i])));
    }
  }
  dualSolution.final = toMultiplierCollection(std::move(stateIneq[N]));
  dualSolution.intermediates.push_back(dualSolution.intermediates.back());

  return dualSolution;
}

std::pair<vector_array_t, vector_array_t> fromDualSolution(const std::vector<AnnotatedTime>& time, DualSolution&& dualSolution) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  vector_array_t stateIneq;
  vector_array_t stateInputIneq;
  stateIneq.reserve(N + 1);
  stateInputIneq.reserve(N);

  int preJumpIdx = 0;
  for (int i = 0; i < N; ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent) {
      auto result = fromMultiplierCollection(std::move(dualSolution.preJumps[preJumpIdx]));
      stateIneq.emplace_back(std::move(result.first));
      stateInputIneq.emplace_back(std::move(result.second));
      ++preJumpIdx;
    } else {
      auto result = fromMultiplierCollection(std::move(dualSolution.intermediates[i]));
      stateIneq.emplace_back(std::move(result.first));
      stateInputIneq.emplace_back(std::move(result.second));
    }
  }

  auto result = fromMultiplierCollection(std::move(dualSolution.final));
  stateIneq.emplace_back(std::move(result.first));

  return std::make_pair(std::move(stateIneq), std::move(stateInputIneq));
}

std::pair<vector_array_t, vector_array_t> interpolateInteriorPointTrajectory(const ModeSchedule& oldModeSchedule,
                                                                             const ModeSchedule& newModeSchedule,
                                                                             const std::vector<AnnotatedTime>& time,
                                                                             DualSolution&& oldDualSolution) {
  const auto oldTimeTrajectory = oldDualSolution.timeTrajectory;
  const auto oldPostEventIndices = oldDualSolution.postEventIndices;

  if (!oldTimeTrajectory.empty()) {
    std::ignore = trajectorySpread(oldModeSchedule, newModeSchedule, oldDualSolution);
  }

  const auto newTimeTrajectory = toInterpolationTime(time);
  const auto newPostEventIndices = toPostEventIndices(time);

  // find the time period that we can interpolate the cached solution
  const auto timePeriod = std::make_pair(newTimeTrajectory.front(), newTimeTrajectory.back());
  const auto interpolatableTimePeriod = findIntersectionToExtendableInterval(oldTimeTrajectory, newModeSchedule.eventTimes, timePeriod);
  const bool interpolateTillFinalTime = numerics::almost_eq(interpolatableTimePeriod.second, timePeriod.second);

  DualSolution newDualSolution;

  // set time and post-event indices
  newDualSolution.timeTrajectory = std::move(newTimeTrajectory);
  newDualSolution.postEventIndices = std::move(newPostEventIndices);

  // final
  if (interpolateTillFinalTime) {
    newDualSolution.final = std::move(oldDualSolution.final);
  } else {
    newDualSolution.final = toMultiplierCollection(vector_t());
  }

  // pre-jumps
  newDualSolution.preJumps.resize(newDualSolution.postEventIndices.size());
  if (!newDualSolution.postEventIndices.empty()) {
    const auto firstEventTime = newDualSolution.timeTrajectory[newDualSolution.postEventIndices[0] - 1];
    const auto cacheEventIndexBias = getNumberOfPrecedingEvents(oldTimeTrajectory, oldPostEventIndices, firstEventTime);

    for (size_t i = 0; i < newDualSolution.postEventIndices.size(); i++) {
      const auto cachedTimeIndex = cacheEventIndexBias + i;
      if (cachedTimeIndex < oldDualSolution.preJumps.size()) {
        newDualSolution.preJumps[i] = std::move(oldDualSolution.preJumps[cachedTimeIndex]);
      } else {
        newDualSolution.preJumps[i] = toMultiplierCollection(vector_t());
      }
    }
  }

  // intermediates
  newDualSolution.intermediates.resize(newDualSolution.timeTrajectory.size() - 1);
  for (size_t i = 0; i < newDualSolution.timeTrajectory.size() - 1; i++) {
    const auto time = newDualSolution.timeTrajectory[i];

    if (interpolatableTimePeriod.first <= time && time <= interpolatableTimePeriod.second) {
      newDualSolution.intermediates[i] = getIntermediateDualSolutionAtTime(oldDualSolution, time);
    } else {
      newDualSolution.intermediates[i] = toMultiplierCollection(vector_t(), vector_t());
    }
  }

  return fromDualSolution(time, std::move(newDualSolution));
}

}  // namespace ipm
}  // namespace ocs2
