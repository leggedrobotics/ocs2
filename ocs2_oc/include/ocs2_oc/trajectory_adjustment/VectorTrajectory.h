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

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_oc/oc_data/TimeDiscretization.h"
#include "ocs2_oc/trajectory_adjustment/TrajectorySpreading.h"

namespace ocs2 {

struct vector_trajectory_t {
  scalar_array_t timeTrajectory;
  size_array_t postEventIndices;

  vector_t final;
  vector_array_t preJumps;
  vector_array_t intermediates;

  /** Exchanges the content of trajectory */
  void swap(vector_trajectory_t& other) {
    timeTrajectory.swap(other.timeTrajectory);
    postEventIndices.swap(other.postEventIndices);
    final.swap(other.final);
    preJumps.swap(other.preJumps);
    intermediates.swap(other.intermediates);
  }

  /** Clears the content of the trajectory */
  void clear() {
    timeTrajectory.clear();
    postEventIndices.clear();
    final = vector_t();
    preJumps.clear();
    intermediates.clear();
  }

  /** Convert a vector array to a vector trajectory */
  static vector_trajectory_t fromVectorArray(const std::vector<AnnotatedTime>& time, vector_array_t&& vector_array) {
    vector_trajectory_t result;

    // Problem horizon
    const int N = static_cast<int>(time.size()) - 1;

    for (int i = 0; i < N; ++i) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        result.preJumps.push_back(std::move(vector_array[i]));
      } else {
        result.intermediates.push_back(std::move(vector_array[i]));
      }
    }
    result.final = std::move(vector_array[N]);

    return result;
  }

  /** Convert a vector trajectory to a vector array */
  static vector_array_t toVectorArray(const std::vector<AnnotatedTime>& time, vector_trajectory_t&& vector_trajectory) {
    vector_array_t result;

    // Problem horizon
    const int N = static_cast<int>(time.size()) - 1;
    result.reserve(N + 1);

    int intermediateIdx = 0;
    int preJumpIdx = 0;
    for (int i = 0; i < N; ++i) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        result.push_back(std::move(vector_trajectory.preJumps[preJumpIdx]));
        ++preJumpIdx;
      } else {
        result.push_back(std::move(vector_trajectory.intermediates[intermediateIdx]));
        ++intermediateIdx;
      }
    }
    result.push_back(std::move(vector_trajectory.final));

    return result;
  }
};

/**
 * Calculates the intermediate vector value at the given time.
 *
 * @param [in] trajectory: The trajectory
 * @param [in] time: The inquiry time
 * @return The vector value at the given time.
 */
inline vector_t getIntermediateValueAtTime(const vector_trajectory_t& trajectory, scalar_t time) {
  const auto indexAlpha = LinearInterpolation::timeSegment(time, trajectory.timeTrajectory);
  return LinearInterpolation::interpolate(indexAlpha, trajectory.intermediates);
}

/**
 * Adjusts in-place a vector trajectory based on the last changes in mode schedule using a TrajectorySpreading strategy.
 *
 * @param [in] oldModeSchedule: The old mode schedule associated to the trajectories which should be adjusted.
 * @param [in] newModeSchedule: The new mode schedule that should be adapted to.
 * @param [in, out] trajectory: The dual solution that is associated with the old mode schedule.
 * @returns the status of the devised trajectory spreading strategy.
 */
inline TrajectorySpreading::Status trajectorySpread(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                                                    vector_trajectory_t& trajectory) {
  // trajectory spreading
  constexpr bool debugPrint = false;
  TrajectorySpreading trajectorySpreading(debugPrint);
  const auto status = trajectorySpreading.set(oldModeSchedule, newModeSchedule, trajectory.timeTrajectory);

  // adjust final, pre-jump, intermediate, time, postEventIndices
  if (status.willTruncate) {
    trajectory.final = vector_t();
  }
  trajectory.preJumps = trajectorySpreading.extractEventsArray(trajectory.preJumps);
  trajectorySpreading.adjustTrajectory(trajectory.intermediates);
  trajectorySpreading.adjustTimeTrajectory(trajectory.timeTrajectory);
  trajectory.postEventIndices = trajectorySpreading.getPostEventIndices();

  return status;
}

/**
 * Adjusts a vector trajectory based on the last changes in mode schedule using a TrajectorySpreading strategy.
 *
 * @param [in] trajectorySpreading: An updated trajectorySpreading instance. In order to update trajectorySpreading
 *                                  call TrajectorySpreading::set.
 * @param [in] oldDualSolution: The dual solution that is associated with the old mode schedule.
 * @param [out] newDualSolution: The updated dual solution that is associated with the new mode schedule.
 */
inline void trajectorySpread(const TrajectorySpreading& trajectorySpreading, const vector_trajectory_t& oldTrajectory,
                             vector_trajectory_t& newTrajectory) {
  newTrajectory.clear();

  // adjust time and postEventIndices
  newTrajectory.timeTrajectory = oldTrajectory.timeTrajectory;
  trajectorySpreading.adjustTimeTrajectory(newTrajectory.timeTrajectory);
  newTrajectory.postEventIndices = trajectorySpreading.getPostEventIndices();

  // adjust final, pre-jump and intermediate
  if (!trajectorySpreading.getStatus().willTruncate) {
    newTrajectory.final = oldTrajectory.final;
  }
  newTrajectory.preJumps = trajectorySpreading.extractEventsArray(oldTrajectory.preJumps);
  newTrajectory.intermediates = oldTrajectory.intermediates;
  trajectorySpreading.adjustTrajectory(newTrajectory.intermediates);
}

}  // namespace ocs2
