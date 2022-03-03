/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_oc/oc_data/PrimalSolution.h"
#include "ocs2_oc/trajectory_adjustment/TrajectorySpreading.h"

namespace ocs2 {

/**
 * Adjusts a linear controller based on the last changes in model schedule using a TrajectorySpreading strategy.
 *
 * @param [in] oldModeSchedule: The old mode schedule associated to the trajectories which should be adjusted.
 * @param [in] newModeSchedule: The new mode schedule that should be adapted to.
 * @param [in, out] controller: The control policy that is associated with the old mode schedule.
 * @returns the status of the devised trajectory spreading strategy.
 */
inline TrajectorySpreading::Status trajectorySpread(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                                                    LinearController& controller) {
  // trajectory spreading
  constexpr bool debugPrint = false;
  TrajectorySpreading trajectorySpreading(debugPrint);
  const auto status = trajectorySpreading.set(oldModeSchedule, newModeSchedule, controller.timeStamp_);

  // adjust bias, gain, and time
  trajectorySpreading.adjustTrajectory(controller.biasArray_);
  trajectorySpreading.adjustTrajectory(controller.gainArray_);
  trajectorySpreading.adjustTimeTrajectory(controller.timeStamp_);

  return status;
}

/**
 * Adjusts a primal solution based on the last changes in model schedule using a TrajectorySpreading strategy.
 * Note: PrimalSolution::controllerPtr_ will not be adjusted.
 *
 * @param [in] oldModeSchedule: The old mode schedule associated to the trajectories which should be adjusted.
 * @param [in] newModeSchedule: The new mode schedule that should be adapted to.
 * @param [in, out] primalSolution: The primal solution that is associated with the old mode schedule.
 * @returns the status of the devised trajectory spreading strategy.
 */
inline TrajectorySpreading::Status trajectorySpread(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                                                    PrimalSolution& primalSolution) {
  // trajectory spreading
  constexpr bool debugPrint = false;
  TrajectorySpreading trajectorySpreading(debugPrint);
  const auto status = trajectorySpreading.set(oldModeSchedule, newModeSchedule, primalSolution.timeTrajectory_);

  // adjust modeSchedule, state, input, time, postEventIndices
  primalSolution.modeSchedule_ = newModeSchedule;
  trajectorySpreading.adjustTrajectory(primalSolution.stateTrajectory_);
  trajectorySpreading.adjustTrajectory(primalSolution.inputTrajectory_);
  trajectorySpreading.adjustTimeTrajectory(primalSolution.timeTrajectory_);
  primalSolution.postEventIndices_ = trajectorySpreading.getPostEventIndices();

  return status;
}

}  // namespace ocs2
