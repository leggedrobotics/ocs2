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

#include <ocs2_oc/oc_data/DualSolution.h>
#include <ocs2_oc/oc_data/TimeDiscretization.h>

namespace ocs2 {
namespace ipm {

/**
 * Convert the optimized slack or dual trajectories as a DualSolution.
 *
 * @param time: The time discretization.
 * @param stateIneq: The slack/dual of the state inequality constraints.
 * @param stateInputIneq: The slack/dual of the state-input inequality constraints.
 * @return A dual solution.
 */
DualSolution toDualSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& stateIneq, vector_array_t&& stateInputIneq);

/**
 * Convert the optimized slack or dual trajectories as a DualSolution.
 *
 * @param time: The time discretization.
 * @param dualSolution: The dual solution.
 * @return The slack/dual of the state inequality constraints (first) and state-input inequality constraints (second).
 */
std::pair<vector_array_t, vector_array_t> fromDualSolution(const std::vector<AnnotatedTime>& time, DualSolution&& dualSolution);

/**
 * Initializes the interior point trajectory.
 *
 * @param time: The time discretization.
 * @param stateIneq: The slack/dual of the state inequality constraints.
 * @param stateInputIneq: The slack/dual of the state-input inequality constraints.
 * @return A dual solution.
 */
std::pair<vector_array_t, vector_array_t> initializeInteriorPointTrajectory(const ModeSchedule& oldModeSchedule,
                                                                            const ModeSchedule& newModeSchedule,
                                                                            const std::vector<AnnotatedTime>& time,
                                                                            DualSolution&& oldDualSolution);

}  // namespace ipm
}  // namespace ocs2