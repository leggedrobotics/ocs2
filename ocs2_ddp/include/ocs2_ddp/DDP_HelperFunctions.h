/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_oc/oc_data/Metrics.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>
#include <ocs2_oc/rollout/RolloutBase.h>

#include "ocs2_ddp/DDP_Data.h"

namespace ocs2 {

/**
 * Computes cost, soft constraints and constraints values of each point in the the primalSolution rollout.
 *
 * @param [in] problem: A reference to the optimal control problem.
 * @param [in] primalSolution: The primal solution.
 * @param [out] metrics: The cost, soft constraints and constraints values of the primalSolution rollout.
 */
void computeRolloutMetrics(OptimalControlProblem& problem, const PrimalSolution& primalSolution, MetricsCollection& metrics);

/**
 * Calculates the PerformanceIndex associated to the input Metrics.
 *
 * @param [in] timeTrajectory: Time stamp of the rollout.
 * @param [in] metrics: The cost, soft constraints and constraints values of the primalSolution rollout.
 *
 * @return The PerformanceIndex of the trajectory.
 */
PerformanceIndex computeRolloutPerformanceIndex(const scalar_array_t& timeTrajectory, const MetricsCollection& metrics);

/**
 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
 * to integrate the system dynamics in time period [initTime, finalTime].
 *
 * @param [in] rollout: A reference to the rollout class.
 * @param [in] initTime: The initial time.
 * @param [in] initState: The initial state.
 * @param [in] finalTime: The final time.
 * @param [in, out] primalSolution: The resulting primal solution. Make sure that primalSolution::controllerPtr is set since
 *                                  the rollout is performed based on the controller stored in primalSolution. Moreover,
 *                                  except for StateTriggeredRollout, one should also set primalSolution::modeSchedule.
 *
 * @return average time step.
 */
scalar_t rolloutTrajectory(RolloutBase& rollout, scalar_t initTime, const vector_t& initState, scalar_t finalTime,
                           PrimalSolution& primalSolution);

/**
 * Computes the integral of the squared (IS) norm of the controller update.
 *
 * @param [in] controller: Input controller.
 * @return The integral of the squared (IS) norm of the controller update.
 */
scalar_t computeControllerUpdateIS(const LinearController& controller);

/**
 * Outputs a controller with the same time stamp and gains as unoptimizedController. However, bias is incremented based on:
 * biasArray = unoptimizedController.biasArray + stepLength * unoptimizedController.deltaBiasArray
 */
void incrementController(scalar_t stepLength, const LinearController& unoptimizedController, LinearController& controller);

/**
 * Retrieve time and post event trajectories of the current partition from the entire time and post event trajectories.
 * The resulting time and event indics are normalized to start integration from back.
 *
 *
 * The riccati equations are solved backwards in time
 * the normalizedTimeTrajectory time is therefore filled with negative time in the reverse order, for example:
 * nominalTime =      [0.0, 1.0, 2.0, ..., 10.0]
 * normalizedTime = [-10.0, ..., -2.0, -1.0, -0.0]
 *
 * The event indices are counted from the back the current partition, for example:
 * nominalTime =      [0.0, 1.0, 2.0(*), 3.0, 4.0(*)]
 * eventIndices = [2, 4]
 *
 * normalizedTime = [-4.0, -3.0(*), -2.0, -1.0(*), -0.0]
 * normalizedeventIndices = [1, 3]
 *
 * @param [in] partitionInterval: Current active interval
 * @param [in] timeTrajectory: The whole time trajectory
 * @param [in] postEventIndices: The post event index array
 * @param [out] normalizedTimeTrajectory: Nomalized time trajectory of the current interval
 * @param [out] normalizedPostEventIndices: Nomalized ost event index array of the current interval
 */
void retrieveActiveNormalizedTime(const std::pair<int, int>& partitionInterval, const scalar_array_t& timeTrajectory,
                                  const size_array_t& postEventIndices, scalar_array_t& normalizedTimeTrajectory,
                                  size_array_t& normalizedPostEventIndices);

/**
 * Gets a reference to the linear controller from the given primal solution.
 */
inline LinearController& getLinearController(PrimalSolution& primalSolution) {
  assert(dynamic_cast<LinearController*>(primalSolution.controllerPtr_.get()) != nullptr);
  return static_cast<LinearController&>(*primalSolution.controllerPtr_);
}

/**
 * Gets a const reference to the linear controller from the given primal solution.
 */
inline const LinearController& getLinearController(const PrimalSolution& primalSolution) {
  assert(dynamic_cast<const LinearController*>(primalSolution.controllerPtr_.get()) != nullptr);
  return static_cast<const LinearController&>(*primalSolution.controllerPtr_);
}

}  // namespace ocs2
