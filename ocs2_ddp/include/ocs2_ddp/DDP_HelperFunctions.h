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
#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_oc/oc_data/DualSolution.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/RolloutBase.h>

#include "ocs2_ddp/DDP_Data.h"

namespace ocs2 {

/**
 * Computes cost, soft constraints and constraints values of each point in the the primalSolution rollout.
 *
 * @param [in] problem: A reference to the optimal control problem.
 * @param [in] primalSolution: The primal solution.
 * @param [in] dualSolution: Const reference view to the dual solution
 * @param [out] problemMetrics: The cost, soft constraints and constraints values of the rollout.
 */
void computeRolloutMetrics(OptimalControlProblem& problem, const PrimalSolution& primalSolution, DualSolutionConstRef dualSolution,
                           ProblemMetrics& problemMetrics);

/**
 * Calculates the PerformanceIndex associated to the given ProblemMetrics.
 *
 * @param [in] timeTrajectory: Time stamp of the rollout.
 * @param [in] problemMetrics: The cost, soft constraints and constraints values of the rollout.
 *
 * @return The PerformanceIndex of the trajectory.
 */
PerformanceIndex computeRolloutPerformanceIndex(const scalar_array_t& timeTrajectory, const ProblemMetrics& problemMetrics);

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
 * Projects the unconstrained LQ coefficients to constrained ones.
 *
 * @param [in] modelData: The model data.
 * @param [in] constraintRangeProjector: The projection matrix to the constrained subspace.
 * @param [in] constraintNullProjector: The projection matrix to the null space of constrained.
 * @param [out] projectedModelData: The projected model data.
 */
void projectLQ(const ModelData& modelData, const matrix_t& constraintRangeProjector, const matrix_t& constraintNullProjector,
               ModelData& projectedModelData);

/**
 * Extract a primal solution for the range [initTime, finalTime] from a given primal solution. It assumes that the
 * given range is within the solution time of input primal solution.
 *
 * @note: The controller field is ignored.
 * @note: The extracted primal solution can have an event time at final time but ignores it at initial time.
 *
 * @param [in] timePeriod: The time period for which the solution should be extracted.
 * @param [in] inputPrimalSolution: The input PrimalSolution
 * @param [out] outputPrimalSolution: The output PrimalSolution.
 */
void extractPrimalSolution(const std::pair<scalar_t, scalar_t>& timePeriod, const PrimalSolution& inputPrimalSolution,
                           PrimalSolution& outputPrimalSolution);

/**
 * Calculates max feedforward update norm of the controller.
 *
 * @param [in] controller: Control policy
 * @return max feedforward update norm.
 */
scalar_t maxControllerUpdateNorm(const LinearController& controller);

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
 * Get the Partition Intervals From Time Trajectory. Intervals are defined as [start, end).
 *
 * Pay attention, the rightmost index of the end partition is (..., timeArray.size() - 1) , as the last value function is filled manually.
 * The reason is though we don’t write to the end index, we do have to read it. Adding the last index to the final partition will
 * cause a segmentation fault. There is no trivial method to distinguish the final partition from other partitions because, by design,
 * partitions should be treated equally.
 *
 * Every time point that is equal or larger to the desiredPartitionPoint should be included in that partition. This logic here is the same
 * as the event times.
 *
 * The last time of desiredPartitionPoints is filled manually. There is no round-off error involved. So it is safe to use == for
 * floating-point numbers. The last time point is naturally included by using std::lower_bound.
 *
 * @param [in] timeTrajectory: time trajectory that will be divided
 * @param [in] numWorkers: number of worker i.e. number of partitions
 * @return array of index pairs indicating the start and end of each partition
 */
std::vector<std::pair<int, int>> computePartitionIntervals(const scalar_array_t& timeTrajectory, int numWorkers);

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
