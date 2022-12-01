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
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include "ocs2_oc/oc_data/PrimalSolution.h"
#include "ocs2_oc/oc_data/TimeDiscretization.h"

namespace ocs2 {
namespace multiple_shooting {

/**
 * Uses the system initializer to determine a state-input initialization at a intermediate node
 *
 * @param initializer : System initializer
 * @param t :  Start of the discrete interval
 * @param tNext : End time of te discrete interval
 * @param x : Starting state of the discrete interval
 * @return {u(t), x(tNext)} : input and state transition
 */
inline std::pair<vector_t, vector_t> initializeIntermediateNode(Initializer& initializer, scalar_t t, scalar_t tNext, const vector_t& x) {
  vector_t input, nextState;
  initializer.compute(t, x, tNext, input, nextState);
  return {input, nextState};
}

/**
 * Interpolate a primal solution for state-input initialization at a intermediate node
 *
 * @param primalSolution : previous solution
 * @param t :  Start of the discrete interval
 * @param tNext : End time of te discrete interval
 * @return {u(t), x(tNext)} : input and state transition
 */
inline std::pair<vector_t, vector_t> initializeIntermediateNode(const PrimalSolution& primalSolution, scalar_t t, scalar_t tNext) {
  return {LinearInterpolation::interpolate(t, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_),
          LinearInterpolation::interpolate(tNext, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_)};
}

/**
 * Initialize the state jump at an event node.
 *
 * @param t : Time at the event node
 * @param x : Pre-event state
 * @return x_next : Post-event state
 */
inline vector_t initializeEventNode(scalar_t t, const vector_t& x) {
  // Assume identity map for now
  return x;
}

/**
 * Initializes for the state-input trajectories. It interpolates the primalSolution for the starting intersecting time period and then uses
 * initializer for the tail.
 *
 * @param [in] initState :  Initial state
 * @param [in] timeDiscretization : The annotated time trajectory
 * @param [in] primalSolution : previous solution
 * @param [in] initializer : System initializer
 * @param [out] stateTrajectory : The initialized state trajectory
 * @param [out] inputTrajectory : The initialized input trajectory
 */
void initializeStateInputTrajectories(const vector_t& initState, const std::vector<AnnotatedTime>& timeDiscretization,
                                      const PrimalSolution& primalSolution, Initializer& initializer, vector_array_t& stateTrajectory,
                                      vector_array_t& inputTrajectory);

}  // namespace multiple_shooting
}  // namespace ocs2
