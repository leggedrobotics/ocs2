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

#include <functional>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/TrapezoidalIntegration.h>

namespace ocs2 {
namespace PerformanceIndicesRollout {

template <int STATE_DIM, int INPUT_DIM>
using cost_wraper_t =
    std::function<scalar_t(scalar_t, const Eigen::Matrix<scalar_t, STATE_DIM, 1>&, const Eigen::Matrix<scalar_t, INPUT_DIM, 1>&)>;

template <int STATE_DIM, int INPUT_DIM>
using constraints_wraper_t =
    std::function<dynamic_vector_t(scalar_t, const Eigen::Matrix<scalar_t, STATE_DIM, 1>&, const Eigen::Matrix<scalar_t, INPUT_DIM, 1>&)>;

template <int STATE_DIM, int INPUT_DIM>
using state_vector_array_t = typename Dimensions<STATE_DIM, INPUT_DIM>::state_vector_array_t;
template <int STATE_DIM, int INPUT_DIM>
using input_vector_array_t = typename Dimensions<STATE_DIM, INPUT_DIM>::input_vector_array_t;

/**
 * Computes the accumulated cost over the given trajectory.
 */
template <int STATE_DIM, int INPUT_DIM>
scalar_t rolloutCost(cost_wraper_t<STATE_DIM, INPUT_DIM> costWraper, const scalar_array_t& timeTrajectory,
                     const state_vector_array_t<STATE_DIM, INPUT_DIM>& stateTrajectory,
                     const input_vector_array_t<STATE_DIM, INPUT_DIM>& inputTrajectory) {
  const size_t N = timeTrajectory.size();
  assert(stateTrajectory.size() == N);
  assert(inputTrajectory.size() == N);

  scalar_array_t costTrajectory;
  costTrajectory.reserve(N);
  for (size_t i = 0; i < N; i++) {
    costTrajectory.push_back(costWraper(timeTrajectory[i], stateTrajectory[i], inputTrajectory[i]));
  }

  return trapezoidalIntegration(timeTrajectory, costTrajectory);
}

/**
 * Computes the ISE (Integral of Square Error) of constraint over the given trajectory.
 */
template <int STATE_DIM, int INPUT_DIM>
scalar_t rolloutConstraint(constraints_wraper_t<STATE_DIM, INPUT_DIM> constraintWraper, const scalar_array_t& timeTrajectory,
                           const state_vector_array_t<STATE_DIM, INPUT_DIM>& stateTrajectory,
                           const input_vector_array_t<STATE_DIM, INPUT_DIM>& inputTrajectory) {
  const size_t N = timeTrajectory.size();
  assert(stateTrajectory.size() == N);
  assert(inputTrajectory.size() == N);

  scalar_array_t constraintTrajectoryISE;
  constraintTrajectoryISE.reserve(N);
  for (size_t i = 0; i < N; i++) {
    const auto constraint = constraintWraper(constraintWraper(timeTrajectory[i], stateTrajectory[i], inputTrajectory[i]));
    constraintTrajectoryISE.push_back(constraint.squaredNorm());
  }

  return trapezoidalIntegration(timeTrajectory, constraintTrajectoryISE);
}

}  // namespace PerformanceIndicesRollout
}  // namespace ocs2
