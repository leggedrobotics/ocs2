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

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {

/**
 * This class is an interface class for constructing LQ approximation of the continous time optimal control problem.
 */
class LinearQuadraticApproximator {
 public:
  /**
   * Constructor
   * @note This class does not take ownership of any of the parameters.
   *       Make sure that the parameter lifetime is longer than this class instance.
   */
  explicit LinearQuadraticApproximator(const OptimalControlProblem& problem, bool checkNumericalCharacteristics = true)
      : problemPtr_(&problem), checkNumericalCharacteristics_(checkNumericalCharacteristics) {}

  /**
   * Calculates an LQ approximate of the constrained optimal control problem at a given time, state, and input.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input.
   * @param [out] modelData: The output data model.
   */
  void approximateLQProblem(const scalar_t& time, const vector_t& state, const vector_t& input, ModelData& modelData) const;

  /**
   * Calculates an LQ approximate of the constrained optimal control problem at a jump event time.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [out] modelData: The output data model.
   */
  void approximateLQProblemAtEventTime(const scalar_t& time, const vector_t& state, ModelData& modelData) const;

  /**
   * Calculates an LQ approximate of the constrained optimal control problem at final time.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [out] modelData: The output data model.
   */
  void approximateLQProblemAtFinalTime(const scalar_t& time, const vector_t& state, ModelData& modelData) const;

 private:
  void approximateDynamics(const scalar_t& time, const vector_t& state, const vector_t& input, ModelData& modelData) const;
  void approximateConstraints(const scalar_t& time, const vector_t& state, const vector_t& input, ModelData& modelData) const;
  void approximateCost(const scalar_t& time, const vector_t& state, const vector_t& input, ModelData& modelData) const;

  const OptimalControlProblem* problemPtr_;
  bool checkNumericalCharacteristics_;
};

/**
 * Compute the total intermediate cost (i.e. cost + softConstraints). It is assumed that the precomputation request is already made.
 */
scalar_t computeCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state, const vector_t& input);

/**
 * Compute the quadratic approximation of the total intermediate cost (i.e. cost + softConstraints). It is assumed that the precomputation
 * request is already made.
 */
ScalarFunctionQuadraticApproximation approximateCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                                                     const vector_t& input);

/**
 * Compute the total preJump cost (i.e. cost + softConstraints). It is assumed that the precomputation request is already made.
 */
scalar_t computeEventCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state);

/**
 * Compute the quadratic approximation of the total preJump cost (i.e. cost + softConstraints). It is assumed that the precomputation
 * request is already made.
 */
ScalarFunctionQuadraticApproximation approximateEventCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state);

/**
 * Compute the total final cost (i.e. cost + softConstraints). It is assumed that the precomputation request is already made.
 */
scalar_t computeFinalCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state);

/**
 * Compute the quadratic approximation of the total final cost (i.e. cost + softConstraints). It is assumed that the precomputation
 * request is already made.
 */
ScalarFunctionQuadraticApproximation approximateFinalCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state);

}  // namespace ocs2
