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

#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_core/model_data/Multiplier.h>

#include "ocs2_oc/oc_problem/OptimalControlProblem.h"

namespace ocs2 {

/** Initializes final MultiplierCollection for equality and inequality Lagrangians. */
void initializeFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection);

/** Initializes pre-jump MultiplierCollection for equality and inequality Lagrangians. */
void initializePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection);

/** Initializes intermediate MultiplierCollection for equality and inequality Lagrangians. */
void initializeIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time,
                                                MultiplierCollection& multiplierCollection);

/**
 * Updates final MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of MetricsCollection based on the update multipliers.
 */
void updateFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                     MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection);

/**
 * Updates pre-jump MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of MetricsCollection based on the update multipliers.
 */
void updatePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                       MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection);

/**
 * Updates intermediate MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of MetricsCollection based on the update multipliers.
 */
void updateIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, const vector_t& input,
                                            MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection);

}  // namespace ocs2
