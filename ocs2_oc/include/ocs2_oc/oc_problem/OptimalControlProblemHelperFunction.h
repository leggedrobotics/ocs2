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

#include "ocs2_oc/oc_data/DualSolution.h"
#include "ocs2_oc/oc_data/PrimalSolution.h"
#include "ocs2_oc/oc_data/ProblemMetrics.h"
#include "ocs2_oc/oc_problem/OptimalControlProblem.h"

namespace ocs2 {

/**
 * Initializes the dual solution based on the cached dual solution. It will use interpolation if cachedDualSolution has any component
 * in the same mode otherwise it will use the Lagrangian initialization method of ocp.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] primalSolution : The primal solution.
 * @param [in] cachedDualSolution : The cached dual solution which will be used for interpolation.
 * @param [out] dualSolution : The initialized dual solution.
 */
void initializeDualSolution(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, const DualSolution& cachedDualSolution,
                            DualSolution& dualSolution);

/**
 * Initializes final MultiplierCollection for equality and inequality Lagrangians.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Final time.
 * @param [out] multiplierCollection : The initialized final MultiplierCollection.
 */
void initializeFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection);

/**
 * Initializes pre-jump MultiplierCollection for equality and inequality Lagrangians.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Pre-jump time.
 * @param [out] multiplierCollection : The initialized pre-jump MultiplierCollection.
 */
void initializePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection);

/**
 * Initializes intermediate MultiplierCollection for equality and inequality Lagrangians.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Intermediate time.
 * @param [out] multiplierCollection : The initialized intermediate MultiplierCollection.
 */
void initializeIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time,
                                                MultiplierCollection& multiplierCollection);

/**
 * Updates in-place the dual solution based on its current solution and state-input values using the Lagrangian update method in ocp.
 * Moreover it also updates the penalties of ProblemMetrics based on the update of dual solution.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] primalSolution : The primal solution.
 * @param [in, out] problemMetrics : The problem metric. Its penalties will be updated based on the update of dualSolution.
 * @param [out] dualSolution : The updated dual solution.
 */
void updateDualSolution(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, ProblemMetrics& problemMetrics,
                        DualSolutionRef dualSolution);

/**
 * Updates in-place final MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of MetricsCollection based on the update of multipliers.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Final time.
 * @param [in] state : Final state.
 * @param [in, out] metricsCollection: The final MetricsCollection. Its penalties will be updated based on
 *                                     the update of multiplierCollection.
 * @param [out] multiplierCollection : The updated final MultiplierCollection.
 */
void updateFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                     MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection);

/**
 * Updates in-place pre-jump MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of MetricsCollection based on the update of multipliers.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Pre-jump time.
 * @param [in] state : Pre-jump state.
 * @param [in, out] metricsCollection: The pre-jump MetricsCollection. Its penalties will be updated based on
 *                                     the update of multiplierCollection.
 * @param [out] multiplierCollection : The updated pre-jump MultiplierCollection.
 */
void updatePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                       MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection);

/**
 * Updates in-place intermediate MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of MetricsCollection based on the update of multipliers.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Intermediate time.
 * @param [in] state : Intermediate state.
 * @param [in] input : Intermediate input.
 * @param [in, out] metricsCollection: The intermediate MetricsCollection. Its penalties will be updated based on
 *                                     the update of multiplierCollection.
 * @param [out] multiplierCollection : The updated Intermediate MultiplierCollection.
 */
void updateIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, const vector_t& input,
                                            MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection);

/**
 * Extracts a requested final Term LagrangianMetrics from the input MetricsCollection.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metricsColl : MetricsCollection.
 * @param [out] metrics : A const pointer to the term LagrangianMetrics.
 * @return True if the term found in the final collection.
 */
const LagrangianMetrics* extractFinalTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                                 const MetricsCollection& metricsColl);

/**
 * Extracts a requested pre-jump Term LagrangianMetrics array from the input MetricsCollection array.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metricsCollArray : MetricsCollection array.
 * @param [out] metricsArray : Array of const references to term LagrangianMetrics.
 * @return True if the term found in the pre-jump collection.
 */
bool extractPreJumpTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                               const std::vector<MetricsCollection>& metricsCollArray,
                               std::vector<LagrangianMetricsConstRef>& metricsArray);

/**
 * Extracts a requested intermediate Term LagrangianMetrics trajectory from the input MetricsCollection trajectory.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metricsCollTraj : MetricsCollection trajectory.
 * @param [out] metricsTrajectory : Trajectory of const references to term LagrangianMetrics.
 * @return True if the term found in the intermediate collection.
 */
bool extractIntermediateTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                    const std::vector<MetricsCollection>& metricsCollTraj,
                                    std::vector<LagrangianMetricsConstRef>& metricsTrajectory);

/**
 * Extracts a requested final Term Multiplier from the input MultiplierCollection.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] multiplierColl : MultiplierCollection.
 * @param [out] multiplier : A const reference to the term Multiplier.
 * @return True if the term found in the final collection.
 */
const Multiplier* extractFinalTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                             const MultiplierCollection& multiplierColl);

/**
 * Extracts a requested pre-jump Term Multiplier array from the input MultiplierCollection array.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] multiplierCollArray : MultiplierCollection array.
 * @param [out] multiplierArray : Array of const references to term Multiplier.
 * @return True if the term found in the pre-jump collection.
 */
bool extractPreJumpTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                  const std::vector<MultiplierCollection>& multiplierCollArray,
                                  std::vector<MultiplierConstRef>& multiplierArray);

/**
 * Extracts a requested intermediate Term Multiplier trajectory from the input MultiplierCollection trajectory.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] multiplierCollTraj : MultiplierCollection trajectory.
 * @param [out] multiplierTrajectory : Trajectory of const references to term Multiplier.
 * @return True if the term found in the intermediate collection.
 */
bool extractIntermediateTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                       const std::vector<MultiplierCollection>& multiplierCollTraj,
                                       std::vector<MultiplierConstRef>& multiplierTrajectory);

}  // namespace ocs2
