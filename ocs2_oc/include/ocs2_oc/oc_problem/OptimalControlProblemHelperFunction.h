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

#include <functional>

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
 * Moreover it also updates the penalties of Metrics based on the update of multipliers.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Final time.
 * @param [in] state : Final state.
 * @param [in, out] metrics: The final Metrics. Its penalties will be updated based on the update of multiplierCollection.
 * @param [out] multipliers : The updated final MultiplierCollection.
 */
void updateFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, Metrics& metrics,
                                     MultiplierCollection& multipliers);

/**
 * Updates in-place pre-jump MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of Metrics based on the update of multipliers.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Pre-jump time.
 * @param [in] state : Pre-jump state.
 * @param [in, out] metrics: The pre-jump Metrics. Its penalties will be updated based on the update of multiplierCollection.
 * @param [out] multipliers : The updated pre-jump MultiplierCollection.
 */
void updatePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, Metrics& metrics,
                                       MultiplierCollection& multipliers);

/**
 * Updates in-place intermediate MultiplierCollection for equality and inequality Lagrangians.
 * Moreover it also updates the penalties of Metrics based on the update of multipliers.
 *
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] time : Intermediate time.
 * @param [in] state : Intermediate state.
 * @param [in] input : Intermediate input.
 * @param [in, out] metrics: The intermediate Metrics. Its penalties will be updated based on the update of multiplierCollection.
 * @param [out] multipliers : The updated Intermediate MultiplierCollection.
 */
void updateIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, const vector_t& input,
                                            Metrics& metrics, MultiplierCollection& multipliers);

/**
 * Extracts a requested final constraint term vector from the input Metrics.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metrics : Metrics.
 * @return A const pointer to the constraint term value.
 */
const vector_t* extractFinalTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const Metrics& metrics);

/**
 * Extracts a requested final term LagrangianMetrics from the input Metrics.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metrics : Metrics.
 * @return A const pointer to the term LagrangianMetrics.
 */
const LagrangianMetrics* extractFinalTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                                           const Metrics& metrics);

/**
 * Extracts an array of the requested pre-jump constraint term from the input Metrics array.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metricsArray : Metrics array.
 * @param [out] constraintArray : Array of const references to constraint term values.
 * @return True if the term found in the pre-jump collection.
 */
bool extractPreJumpTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const std::vector<Metrics>& metricsArray,
                                  std::vector<std::reference_wrapper<const vector_t>>& constraintArray);

/**
 * Extracts a requested pre-jump term LagrangianMetrics array from the input Metrics array.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metricsArray : Metrics array.
 * @param [out] lagrangianMetricsArray : Array of const references to term LagrangianMetrics.
 * @return True if the term found in the pre-jump collection.
 */
bool extractPreJumpTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                         const std::vector<Metrics>& metricsArray,
                                         std::vector<LagrangianMetricsConstRef>& lagrangianMetricsArray);

/**
 * Extracts a trajectory of the requested intermediate constraint term from the input Metrics trajectory.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metricsTraj : Metrics trajectory.
 * @param [out] constraintTraj : Trajectory of const references to constraint term values.
 * @return True if the term found in the intermediate collection.
 */
bool extractIntermediateTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const std::vector<Metrics>& metricsTraj,
                                       std::vector<std::reference_wrapper<const vector_t>>& constraintTraj);

/**
 * Extracts a requested intermediate term LagrangianMetrics trajectory from the input Metrics trajectory.
 * @param [in] ocp : A const reference to the optimal control problem.
 * @param [in] name : The name of the term.
 * @param [in] metricsTraj : Metrics trajectory.
 * @param [out] lagrangianMetricsTraj : Trajectory of const references to term LagrangianMetrics.
 * @return True if the term found in the intermediate collection.
 */
bool extractIntermediateTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                              const std::vector<Metrics>& metricsTraj,
                                              std::vector<LagrangianMetricsConstRef>& lagrangianMetricsTraj);

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
