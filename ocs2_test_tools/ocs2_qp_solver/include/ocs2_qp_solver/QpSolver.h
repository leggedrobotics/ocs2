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

#include "ocs2_qp_solver/QpSolverTypes.h"
#include "ocs2_qp_solver/QpTrajectories.h"

namespace ocs2 {
namespace qp_solver {

/**
 * Solves the discretized linear quadratic optimal control problem by constructing a dense QP and inverting the full KKT system.
 * The decision vector is defined as w = [dx[0], du[0], dx[1],  du[1], ..., dx[N]]
 *
 * @param lqApproximation : vector of stage-wise discrete quadratic cost and linear dynamics
 * @param dx0 : initial state deviation from the nominal trajectories.
 * @return trajectory of state and inputs (in relative coordinates), .i.e. dx(t), du(t)
 */
std::pair<vector_array_t, vector_array_t> solveLinearQuadraticProblem(const std::vector<LinearQuadraticStage>& lqApproximation,
                                                                      const vector_t& dx0);

/**
 * Constructs the matrix of stacked dynamic constraints A w + b = 0
 *
 * A = [-I  *
 *       C  D  *  *
 *       A  B -I  *
 *       *  *  C  D  *  *
 *       *  *  A  B -I  *
 *       *  *  *  *  C  D  *
 *       *  *  *  *  A  B -I ]
 *       *  *  *  *  *  *  C ]
 *
 * b = [x0; e[0]; b[0]; ... e[N-1]; b[N-1]; e[N]]
 *
 * @param lqp : linear quadratic problem.
 * @param dx0 : initial state deviation from the nominal trajectories.
 * @param numConstraints : number of rows in A
 * @param numDecisionVariables : size of w
 * @return linear constraints in w, where w is the vector of decision variables
 */
VectorFunctionLinearApproximation getConstraintMatrices(const std::vector<LinearQuadraticStage>& lqp, const vector_t& dx0,
                                                        int numConstraints, int numDecisionVariables);

/**
 * Constructs a matrix of stacked cost functions  1/2 w' H w + g' w
 *
 * H = [ Q  P'
 *       P  R
 *             Q  P'
 *             P  R
 *                   Qf ]
 *
 * g = [q[0]; r[0]; q[1]; r[1]; ... qf ]
 *
 * @param lqp
 * @param numDecisionVariables : size of w
 * @return quadratic cost function in w, where w is the vector of decision variables
 */
ScalarFunctionQuadraticApproximation getCostMatrices(const std::vector<LinearQuadraticStage>& lqp, int numDecisionVariables);

/**
 * Constructs the equality constrained QP from the discretized linear quadratic control problem.
 * @param lqp : linear quadratic problem.
 * @param dx0 : initial state deviation from the nominal trajectories.
 * @return { cost matrices, constraint matrices }
 */
std::pair<ScalarFunctionQuadraticApproximation, VectorFunctionLinearApproximation> getDenseQp(const std::vector<LinearQuadraticStage>& lqp,
                                                                                              const vector_t& dx0);

/**
 * Solves the equality constrained QP
 * min_w  1/2 w' H w + g' w
 *   s.t. A w + b = 0
 *
 *   Assumes H is positive definite, rows of A are linearly independent.
 *
 * @return {w, lambda} at the solution, where w is the vector of decision variables, and lambda is the vector of lagrange multipliers
 */
std::pair<vector_t, vector_t> solveDenseQp(const ScalarFunctionQuadraticApproximation& cost,
                                           const VectorFunctionLinearApproximation& constraints);

/**
 * Reconstructs the optimal state and input trajectory recursively based on the full qp solution vector
 * @param numStates : number of states per stage
 * @param numInputs : number of inputs per stage
 * @param w : the vector of decision variables
 * @return { state_trajectory, input_trajectory }
 */
std::pair<vector_array_t, vector_array_t> getStateAndInputTrajectory(const std::vector<int>& numStates, const std::vector<int>& numInputs,
                                                                     const vector_t& w);

}  // namespace qp_solver
}  // namespace ocs2
