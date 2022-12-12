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

#include <Eigen/Sparse>

#include <ocs2_core/Types.h>

#include "ocs2_oc/oc_problem/OcpSize.h"

namespace ocs2 {

/**
 * Constructs concatenated linear approximation of the constraints from the dynamics and constraints arrays w.r.t.
 * Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G Z = g
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 *
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * @param[in] ocpSize: The size of optimal control problem.
 * @param[in] x0: The initial state.
 * @param[in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param[in] constraints: Linear approximation of the constraints over the time horizon. Pass nullptr if there is no constraints.
 * @param[in] scalingVectorsPtr: Vector representatoin for the identity parts of the dynamics inside the constraint matrix. After scaling,
 *                               they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param[out] res: The resulting constraints approxmation. Here we misused dfdx field to store the jacobian w.r.t. Z.
 */
void getConstraintMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                         const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                         VectorFunctionLinearApproximation& res);

/**
 * Constructs concatenated linear approximation of the constraints from the dynamics and constraints arrays w.r.t.
 * Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G Z = g
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 *
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * @param[in] ocpSize: The size of optimal control problem.
 * @param[in] x0: The initial state.
 * @param[in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param[in] constraints: Linear approximation of the constraints over the time horizon. Pass nullptr if there is no constraints.
 * @param[in] scalingVectorsPtr: Vector representatoin for the identity parts of the dynamics inside the constraint matrix. After scaling,
 *                               they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param[out] G: The jacobian of the concatenated constraints w.r.t. Z.
 * @param[out] g: The concatenated constraints value.
 */
void getConstraintMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                               const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                               Eigen::SparseMatrix<scalar_t>& G, vector_t& g);

/**
 * Constructs concatenated quadratic approximation of the total cost w.r.t. Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * totalCost = 0.5 Z' H Z + Z' h + h0
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 *
 * h = [(P0 x0 + r0); q1; r1 ...; qn; rn; q_{n+1}]
 *
 * @param[in] ocpSize: The size of optimal control problem.
 * @param[in] x0: The initial state.
 * @param[in] cost: Quadratic approximation of the cost over the time horizon.
 * @param[out] res: The resulting cost approxmation. Here we misused dfdx and dfdxx fields to store the jacobian and the hessian
 *                  matrices w.r.t. Z.
 */
void getCostMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   ScalarFunctionQuadraticApproximation& res);

/**
 * Constructs concatenated the jacobian and the hessian matrices of the total cost w.r.t. Z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * totalCost = 0.5 Z' H Z + Z' h + h0
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 *
 * h = [(P0 x0 + r0); q1; r1 ...; qn; rn; q_{n+1}]
 *
 * @param[in] ocpSize: The size of optimal control problem.
 * @param[in] x0: The initial state.
 * @param[in] cost: Quadratic approximation of the cost over the time horizon.
 * @param[out] H: The concatenated hessian matrix w.r.t. Z.
 * @param[out] h: The concatenated jacobian vector w.r.t. Z.
 */
void getCostMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                         Eigen::SparseMatrix<scalar_t>& H, vector_t& h);

/**
 * Deserializes the stacked solution to state-input trajecotries. Note that the initial state is not part of the stacked solution.
 *
 * @param [in] ocpSize : Optimal control problem sizes.
 * @param [in] stackedSolution : Defined as [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 * @param [in] x0 : The initial state.
 * @param [out] xTrajectory : State tarjectory.
 * @param [out] uTrajectory : Input trajecotry.
 */
void toOcpSolution(const OcpSize& ocpSize, const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory,
                   vector_array_t& uTrajectory);

/**
 * Serializes the state-input trajecotries. Note that the initial state is not considered as a decision variable.
 *
 * @param [in] xTrajectory : State tarjectory.
 * @param [in] UTrajectory : Input trajecotry.
 * @param [out] stackedSolution : [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 */
void toKktSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution);

}  // namespace ocs2