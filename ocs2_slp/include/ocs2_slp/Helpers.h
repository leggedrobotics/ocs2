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
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_oc/oc_problem/OcpSize.h>

namespace ocs2 {
namespace slp {

/**
 * Computes the upper bound of eigenvalues for the total cost hessian matrix. The bound is computed based on the Gershgorin Circle Theorem.
 *
 * As the hessian matrix is a real symmetric matrix, all eigenvalues are real. Therefore, instead of estimating the
 * upper bound on a complex plane, we can estimate the bound along a 1D real number axis. The estimated upper bound is
 * defined as max_{i} [H_{ii} + R_{i}], where R_{i} is the radius of i'th Gershgorin discs. The Gershgorin discs radius is
 * defined ad R_{i} = sum_{j \neq i} |H_{ij}|. Thus the upper bound of eigenvalues yields from max_{i}sum{j} |H_{ij}|.
 *
 * Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * totalCost = 0.5 z' H z + z' h + h0
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
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] cost: Quadratic approximation of the cost over the time horizon.
 * @return: The upper bound of eigenvalues for H.
 */
scalar_t hessianEigenvaluesUpperBound(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost);

/**
 * Computes the upper bound of eigenvalues for the matrix G G' ( in parallel). The bound is computed based on the Gershgorin Circle Theorem.
 *
 * As the G G' matrix is a real symmetric matrix, all eigenvalues are real. Therefore, instead of estimating the
 * upper bound on a complex plane, we can estimate the bound along a 1D real number axis. The estimated upper bound is
 * defined as max_{i} [GG'_{ii} + R_{i}], where R_{i} is the radius of i'th Gershgorin discs. The Gershgorin discs radius is
 * defined ad R_{i} = sum_{j \neq i} |GG'_{ij}|. Thus the upper bound of eigenvalues yields from max_{i}sum{j} |GG'_{ij}|.
 *
 * Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G z = g
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
 * @param [in] threadPool: The thread pool.
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param [in] constraints: Linear approximation of the constraints over the time horizon. Pass nullptr if there is no constraints.
 * @param [in] scalingVectorsPtr: Vector representation for the identity parts of the dynamics inside the constraint matrix. After scaling,
 *                                they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @return: The upper bound of eigenvalues for G G'.
 */
scalar_t GGTEigenvaluesUpperBound(ThreadPool& threadPool, const OcpSize& ocpSize,
                                  const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                  const std::vector<VectorFunctionLinearApproximation>* constraintsPtr,
                                  const vector_array_t* scalingVectorsPtr);

/**
 * Computes the row-wise absolute sum of the cost hessian matrix, H. Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * totalCost = 0.5 z' H z + z' h + h0
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
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] cost: Quadratic approximation of the cost over the time horizon.
 * @return: The absolute sum of rows of matrix H.
 */
vector_t hessianAbsRowSum(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost);

/**
 * Computes the row-wise absolute sum of matrix G G' in parallel. Also refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * z = [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * G z = g
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
 * @param [in] threadPool: The thread pool.
 * @param [in] ocpSize: The size of optimal control problem.
 * @param [in] dynamics: Linear approximation of the dynamics over the time horizon.
 * @param [in] constraints: Linear approximation of the constraints over the time horizon. Pass nullptr if there is no constraints.
 * @param [in] scalingVectorsPtr: Vector representation for the identity parts of the dynamics inside the constraint matrix. After scaling,
 *                                they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @return The absolute sum of rows of matrix G G'.
 */
vector_t GGTAbsRowSumInParallel(ThreadPool& threadPool, const OcpSize& ocpSize,
                                const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<VectorFunctionLinearApproximation>* constraintsPtr,
                                const vector_array_t* scalingVectorsPtr);

}  // namespace slp
}  // namespace ocs2