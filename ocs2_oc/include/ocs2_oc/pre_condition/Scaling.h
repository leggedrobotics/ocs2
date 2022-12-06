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

#include <Eigen/Sparse>

#include <ocs2_core/Types.h>
#include <ocs2_core/thread_support/ThreadPool.h>

#include "ocs2_oc/oc_problem/OcpSize.h"

namespace ocs2 {

/**
 * Calculates the scaling factor D, E and c, and scale the input dynamics, cost data in place in parallel.
 *
 * There are a few pre-conditioning methods aiming to shape different aspect of the problem. To balance the performance and
 * computational effort, we choose a modified Ruzi equilibration summarized in Algorithm 2. Interested readers can find the
 * original Ruiz equilibration in: "Ruiz, D., 2001. A scaling algorithm to equilibrate both rows and columns norms in matrices"
 *
 *
 *
 * @param[in] : threadPool External thread pool.
 * @param[in] x0 : Initial state
 * @param[in] ocpSize : The size of the oc problem.
 * @param[in] iteration : Number of iteration.
 * @param[in, out] : dynamics The dynamics array of all time points.
 * @param[in, out] : cost The cost array of all time points.
 * @param[out] : DOut Scaling factor D
 * @param[out] : EOut Scaling factor E
 * @param[out] scalingVectors : Vector representatoin for the identity parts of the dynamics constraints inside the constraint matrix.
 *                              After scaling, they become arbitrary diagonal matrices. scalingVectors store the diagonal components
 *                              of this type of matrix for every timestamp.
 * @param[out] cOut : Scaling factor c
 */
void preConditioningInPlaceInParallel(ThreadPool& threadPool, const vector_t& x0, const OcpSize& ocpSize, const int iteration,
                                      std::vector<VectorFunctionLinearApproximation>& dynamics,
                                      std::vector<ScalarFunctionQuadraticApproximation>& cost, vector_array_t& DOut, vector_array_t& EOut,
                                      vector_array_t& scalingVectors, scalar_t& cOut);

}  // namespace ocs2