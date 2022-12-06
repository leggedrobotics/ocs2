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

#include "ocs2_oc/oc_problem/OcpSize.h"

namespace ocs2 {

/**
 * Calculate the scaling factor D, E and c, and scale the input matrix(vector) H, h and G in place. The scaled matrix(vector) are defined as
 * \f[
 *      \tilde{H} = cDHD, \tilde{h} = cDh,
 *      \tilde{G} = EGD,  \tilde{g} = Eg
 * \f]
 *
 *
 * @param[in, out] H Cost hessian.
 * @param[in, out] h Linear cost term.
 * @param[in, out] G Linear constraint matrix.
 * @param[in] iteration Number of iteration.
 * @param[out] DOut Scaling factor D
 * @param[out] EOut Scaling factor E
 * @param[out] cOut Scaling factor c
 */
void preConditioningSparseMatrixInPlace(Eigen::SparseMatrix<scalar_t>& H, vector_t& h, Eigen::SparseMatrix<scalar_t>& G,
                                        const int iteration, vector_t& DOut, vector_t& EOut, scalar_t& cOut);

/**
 * @brief Scale the dynamics and cost array in place and construct scaling vector array from the given scaling factors E, D and c.
 *
 * @param[in] ocpSize The size of the oc problem.
 * @param[in] D Scaling factor D
 * @param[in] E Scaling factor E
 * @param[in] c Scaling factor c
 * @param[in, out] dynamics Dynamics array
 * @param[in, out] cost Cost array
 * @param[out] scalingVectors Vector representatoin for the identity parts of the dynamics constraints inside the constraint matrix.
 * After scaling, they become arbitrary diagonal matrices. scalingVectors store the diagonal components of this type of matrix for every
 * timestamp.
 */
void scaleDataInPlace(const OcpSize& ocpSize, const vector_t& D, const vector_t& E, const scalar_t c,
                      std::vector<VectorFunctionLinearApproximation>& dynamics, std::vector<ScalarFunctionQuadraticApproximation>& cost,
                      std::vector<vector_t>& scalingVectors);

}  // namespace ocs2