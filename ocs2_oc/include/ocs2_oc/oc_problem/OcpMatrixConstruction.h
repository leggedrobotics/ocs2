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
 * @brief Get liner constraint matrix G from the dynamics, cost and constraint arrays.
 *
 * @param ocpSize: The size of optimal control problem.
 * @param x0: Initial state.
 * @param dynamics: Dynamics array.
 * @param constraints: Constraints array.
 * @param scalingVectorsPtr: Vector representatoin for the identity parts of the dynamics constraints inside the constraint matrix. After
 * scaling, they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param res: The resulting constraints approxmation.
 */
void getConstraintMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                         const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                         VectorFunctionLinearApproximation& res);

void getConstraintMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                               const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                               Eigen::SparseMatrix<scalar_t>& G, vector_t& g);

void getCostMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   ScalarFunctionQuadraticApproximation& res);

void getCostMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                         Eigen::SparseMatrix<scalar_t>& H, vector_t& h);
}  // namespace ocs2