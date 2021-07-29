/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

namespace ocs2 {

/**
 * Applies the following change of input variables to the quadraticApproximation (stateDim=n, inputDim=m):
 * \delta u = Pu * \tilde{\delta u} + Px * \delta x + \delta u_0,
 * with sizes Pu (m x p), Px (m x n), u0 (m x 1)
 *
 * The altered model data will be of size stateDim=n, inputDim=p
 *
 * A Px / u0 of zeros can be efficiently applied by passing an empty matrix / vector (of size 0).
 *
 * @param quadraticApproximation : Approximation to be adapted in-place
 * @param Pu : Matrix defining the range of \tilde{\delta u}
 * @param Px : Matrix defining the range of \delta x
 * @param u0 : Input offset
 */
void changeOfInputVariables(ScalarFunctionQuadraticApproximation& quadraticApproximation, const matrix_t& Pu,
                            const matrix_t& Px = matrix_t(), const vector_t& u0 = vector_t());

/** Applies the change of input variables to a linear system */
void changeOfInputVariables(VectorFunctionLinearApproximation& linearApproximation, const matrix_t& Pu, const matrix_t& Px = matrix_t(),
                            const vector_t& u0 = vector_t());

}  // namespace ocs2