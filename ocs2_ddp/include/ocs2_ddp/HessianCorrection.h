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

#include <string>

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {
namespace hessian_correction {

/**
 * @brief The Hessian matrix correction strategy
 * Enum used in selecting either DIAGONAL_SHIFT, CHOLESKY_MODIFICATION, EIGENVALUE_MODIFICATION, or GERSHGORIN_MODIFICATION strategies.
 */
enum class Strategy { DIAGONAL_SHIFT, CHOLESKY_MODIFICATION, EIGENVALUE_MODIFICATION, GERSHGORIN_MODIFICATION };

/**
 * Get string name of Hessian_Correction type
 * @param [in] strategy: Hessian_Correction type enum
 */
std::string toString(Strategy strategy);

/**
 * Get Hessian_Correction type from string name, useful for reading config file
 * @param [in] name: Hessian_Correction name
 */
Strategy fromString(const std::string& name);

/**
 * Shifts the Hessian based on the strategy defined by Line_Search::hessianCorrectionStrategy_.
 *
 * @param [in] strategy: Hessian matrix correction strategy.
 * @param [in, out] matrix: The Hessian matrix.
 * @param [in] minEigenvalue: The minimum expected eigenvalue after correction.
 */
void shiftHessian(Strategy strategy, matrix_t& matrix, scalar_t minEigenvalue = numeric_traits::limitEpsilon<scalar_t>());

}  // namespace hessian_correction
}  // namespace ocs2
