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

#include <algorithm>
#include <functional>  // missing header in cg.hpp
#include <vector>

#include <cppad/cg.hpp>

namespace ocs2 {

namespace cppad_sparsity {

/**
 *  Vector contains 1 set per row, each set contains the indices of non-zero columns.
 */
using SparsityPattern = std::vector<std::set<size_t>>;

/**
 * Gets the Jacobian sparsity pattern of a taped CppAD function.
 * @tparam ad_fun_t : CppAD function type.
 * @param fun : function that has been taped already.
 * @return Sparsity pattern.
 */
template <typename ad_fun_t>
SparsityPattern getJacobianSparsityPattern(ad_fun_t& fun) {
  return CppAD::cg::jacobianSparsitySet<SparsityPattern>(fun);
}

/**
 * Gets the Hessian sparsity pattern of a taped CppAD function.
 * @tparam ad_fun_t : CppAD function type.
 * @param fun : function that has been taped already.
 * @return Sparsity pattern.
 */
template <typename ad_fun_t>
SparsityPattern getHessianSparsityPattern(ad_fun_t& fun) {
  return CppAD::cg::hessianSparsitySet<SparsityPattern>(fun);
}

/**
 * Returns a sparsity pattern that has entries only there where both p0 and p1 have entries.
 * @param p0 : First sparsity pattern.
 * @param p1 : Second sparsity pattern.
 * @return Intersected sparsity pattern.
 */
SparsityPattern getIntersection(const SparsityPattern& p0, const SparsityPattern& p1);

/**
 * Constructs a Jacobian sparsity pattern that has dense entries for the first variableDim columns of rangeDim rows.
 * @param rangeDim : number of rows to generate dense sparsity for.
 * @param variableDim : number of cols to generate dense sparsity for.
 * @return sparsity pattern.
 */
SparsityPattern getJacobianVariableSparsity(int rangeDim, int variableDim);

/**
 * Constructs a upper triangular Hessian sparsity pattern that has dense entries for the variableDim x variableDim in the top left corner.
 * @param variableDim : size of the dense uppertriangular block in the upper left corner.
 * @param parameterDim : Additional empty rows to add after the dense block.
 * @return sparsity pattern.
 */
SparsityPattern getHessianVariableSparsity(int variableDim, int parameterDim);

/**
 * Get number of nonzeros in sparsity pattern
 *
 * @param sparsityPattern
 * @return number of nonzero elements
 */
size_t getNumberOfNonZeros(const SparsityPattern& sparsityPattern);

}  // namespace cppad_sparsity
}  // namespace ocs2
