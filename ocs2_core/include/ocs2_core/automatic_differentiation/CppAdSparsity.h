//
// Created by rgrandia on 16.08.19.
//

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
