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
 *  Vector contains 1 set per row, each set contains the indices of non-zero columns
 */
using SparsityPattern = std::vector<std::set<size_t>>;

template <typename ad_fun_t>
SparsityPattern getJacobianSparsityPattern(ad_fun_t& fun) {
  return CppAD::cg::jacobianSparsitySet<SparsityPattern>(fun);
}

template <typename ad_fun_t>
SparsityPattern getHessianSparsityPattern(ad_fun_t& fun) {
  return CppAD::cg::hessianSparsitySet<SparsityPattern>(fun);
}

SparsityPattern getIntersection(const SparsityPattern& p0, const SparsityPattern& p1);
SparsityPattern getJacobianVariableSparsity(int rangeDim, int variableDim);
SparsityPattern getHessianVariableSparsity(int variableDim, int parameterDim);

}  // namespace cppad_sparsity
}  // namespace ocs2
