

#include <ocs2_core/automatic_differentiation/CppAdSparsity.h>

namespace ocs2 {

namespace cppad_sparsity {

SparsityPattern getIntersection(const SparsityPattern& p0, const SparsityPattern& p1) {
  assert(p0.size() == p1.size());
  const auto numRows = p0.size();

  SparsityPattern result(p0.size());
  for (int row = 0; row < numRows; row++) {
    std::set_intersection(p0[row].begin(), p0[row].end(), p1[row].begin(), p1[row].end(), std::inserter(result[row], result[row].begin()));
  }
  return result;
}

SparsityPattern getJacobianVariableSparsity(int rangeDim, int variableDim) {
  // Jacobian : all variables are declared non-zero
  SparsityPattern jacobianSparsity(rangeDim);
  for (auto& sparsityRow : jacobianSparsity) {
    for (size_t i = 0; i < variableDim; i++) {
      sparsityRow.insert(i);
    }
  }
  return jacobianSparsity;
}

SparsityPattern getHessianVariableSparsity(int variableDim, int parameterDim) {
  // Hessian : all upper triangular variable entries are declared non-zero
  SparsityPattern hessianSparsity(variableDim + parameterDim);
  for (size_t i = 0; i < variableDim; i++) {
    for (size_t j = i; j < variableDim; j++) {
      hessianSparsity[i].insert(j);
    }
  }
  return hessianSparsity;
}

size_t getNumberOfNonZeros(const SparsityPattern& sparsityPattern) {
  size_t nnz = 0;
  for (const auto& row : sparsityPattern) {
    nnz += row.size();
  }
  return nnz;
}

}  // namespace cppad_sparsity
}  // namespace ocs2
