

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/TransferFunctionBase.h>
#include <Eigen/Dense>

namespace ocs2 {

class Filter {
 public:
  Filter();

  Filter(matrix_t A, matrix_t B, matrix_t C, matrix_t D);

  size_t getNumStates() const { return numStates_; };
  size_t getNumInputs() const { return numInputs_; };
  size_t getNumOutputs() const { return numOutputs_; };

  const matrix_t& getA() const { return A_; };
  const matrix_t& getB() const { return B_; };
  const matrix_t& getC() const { return C_; };
  const matrix_t& getD() const { return D_; };

  /// Get the diagonal of the filter matrices
  const vector_t& getAdiag() const { return a_; };
  const vector_t& getBdiag() const { return b_; };
  const vector_t& getCdiag() const { return c_; };
  const vector_t& getDdiag() const { return d_; };

  void print() const;

  void findEquilibriumForOutput(const vector_t& y, vector_t& x, vector_t& u) const;
  void findEquilibriumForInput(const vector_t& u, vector_t& x, vector_t& y) const;

 private:
  void checkSize() const;

  matrix_t A_, B_, C_, D_;
  vector_t a_, b_, c_, d_;
  size_t numStates_ = 0;
  size_t numInputs_ = 0;
  size_t numOutputs_ = 0;
};

}  // namespace ocs2
