

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

  void print() const;

  void findEquilibriumForOutput(const vector_t& y, vector_t& x, vector_t& u) const;
  void findEquilibriumForInput(const vector_t& u, vector_t& x, vector_t& y) const;

 private:
  void checkSize() const;

  matrix_t A_, B_, C_, D_;
  size_t numStates_ = 0;
  size_t numInputs_ = 0;
  size_t numOutputs_ = 0;
};

}  // namespace ocs2
