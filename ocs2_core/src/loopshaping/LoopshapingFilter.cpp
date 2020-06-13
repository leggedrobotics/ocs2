//
// Created by rgrandia on 14.02.20.
//

#include <iostream>

#include <ocs2_core/loopshaping/LoopshapingFilter.h>

namespace ocs2 {

Filter::Filter() {
  A_.resize(0, 0);
  B_.resize(0, 0);
  C_.resize(0, 0);
  D_.resize(0, 0);
}

Filter::Filter(const matrix_t& A, const matrix_t& B, const matrix_t& C, const matrix_t& D)
    : A_(A), B_(B), C_(C), D_(D), numStates_(A.rows()), numInputs_(B.cols()), numOutputs_(C.rows()) {
  checkSize();
}

void Filter::print() const {
  std::cout << "numStates: " << numStates_ << std::endl;
  std::cout << "numInputs: " << numInputs_ << std::endl;
  std::cout << "numOutputs: " << numOutputs_ << std::endl;
  std::cout << "A: \n" << A_ << std::endl;
  std::cout << "B: \n" << B_ << std::endl;
  std::cout << "C: \n" << C_ << std::endl;
  std::cout << "D: \n" << D_ << std::endl;
}

void Filter::findEquilibriumForOutput(const vector_t& y, vector_t& x, vector_t& u) const {
  // Solve (given y)
  // [0  =  [  A    B    [x
  //  y]       C    D  ]  u]
  matrix_t ABCD(numStates_ + numInputs_, numStates_ + numInputs_);
  ABCD << A_, B_, C_, D_;

  vector_t x_u;
  vector_t zero_y(numStates_ + numOutputs_);
  zero_y << vector_t::Zero(numStates_), y;

  x_u = ABCD.colPivHouseholderQr().solve(zero_y);

  x = x_u.segment(0, numStates_);
  u = x_u.segment(numStates_, numInputs_);
}

void Filter::findEquilibriumForInput(const vector_t& u, vector_t& x, vector_t& y) const {
  // Solve (given u)
  // [0  =  [  A    B    [x
  //  y]       C    D  ]  u]
  x = -A_.colPivHouseholderQr().solve(B_ * u);
  y = C_ * x + D_ * u;
}

void Filter::checkSize() const {
  bool correct = true;
  // check number of state
  correct &= numStates_ == A_.rows();
  correct &= numStates_ == B_.rows();
  correct &= numStates_ == C_.cols();
  // Check number of inputs
  correct &= numInputs_ == B_.cols();
  correct &= numInputs_ == D_.cols();
  // Check number of outputs
  correct &= numOutputs_ == C_.rows();
  correct &= numOutputs_ == D_.rows();
  if (!correct) {
    print();
    throw std::runtime_error("Loopshaping: Filer: Matrix dimensions not consistent.");
  }
}

}  // namespace ocs2
