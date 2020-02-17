//
// Created by rgrandia on 14.02.20.
//

#include <ocs2_core/loopshaping/LoopshapingFilter.h>

namespace ocs2 {

Filter::Filter() {
  A_.resize(0, 0);
  B_.resize(0, 0);
  C_.resize(0, 0);
  D_.resize(0, 0);
}

Filter::Filter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D)
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
