

#pragma once

#include <ocs2_core/dynamics/TransferFunctionBase.h>
#include <Eigen/Dense>
#include <vector>

namespace ocs2 {

class Filter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Filter();

  Filter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D);

  size_t getNumStates() const { return numStates_; };
  size_t getNumInputs() const { return numInputs_; };
  size_t getNumOutputs() const { return numOutputs_; };

  const Eigen::MatrixXd& getA() const { return A_; };
  const Eigen::MatrixXd& getB() const { return B_; };
  const Eigen::MatrixXd& getC() const { return C_; };
  const Eigen::MatrixXd& getD() const { return D_; };

  void print() const;

  template <typename DerivedOutput, typename DerivedState, typename DerivedInput>
  void findEquilibriumForOutput(const DerivedOutput& y, DerivedState& x, DerivedInput& u) const {
    // Solve (given y)
    // [0  =  [  A    B    [x
    //  y]       C    D  ]  u]
    Eigen::MatrixXd ABCD(numStates_ + numInputs_, numStates_ + numInputs_);
    ABCD << A_, B_, C_, D_;

    Eigen::VectorXd x_u;
    Eigen::VectorXd zero_y(numStates_ + numOutputs_);
    zero_y << Eigen::VectorXd::Zero(numStates_), y;

    x_u = ABCD.colPivHouseholderQr().solve(zero_y);

    x = x_u.segment(0, numStates_);
    u = x_u.segment(numStates_, numInputs_);
  }

  template <typename DerivedInput, typename DerivedState, typename DerivedOutput>
  void findEquilibriumForInput(const DerivedInput& u, DerivedState& x, DerivedOutput& y) const {
    // Solve (given u)
    // [0  =  [  A    B    [x
    //  y]       C    D  ]  u]
    x = -A_.colPivHouseholderQr().solve(B_ * u);
    y = C_ * x + D_ * u;
  }

 private:
  void checkSize() const;

  Eigen::MatrixXd A_, B_, C_, D_;
  size_t numStates_ = 0;
  size_t numInputs_ = 0;
  size_t numOutputs_ = 0;
};

}  // namespace ocs2
