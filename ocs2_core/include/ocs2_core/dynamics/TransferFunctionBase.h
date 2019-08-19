//
// Created by ruben on 27.07.18.
//

#ifndef OCS2_TRANSFERFUNCTIONBASE_H
#define OCS2_TRANSFERFUNCTIONBASE_H

#include <Eigen/Dense>
#include <iostream>

namespace ocs2 {

inline void padeApproximation(double timeDelay, Eigen::VectorXd& numCoefficients, Eigen::VectorXd& denCoefficients, size_t numZeros,
                              size_t numPoles) {
  numCoefficients.resize(numZeros + 1);
  denCoefficients.resize(numPoles + 1);

  if (numZeros == 0 && numPoles == 1) {
    numCoefficients << 1.0;
    denCoefficients << timeDelay, 1.0;
    return;
  }

  if (numZeros == 1 && numPoles == 0) {
    numCoefficients << -timeDelay, 1.0;
    denCoefficients << 1.0;
    return;
  }

  if (numZeros == 1 && numPoles == 1) {
    numCoefficients << -0.5 * timeDelay, 1.0;
    denCoefficients << 0.5 * timeDelay, 1.0;
    return;
  }

  std::runtime_error("padeApproximation not implemented for nZeros=" + std::to_string(numZeros) + " nPoles= " + std::to_string(numPoles));
};

inline Eigen::VectorXd multiplyPolynomials(const Eigen::VectorXd& p_lhs, const Eigen::VectorXd& p_rhs) {
  Eigen::VectorXd p_result(p_lhs.size() + p_rhs.size() - 1);
  p_result.setZero();
  for (int i = 0; i < p_lhs.size(); i++) {
    for (int j = 0; j < p_lhs.size(); j++) {
      p_result(i + j) += p_lhs(i) * p_rhs(j);
    }
  }
  return p_result;
}

class TransferFunctionBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TransferFunctionBase(Eigen::VectorXd numCoefficients, Eigen::VectorXd denCoefficients, double timedelay = 0.0, bool balance = true)
      : numCoefficients_(numCoefficients), denCoefficients_(denCoefficients), timeDelay_(timedelay), balance_(balance){};

  void absorbDelay(size_t numZeros, size_t numPoles) {
    if (timeDelay_ > delayTol_) {
      // Approximate delay
      Eigen::VectorXd padeNum, padeDen;
      ocs2::padeApproximation(timeDelay_, padeNum, padeDen, numZeros, numPoles);

      // multiply approximated delay
      numCoefficients_ = multiplyPolynomials(numCoefficients_, padeNum);
      denCoefficients_ = multiplyPolynomials(denCoefficients_, padeDen);
    }
    delayAbsorbed_ = true;
  };

  void normalize() {
    double scaling = denCoefficients_(0);
    numCoefficients_ /= scaling;
    denCoefficients_ /= scaling;
  }

  void getStateSpace(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D) {
    if (numCoefficients_.size() > denCoefficients_.size()) {
      throw std::runtime_error("Transfer function must be proper to convert to a state space model");
    }

    // Absorb delay and normalize
    if (!delayAbsorbed_) {  // Default approximation of time delay
      this->absorbDelay(1, 1);
    }
    this->normalize();

    size_t numStates = denCoefficients_.size() - 1;
    size_t numInputs = 1;
    size_t numOutputs = 1;

    A.resize(numStates, numStates);
    B.resize(numStates, numInputs);
    C.resize(numOutputs, numStates);
    D.resize(numOutputs, numInputs);

    // prepend zeros to numerator
    Eigen::VectorXd numExtended(denCoefficients_.size());
    numExtended.setZero();
    numExtended.tail(numCoefficients_.size()) = numCoefficients_;

    // Create strictly proper transfer function
    D(0) = numExtended(0);
    numExtended -= denCoefficients_ * D(0);

    if (numStates > 0) {
      A << -denCoefficients_.tail(numStates).transpose(), Eigen::MatrixXd::Identity(numStates - 1, numStates);
      B << 1.0, Eigen::VectorXd::Zero(numStates - 1);
      C << numExtended.tail(numStates).transpose();

      if (balance_) {
        Eigen::MatrixXd T(numStates, numStates);
        T.setZero();
        T.diagonal() = denCoefficients_.tail(numStates).cwiseSqrt();
        T(0, 0) = 1.0;
        A = T * A.eval() * T.inverse();
        B = T * B.eval();
        C = C.eval() * T.inverse();
      }
    }
  }

 private:
  Eigen::VectorXd numCoefficients_, denCoefficients_;
  double timeDelay_;
  double delayTol_ = 1e-6;
  bool delayAbsorbed_ = false;
  bool balance_;
};

inline void tf2ss(Eigen::VectorXd numCoefficients, Eigen::VectorXd denCoefficients, Eigen::MatrixXd& A, Eigen::MatrixXd& B,
                  Eigen::MatrixXd& C, Eigen::MatrixXd& D, double timeDelay = 0.0, bool balance = true) {
  TransferFunctionBase tf(numCoefficients, denCoefficients, timeDelay, balance);
  tf.getStateSpace(A, B, C, D);
}

};  // namespace ocs2

#endif  // OCS2_TRANSFERFUNCTIONBASE_H
