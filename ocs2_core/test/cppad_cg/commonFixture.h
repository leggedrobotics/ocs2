#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/CppAdSparsity.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2 {

class CommonCppAdNoParameterFixture : public ::testing::Test {
 public:
  using ad_fun_t = CppAdInterface::ad_fun_t;
  using ad_vector_t = ocs2::ad_vector_t;

  const size_t variableDim_ = 3;
  const size_t rangeDim_ = 1;
  const size_t parameterDim_ = 0;

  CommonCppAdNoParameterFixture() {
    create();
    init();
  }

  virtual ~CommonCppAdNoParameterFixture() = default;

  void create(){};

  static void funImpl(const ad_vector_t& x, ad_vector_t& y) {
    // the model equation
    y.resize(1);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * x(0) * x(1) * x(2);
  }

  vector_t testFun(const vector_t& x) {
    vector_t y(rangeDim_);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * x(0) * x(1) * x(2);
    return y;
  }

  matrix_t testJacobian(const vector_t& x) {
    matrix_t jacobian(rangeDim_, variableDim_);
    // y(0):
    jacobian(0, 0) = 1.0 + 0.5 * x(1) + 2.0 * x(1) * x(2);
    jacobian(0, 1) = 0.5 * x(0) + 2.0 * x(0) * x(2);
    jacobian(0, 2) = 2.0 * x(0) * x(1);
    return jacobian;
  }

  matrix_t testHessian(const vector_t& x) {
    matrix_t hessian(variableDim_, variableDim_);
    hessian.setZero();

    hessian(0, 0) = 0.0;
    hessian(0, 1) = 0.5 + 2.0 * x(2);
    hessian(1, 0) = hessian(0, 1);
    hessian(0, 2) = 2.0 * x(1);
    hessian(2, 0) = hessian(0, 2);
    hessian(1, 1) = 0.0;
    hessian(1, 2) = 2.0 * x(0);
    hessian(2, 1) = hessian(1, 2);
    hessian(2, 2) = 0.0;

    return hessian;
  }

  void init() {}
};

class CommonCppAdParameterizedFixture : public ::testing::Test {
 public:
  using ad_fun_t = CppAdInterface::ad_fun_t;
  using ad_vector_t = ocs2::ad_vector_t;

  const size_t variableDim_ = 2;
  const size_t rangeDim_ = 2;
  const size_t parameterDim_ = 1;

  CommonCppAdParameterizedFixture() {
    create();
    init();
  }

  virtual ~CommonCppAdParameterizedFixture() = default;

  void create() {
    // set and declare independent variables and start tape recording
    ad_vector_t xp(variableDim_ + parameterDim_);
    xp.setOnes();
    CppAD::Independent(xp);

    ad_vector_t x = xp.segment(0, variableDim_);
    ad_vector_t p = xp.segment(variableDim_, parameterDim_);
    ad_vector_t y;
    funImpl(x, p, y);

    // create f: x -> y and stop tape recording
    fun_.reset(new ad_fun_t(xp, y));
    fun_->optimize();

    jacobianSparsity_.resize(rangeDim_);
    jacobianSparsity_[0] = {0, 1, 2};
    jacobianSparsity_[1] = {0, 1};

    hessianSparsity_.resize(variableDim_ + parameterDim_);
    hessianSparsity_[0] = {0, 1};
    hessianSparsity_[1] = {0, 1, 2};
    hessianSparsity_[2] = {1};
  }

  static void funImpl(const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    // the model equation
    y.resize(2);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * p(0) * x(1) * x(1);
    y(1) = x(0) * x(0) * x(1) / 2.0;
  }

  vector_t testFun(const vector_t& x, const vector_t& p) {
    vector_t y(rangeDim_);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * p(0) * x(1) * x(1);
    y(1) = x(0) * x(0) * x(1) / 2.0;
    return y;
  }

  matrix_t testJacobian(const vector_t& x, const vector_t& p) {
    matrix_t jacobian(rangeDim_, variableDim_);
    // y(0):
    jacobian(0, 0) = 1.0 + 0.5 * x(1);
    jacobian(0, 1) = 0.5 * x(0) + 4.0 * p(0) * x(1);
    // y(1):
    jacobian(1, 0) = x(0) * x(1);
    jacobian(1, 1) = x(0) * x(0) / 2.0;
    return jacobian;
  }

  matrix_t testHessian(int outputIndex, const vector_t& x, const vector_t& p) {
    matrix_t hessian(variableDim_, variableDim_);

    switch (outputIndex) {
      case 0:
        hessian(0, 0) = 0.0;
        hessian(0, 1) = 0.5;
        hessian(1, 0) = hessian(0, 1);
        hessian(1, 1) = 4.0 * p(0);
        break;
      case 1:
        hessian(0, 0) = x(1);
        hessian(0, 1) = x(0);
        hessian(1, 0) = hessian(0, 1);
        hessian(1, 1) = 0.0;
        break;
    }

    return hessian;
  }

  void init() {}

  std::unique_ptr<ad_fun_t> fun_;
  cppad_sparsity::SparsityPattern jacobianSparsity_;
  cppad_sparsity::SparsityPattern hessianSparsity_;
};

}  // namespace ocs2
