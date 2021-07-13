

#include <gtest/gtest.h>

#include "commonFixture.h"

using namespace ocs2;

class CppAdInterfaceNoParameterFixture : public CommonCppAdNoParameterFixture {};
class CppAdInterfaceParameterizedFixture : public CommonCppAdParameterizedFixture {};

TEST_F(CppAdInterfaceNoParameterFixture, testModelGeneration) {
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, "testModelWithoutParameters");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x).isApprox(testFun(x)));
  ASSERT_TRUE(adInterface.getJacobian(x).isApprox(testJacobian(x)));
  ASSERT_TRUE(adInterface.getHessian(0, x).isApprox(testHessian(x)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x).transpose() * testFun(x)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x).transpose() * testJacobian(x)));
}

TEST_F(CppAdInterfaceParameterizedFixture, testModelGeneration) {
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, "testModelWithParameters");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);
  vector_t p = vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x, p).transpose() * testFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x, p).transpose() * testJacobian(x, p)));
}

TEST_F(CppAdInterfaceParameterizedFixture, loadIfAvailable) {
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, "testModelLoadIfAvailable");

  adInterface.loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);
  vector_t p = vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x, p).transpose() * testFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x, p).transpose() * testJacobian(x, p)));
}
