

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

  const auto gnApprox = adInterface.getJacobianAndGaussNewtonHessian(x);
  ASSERT_TRUE(gnApprox.second.isApprox(gnApprox.first.transpose() * gnApprox.first));
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

  const auto gnApprox = adInterface.getJacobianAndGaussNewtonHessian(x, p);
  ASSERT_TRUE(gnApprox.second.isApprox(gnApprox.first.transpose() * gnApprox.first));
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

  const auto gnApprox = adInterface.getJacobianAndGaussNewtonHessian(x, p);
  ASSERT_TRUE(gnApprox.second.isApprox(gnApprox.first.transpose() * gnApprox.first));
}
