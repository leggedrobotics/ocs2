

#include <gtest/gtest.h>

#include "commonFixture.h"

class CppAdInterfaceNoParameterFixture : public CommonCppAdNoParameterFixture {};
class CppAdInterfaceParameterizedFixture : public CommonCppAdParameterizedFixture {};

TEST_F(CppAdInterfaceNoParameterFixture, testModelGeneration) {
  ocs2::CppAdInterface<scalar_t> adInterface(funImpl, rangeDim_, variableDim_, "testModelWithoutParameters");

  adInterface.createModels(ocs2::CppAdInterface<scalar_t>::ApproximationOrder::Second, true);
  dynamic_vector_t x = dynamic_vector_t::Random(variableDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x).isApprox(testFun(x)));
  ASSERT_TRUE(adInterface.getJacobian(x).isApprox(testJacobian(x)));
  ASSERT_TRUE(adInterface.getHessian(0, x).isApprox(testHessian(x)));
}

TEST_F(CppAdInterfaceParameterizedFixture, testModelGeneration) {
  ocs2::CppAdInterface<scalar_t> adInterface(funImpl, rangeDim_, variableDim_, parameterDim_, "testModelWithParameters");

  adInterface.createModels(ocs2::CppAdInterface<scalar_t>::ApproximationOrder::Second, true);
  dynamic_vector_t x = dynamic_vector_t::Random(variableDim_);
  dynamic_vector_t p = dynamic_vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));
}

TEST_F(CppAdInterfaceParameterizedFixture, loadIfAvailable) {
  ocs2::CppAdInterface<scalar_t> adInterface(funImpl, rangeDim_, variableDim_, parameterDim_, "testModelLoadIfAvailable");

  adInterface.loadModelsIfAvailable(ocs2::CppAdInterface<scalar_t>::ApproximationOrder::Second, true);
  dynamic_vector_t x = dynamic_vector_t::Random(variableDim_);
  dynamic_vector_t p = dynamic_vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}