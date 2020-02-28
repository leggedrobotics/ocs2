//
// Created by rgrandia on 25.02.20.
//

#include "ocs2_test_problems/UnconstrainedTestFixture.h"

TYPED_TEST(UnconstrainedTestFixture, DoesBlah) {
  std::cout << "Running a test: " << std::endl;
  std::cout << "StateDim: " << this->testDims.stateDim << std::endl;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}