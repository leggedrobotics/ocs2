#include "testLoopshapingCost.h"

using namespace ocs2;

TEST(TestFixtureLoopShapingCost, testStateInputCostApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingCost test(config);
    test.testStateInputCostApproximation();
  }
};

TEST(TestFixtureLoopShapingCost, testStateCostApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingCost test(config);
    test.testStateCostApproximation();
  }
}
