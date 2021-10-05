
#include "testLoopshapingDynamics.h"

using namespace ocs2;

TEST(TestFixtureLoopShapingDynamics, evaluateDynamics) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingDynamics test(config);
    test.evaluateDynamics();
  }
};

TEST(TestFixtureLoopShapingDynamics, evaluateDynamicsApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingDynamics test(config);
    test.evaluateDynamicsApproximation();
  }
}

TEST(TestFixtureLoopShapingDynamics, evaluateJumpMap) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingDynamics test(config);
    test.evaluateJumpMap();
  }
}

TEST(TestFixtureLoopShapingDynamics, evaluateJumpMapApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingDynamics test(config);
    test.evaluateJumpMapApproximation();
  }
}
