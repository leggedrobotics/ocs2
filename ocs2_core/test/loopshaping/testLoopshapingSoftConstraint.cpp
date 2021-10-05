

#include "testLoopshapingSoftConstraint.h"

#include <ocs2_core/test/testTools.h>

using namespace ocs2;

TEST(TestFixtureLoopShapingSoftConstraint, testStateInputApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingSoftConstraint test(config);
    test.testStateInputApproximation();
  }
}

TEST(TestFixtureLoopShapingSoftConstraint, testStateApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingSoftConstraint test(config);
    test.testStateApproximation();
  }
}
