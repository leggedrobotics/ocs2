#include "testLoopshapingConstraint.h"

using namespace ocs2;

TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintEvaluation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingConstraint test(config);
    test.testStateInputConstraintEvaluation();
  }
}

TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintLinearApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingConstraint test(config);
    test.testStateInputConstraintLinearApproximation();
  }
}

TEST(TestFixtureLoopShapingConstraint, testStateInputConstraintQuadraticApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingConstraint test(config);
    test.testStateInputConstraintQuadraticApproximation();
  }
}

TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintEvaluation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingConstraint test(config);
    test.testStateOnlyConstraintEvaluation();
  }
}

TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintLinearApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingConstraint test(config);
    test.testStateOnlyConstraintLinearApproximation();
  }
}

TEST(TestFixtureLoopShapingConstraint, testStateOnlyConstraintQuadraticApproximation) {
  for (const auto config : configNames) {
    TestFixtureLoopShapingConstraint test(config);
    test.testStateOnlyConstraintQuadraticApproximation();
  }
}
