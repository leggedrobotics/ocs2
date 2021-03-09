//
// Created by rgrandia on 04.03.21.
//

#include <gtest/gtest.h>

#include "ocs2_sqp/MultipleShootingSolver.h"

#include <ocs2_core/OCS2NumericTraits.h>

using namespace ocs2;

TEST(test_discretization, noEvents_plusEps) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3 + std::numeric_limits<scalar_t>::epsilon();
  scalar_t dt = 0.1;
  scalar_t eps = OCS2NumericTraits<scalar_t>::limitEpsilon();
  scalar_array_t eventTimes {};

  auto time = MultipleShootingSolver::timeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes, eps);
  ASSERT_EQ(time[0], initTime);
  ASSERT_DOUBLE_EQ(time[1], initTime + dt);
  ASSERT_EQ(time[2], finalTime); // Absorbs the point at 0.3
}

TEST(test_discretization, noEvents_minEps) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3 - std::numeric_limits<scalar_t>::epsilon();
  scalar_t dt = 0.1;
  scalar_t eps = OCS2NumericTraits<scalar_t>::limitEpsilon();
  scalar_array_t eventTimes {};

  auto time = MultipleShootingSolver::timeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes, eps);
  ASSERT_EQ(time[0], initTime);
  ASSERT_DOUBLE_EQ(time[1], initTime + dt);
  ASSERT_EQ(time[2], finalTime);
}

TEST(test_discretization, withEvents) {
  scalar_t initTime = 3.0;
  scalar_t finalTime = 4.0;
  scalar_t dt = 0.1;
  scalar_t eps = OCS2NumericTraits<scalar_t>::limitEpsilon();
  scalar_array_t eventTimes {3.25, 3.4, 3.88, 4.02, 4.5};

  auto time = MultipleShootingSolver::timeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes, eps);
//  timeDiscretization = {3.0, 3.1, 3.2, 3.25, 3.35, 3.4, 3.5, 3.6, 3.7, 3.8, 3.88, 3.98, 4.0}
  ASSERT_EQ(time[0], initTime);
  ASSERT_DOUBLE_EQ(time[1], initTime + dt);
  ASSERT_DOUBLE_EQ(time[2], initTime + 2.*dt);
  ASSERT_EQ(time[3], eventTimes[0] + eps);
  ASSERT_DOUBLE_EQ(time[4], eventTimes[0] + dt);
  ASSERT_EQ(time[5], eventTimes[1] + eps);
  ASSERT_DOUBLE_EQ(time[6], eventTimes[1] + dt);
  ASSERT_DOUBLE_EQ(time[7], eventTimes[1] + 2.*dt);
  ASSERT_DOUBLE_EQ(time[8], eventTimes[1] + 3.*dt);
  ASSERT_DOUBLE_EQ(time[9], eventTimes[1] + 4.*dt);
  ASSERT_EQ(time[10], eventTimes[2] + eps);
  ASSERT_EQ(time[11], finalTime); // Absorbs 3.98
}