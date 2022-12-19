//
// Created by rgrandia on 29.04.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/logic/SingleLegLogic.h"

using namespace switched_model;

TEST(TestContactTiming, stanceOnly) {
  ContactTiming stanceOnly{timingNaN(), timingNaN()};
  std::vector<ContactTiming> stanceOnlyVector = {stanceOnly};

  ASSERT_FALSE(hasStartTime(stanceOnly));
  ASSERT_FALSE(hasEndTime(stanceOnly));
  ASSERT_FALSE(startsWithSwingPhase(stanceOnlyVector));
  ASSERT_TRUE(startsWithStancePhase(stanceOnlyVector));
  ASSERT_FALSE(endsWithSwingPhase(stanceOnlyVector));
  ASSERT_TRUE(endsWithStancePhase(stanceOnlyVector));
  ASSERT_FALSE(touchesDownAtLeastOnce(stanceOnlyVector));
  ASSERT_FALSE(liftsOffAtLeastOnce(stanceOnlyVector));
}

TEST(TestContactTiming, SwingOnly) {
  std::vector<ContactTiming> swingOnlyVector = {};

  ASSERT_TRUE(startsWithSwingPhase(swingOnlyVector));
  ASSERT_FALSE(startsWithStancePhase(swingOnlyVector));
  ASSERT_TRUE(endsWithSwingPhase(swingOnlyVector));
  ASSERT_FALSE(endsWithStancePhase(swingOnlyVector));
  ASSERT_FALSE(touchesDownAtLeastOnce(swingOnlyVector));
  ASSERT_FALSE(liftsOffAtLeastOnce(swingOnlyVector));
}

TEST(TestContactTiming, SwingToStance) {
  ContactTiming stance{0.0, timingNaN()};
  std::vector<ContactTiming> swingToStanceVector = {stance};

  ASSERT_TRUE(hasStartTime(stance));
  ASSERT_FALSE(hasEndTime(stance));
  ASSERT_TRUE(startsWithSwingPhase(swingToStanceVector));
  ASSERT_FALSE(startsWithStancePhase(swingToStanceVector));
  ASSERT_FALSE(endsWithSwingPhase(swingToStanceVector));
  ASSERT_TRUE(endsWithStancePhase(swingToStanceVector));
  ASSERT_TRUE(touchesDownAtLeastOnce(swingToStanceVector));
  ASSERT_FALSE(liftsOffAtLeastOnce(swingToStanceVector));
}

TEST(TestContactTiming, StanceToSwing) {
  ContactTiming stance{timingNaN(), 0.0};
  std::vector<ContactTiming> stanceToSwingVector = {stance};

  ASSERT_FALSE(hasStartTime(stance));
  ASSERT_TRUE(hasEndTime(stance));
  ASSERT_FALSE(startsWithSwingPhase(stanceToSwingVector));
  ASSERT_TRUE(startsWithStancePhase(stanceToSwingVector));
  ASSERT_TRUE(endsWithSwingPhase(stanceToSwingVector));
  ASSERT_FALSE(endsWithStancePhase(stanceToSwingVector));
  ASSERT_FALSE(touchesDownAtLeastOnce(stanceToSwingVector));
  ASSERT_TRUE(liftsOffAtLeastOnce(stanceToSwingVector));
}