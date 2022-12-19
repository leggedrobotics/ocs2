//
// Created by rgrandia on 15.03.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/logic/Gait.h"

using namespace switched_model;

const Gait singleModeGait = [] {
  Gait gait;
  gait.duration = 0.8;
  gait.eventPhases = {};
  gait.modeSequence = {1};
  return gait;
}();

const Gait multiModeGait = [] {
  Gait gait;
  gait.duration = 0.6;
  gait.eventPhases = {0.33, 0.66};
  gait.modeSequence = {0, 1, 2};
  return gait;
}();

TEST(TestGait, validGait) {
  ASSERT_TRUE(isValidGait(singleModeGait));
  ASSERT_TRUE(isValidGait(multiModeGait));

  // Negative duration
  ASSERT_FALSE(isValidGait({-0.1, {}, {0}}));
  // Event <= 0
  ASSERT_FALSE(isValidGait({0.1, {-0.1}, {0, 1}}));
  ASSERT_FALSE(isValidGait({0.1, {0}, {0, 1}}));
  // Event >= 1
  ASSERT_FALSE(isValidGait({0.1, {1}, {0, 1}}));
  ASSERT_FALSE(isValidGait({0.1, {1.1}, {0, 1}}));
  // Events not sorted
  ASSERT_FALSE(isValidGait({0.1, {0.6, 0.3}, {0, 1, 2}}));
  // Wrong number of mode
  ASSERT_FALSE(isValidGait({0.1, {}, {}}));
  ASSERT_FALSE(isValidGait({0.1, {0.1, 0.2}, {0}}));
}

TEST(TestPhase, isValidPhase) {
  ASSERT_FALSE(isValidPhase(-std::numeric_limits<double>::epsilon()));
  ASSERT_TRUE(isValidPhase(0.0));
  ASSERT_TRUE(isValidPhase(0.5));
  ASSERT_TRUE(isValidPhase(1.0 - std::numeric_limits<double>::epsilon()));
  ASSERT_FALSE(isValidPhase(1.0));
  ASSERT_FALSE(isValidPhase(1.0 + std::numeric_limits<double>::epsilon()));
}

TEST(TestPhaseWrap, wrapPhase) {
  // Wrap around edges
  ASSERT_DOUBLE_EQ(wrapPhase(0.0), 0.0);
  ASSERT_DOUBLE_EQ(wrapPhase(1.0), 0.0);
  ASSERT_DOUBLE_EQ(wrapPhase(1.0 - std::numeric_limits<double>::epsilon()), 1.0 - std::numeric_limits<double>::epsilon());
  ASSERT_DOUBLE_EQ(wrapPhase(1.0), 0.0);
  ASSERT_DOUBLE_EQ(wrapPhase(1.0 + std::numeric_limits<double>::epsilon()), std::numeric_limits<double>::epsilon());
  ASSERT_DOUBLE_EQ(wrapPhase(1.8), 0.8);
  ASSERT_DOUBLE_EQ(wrapPhase(-0.2), 0.8);
}

TEST(TestSingleModeGait, getModeFromPhase) {
  ASSERT_EQ(getModeFromPhase(0.0, singleModeGait), 1);
  ASSERT_EQ(getModeFromPhase(0.33, singleModeGait), 1);
  ASSERT_EQ(getModeFromPhase(1.0 - std::numeric_limits<double>::epsilon(), singleModeGait), 1);
}

TEST(TestMultiModeGait, getModeFromPhase) {
  ASSERT_EQ(getModeFromPhase(0.0, multiModeGait), 0);
  ASSERT_EQ(getModeFromPhase(0.1, multiModeGait), 0);
  ASSERT_EQ(getModeFromPhase(0.33, multiModeGait), 1);
  ASSERT_EQ(getModeFromPhase(0.42, multiModeGait), 1);
  ASSERT_EQ(getModeFromPhase(0.66, multiModeGait), 2);
  ASSERT_EQ(getModeFromPhase(1.0 - std::numeric_limits<double>::epsilon(), multiModeGait), 2);
}

TEST(TestMultiModeGait, timeLeft) {
  ASSERT_DOUBLE_EQ(timeLeftInMode(0.0, multiModeGait), 0.33 * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInMode(0.1, multiModeGait), (0.33 - 0.1) * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInMode(0.33 - std::numeric_limits<double>::epsilon(), multiModeGait),
                   std::numeric_limits<double>::epsilon() * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInMode(0.33, multiModeGait), (0.66 - 0.33) * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInMode(0.42, multiModeGait), (0.66 - 0.42) * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInMode(0.66, multiModeGait), (1.0 - 0.66) * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInMode(1.0 - std::numeric_limits<double>::epsilon(), multiModeGait),
                   std::numeric_limits<double>::epsilon() * multiModeGait.duration);

  ASSERT_DOUBLE_EQ(timeLeftInGait(0.0, multiModeGait), 1.0 * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInGait(0.5, multiModeGait), 0.5 * multiModeGait.duration);
  ASSERT_DOUBLE_EQ(timeLeftInGait(1.0 - std::numeric_limits<double>::epsilon(), multiModeGait),
                   std::numeric_limits<double>::epsilon() * multiModeGait.duration);
}