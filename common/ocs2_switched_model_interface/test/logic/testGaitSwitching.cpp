//
// Created by rgrandia on 15.03.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/logic/GaitSwitching.h"

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

TEST(TestAdvance, withinSameGait) {
  std::vector<Gait> gaitSchedule = {singleModeGait};

  const double dt = 0.1;
  const double phase = 0.1;
  double advancedPhase;
  std::vector<Gait>::iterator nextGaitIt;
  std::tie(advancedPhase, nextGaitIt) = advancePhase(phase, dt, gaitSchedule.begin(), gaitSchedule.end());

  ASSERT_DOUBLE_EQ(advancedPhase, phase + dt / singleModeGait.duration);
  ASSERT_EQ(nextGaitIt, gaitSchedule.begin());

  // Looping the gait
  std::tie(advancedPhase, nextGaitIt) = advancePhase(phase, dt + 3 * singleModeGait.duration, gaitSchedule.begin(), gaitSchedule.end());

  // We loose some precision while recursing over the gaits.
  ASSERT_NEAR(advancedPhase, phase + dt / singleModeGait.duration, 1e-12);
  ASSERT_EQ(nextGaitIt, gaitSchedule.begin());
}

TEST(TestAdvance, advanceExactlyOneGaitcycle) {
  std::vector<Gait> gaitSchedule = {singleModeGait};

  const double dt = singleModeGait.duration;
  const double phase = 0.0;
  double advancedPhase;
  std::vector<Gait>::iterator nextGaitIt;
  std::tie(advancedPhase, nextGaitIt) = advancePhase(phase, dt, gaitSchedule.begin(), gaitSchedule.end());
  ASSERT_DOUBLE_EQ(advancedPhase, 0.0);
}

TEST(TestAdvance, transitionGait) {
  std::vector<Gait> gaitSchedule = {singleModeGait, multiModeGait};

  const double dt = 0.1;
  const double phase = 0.9;
  double advancedPhase;
  std::vector<Gait>::iterator nextGaitIt;
  std::tie(advancedPhase, nextGaitIt) = advancePhase(phase, dt, gaitSchedule.begin(), gaitSchedule.end());

  ASSERT_DOUBLE_EQ((1.0 - phase) * singleModeGait.duration + advancedPhase * multiModeGait.duration, dt);
  ASSERT_EQ(nextGaitIt, gaitSchedule.end() - 1);

  // The same but, while looping the last gait
  std::tie(advancedPhase, nextGaitIt) = advancePhase(phase, dt + 3 * multiModeGait.duration, gaitSchedule.begin(), gaitSchedule.end());

  // We loose some precision while recursing over the gaits.
  ASSERT_NEAR((1.0 - phase) * singleModeGait.duration + advancedPhase * multiModeGait.duration, dt, 1e-12);
  ASSERT_EQ(nextGaitIt, gaitSchedule.end() - 1);
}

TEST(TestGetModeSchedule, singleModeLooping) {
  std::vector<Gait> gaitSchedule = {singleModeGait};

  auto modeSchedule = getModeSchedule(0.1, 123.0, 2.5, gaitSchedule.begin(), gaitSchedule.end());

  // No mode switches -> no event times
  ASSERT_EQ(modeSchedule.eventTimes.size(), 0);
  ASSERT_EQ(modeSchedule.modeSequence.front(), singleModeGait.modeSequence.front());
}

TEST(TestGetModeSchedule, gaitTransition) {
  std::vector<Gait> gaitSchedule = {singleModeGait, multiModeGait};

  double t0 = 0.1;
  auto modeSchedule =
      getModeSchedule(0.0, t0, singleModeGait.duration + multiModeGait.duration / 2, gaitSchedule.begin(), gaitSchedule.end());
  ASSERT_EQ(modeSchedule.eventTimes.size(), 2);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes[0], singleModeGait.duration + t0);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes[1], multiModeGait.eventPhases[0] * multiModeGait.duration + singleModeGait.duration + t0);
  ASSERT_EQ(modeSchedule.modeSequence[0], 1);
  ASSERT_EQ(modeSchedule.modeSequence[1], 0);
  ASSERT_EQ(modeSchedule.modeSequence[2], 1);
}

TEST(TestGetModeSchedule, doubleGaitTransition) {
  std::vector<Gait> gaitSchedule = {multiModeGait, singleModeGait, multiModeGait};

  double t0 = 0.1;
  // Progress to half the last gait and loop it once
  const double timeHorizon = multiModeGait.duration + singleModeGait.duration + 1.5 * multiModeGait.duration;
  auto modeSchedule = getModeSchedule(0.0, t0, timeHorizon, gaitSchedule.begin(), gaitSchedule.end());
  // The switch to single mode and back has two event times. Halve the multiMode as also two event times
  ASSERT_EQ(modeSchedule.eventTimes.size(), multiModeGait.eventPhases.size() + 2 + multiModeGait.eventPhases.size() + 2);
  ASSERT_EQ(modeSchedule.modeSequence.back(), 1);
}

TEST(TestGetModeSchedule, mergePhases) {
  std::vector<Gait> gaitSchedule = {singleModeGait, singleModeGait, multiModeGait};

  double t0 = 0.1;
  // Progress to half the last gait and loop it once
  const double timeHorizon = 2.0 * singleModeGait.duration + multiModeGait.duration;
  auto modeSchedule = getModeSchedule(0.0, t0, timeHorizon, gaitSchedule.begin(), gaitSchedule.end());
  // The switch to between the two single modes is merged to 1.
  ASSERT_EQ(modeSchedule.eventTimes.size(), 1 + multiModeGait.eventPhases.size());
  ASSERT_EQ(modeSchedule.modeSequence[0], 1);
  ASSERT_EQ(modeSchedule.modeSequence[1], 0);
  ASSERT_EQ(modeSchedule.modeSequence[2], 1);
  ASSERT_EQ(modeSchedule.modeSequence[3], 2);
}

TEST(TestGetModeSchedule, finalPhaseNotFullyInHorizon) {
  std::vector<Gait> gaitSchedule = {singleModeGait, multiModeGait};

  double t0 = 0.0;
  // Progress to half the last gait and loop it once
  const double timeHorizon = singleModeGait.duration + 0.999 * multiModeGait.duration;
  auto modeSchedule = getModeSchedule(0.0, t0, timeHorizon, gaitSchedule.begin(), gaitSchedule.end());
  // The switch to between the two single modes is merged to 1.
  ASSERT_EQ(modeSchedule.eventTimes.size(), 1 + multiModeGait.eventPhases.size());
  ASSERT_EQ(modeSchedule.modeSequence[0], 1);
  ASSERT_EQ(modeSchedule.modeSequence[1], 0);
  ASSERT_EQ(modeSchedule.modeSequence[2], 1);
  ASSERT_EQ(modeSchedule.modeSequence[3], 2);
}