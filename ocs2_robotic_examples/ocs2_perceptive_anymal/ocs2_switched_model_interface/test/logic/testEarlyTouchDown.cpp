/*
 * testEarlyTouchDown.cpp
 *
 *  Created on: Jun 4, 2020
 *      Author: Marko Bjelonic
 */

// gtest
#include <gtest/gtest.h>

// ocs2 switched model interface
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/logic/GaitAdaptation.h>
#include <ocs2_switched_model_interface/logic/GaitSchedule.h>

using namespace switched_model;

TEST(testEarlyTouchDown, testTrot) {
  // Generate gait schedule.
  Gait gait;
  gait.duration = 0.8;
  gait.eventPhases = {0.45, 0.5, 0.95};
  gait.modeSequence = {ModeNumber::LF_RH, ModeNumber::STANCE, ModeNumber::RF_LH,
                       ModeNumber::STANCE};
  GaitSchedule gaitSchedule(0.0, gait);
  gaitSchedule.setGaitAfterTime(
      gait, gaitSchedule.getCurrentTime() + 0.99 * gait.duration);

  // Early touch down gait adaptation RF and LH should be in touch down.
  feet_array_t<bool> earlyTouchDownPerLeg{true, true, true, true};
  auto earlyTouchDownGaitAdaptor =
      earlyTouchDownAdaptation(earlyTouchDownPerLeg);
  gaitSchedule.adaptCurrentGait(earlyTouchDownGaitAdaptor);

  // Check.
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[0] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[1] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[2] ==
              ModeNumber::RF_LH);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[3] ==
              ModeNumber::STANCE);

  // Early touch down gait adaptation LF and RH should be in touch down.
  gaitSchedule.advanceToTime(gait.duration * gait.eventPhases[1]);
  gaitSchedule.adaptCurrentGait(earlyTouchDownGaitAdaptor);

  // Check.
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[0] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[1] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[2] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[3] ==
              ModeNumber::STANCE);

  // Check that the next gait was not adapted.
  gaitSchedule.advanceToTime(gait.duration);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[0] ==
              ModeNumber::LF_RH);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[1] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[2] ==
              ModeNumber::RF_LH);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[3] ==
              ModeNumber::STANCE);
}

TEST(testEarlyTouchDown, testOverhangingSwingPhases) {
  // Generate gait schedule.
  Gait gait;
  gait.duration = 1.0;
  gait.eventPhases = {0.5};
  gait.modeSequence = {ModeNumber::RF_LH_RH, ModeNumber::LH_RH};
  GaitSchedule gaitSchedule(0.0, gait);
  Gait nextGait;
  nextGait.duration = 0.5;
  nextGait.eventPhases = {0.5};
  nextGait.modeSequence = {ModeNumber::LH_RH, ModeNumber::LF_LH_RH};
  gaitSchedule.setGaitAfterTime(
      nextGait, gaitSchedule.getCurrentTime() + 0.99 * gait.duration);

  // Early touch down gait adaptation LF should be in touch down.
  feet_array_t<bool> earlyTouchDownPerLeg{true, true, false, false};
  auto earlyTouchDownGaitAdaptor =
      earlyTouchDownAdaptation(earlyTouchDownPerLeg);
  gaitSchedule.adaptCurrentGait(earlyTouchDownGaitAdaptor);

  // Check.
  const auto modeSchedule =
      gaitSchedule.getModeSchedule(gait.duration + nextGait.duration);
  ASSERT_TRUE(modeSchedule.modeSequence[0] == ModeNumber::STANCE);
  ASSERT_TRUE(modeSchedule.modeSequence[1] == ModeNumber::LF_LH_RH);
  ASSERT_EQ(modeSchedule.modeSequence.size(), 2);

  // Early touch down gait adaptation RF should be in touch down.
  gaitSchedule.advanceToTime(gait.eventPhases[0] * gait.duration);
  gaitSchedule.adaptCurrentGait(earlyTouchDownGaitAdaptor);

  // Check.
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[0] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[1] ==
              ModeNumber::STANCE);
  gaitSchedule.advanceToTime(gait.duration);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[0] ==
              ModeNumber::STANCE);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[1] ==
              ModeNumber::STANCE);
  gaitSchedule.advanceToTime(gait.duration + nextGait.duration);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[0] ==
              ModeNumber::LH_RH);
  ASSERT_TRUE(gaitSchedule.getCurrentGait().modeSequence[1] ==
              ModeNumber::LF_LH_RH);
}