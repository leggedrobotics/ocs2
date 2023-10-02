//
// Created by rgrandia on 15.03.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/logic/GaitSchedule.h"
#include "ocs2_switched_model_interface/logic/GaitSwitching.h"

using namespace switched_model;

TEST(TestGaitSchedule, advanceSingleGait) {
  const double t0 = 11.0;
  GaitSchedule gaitSchedule(t0, {1.0, {}, {42}});
  ASSERT_EQ(gaitSchedule.getCurrentMode(), 42);
  ASSERT_DOUBLE_EQ(gaitSchedule.getCurrentPhase(), 0.0);

  gaitSchedule.advanceToTime(t0 + 100.0);
  ASSERT_EQ(gaitSchedule.getCurrentMode(), 42);

  gaitSchedule.advanceToTime(t0 + 110.0);
  ASSERT_EQ(gaitSchedule.getCurrentMode(), 42);
}

TEST(TestGaitSchedule, advanceTwoGaits) {
  const double t0 = 11.0;
  GaitSchedule gaitSchedule(t0, {1.0, {}, {42}});
  gaitSchedule.setNextGait({1.0, {}, {21}});

  ASSERT_EQ(gaitSchedule.getCurrentMode(), 42);

  gaitSchedule.advanceToTime(t0 + 0.5);
  ASSERT_EQ(gaitSchedule.getCurrentMode(), 42);

  gaitSchedule.advanceToTime(t0 + 1.0);
  ASSERT_EQ(gaitSchedule.getCurrentMode(), 21);
}

TEST(TestGaitSchedule, setNextGaitSchedule) {
  const double t0 = 11.0;
  const double timeHorizon = 5.0;
  const std::deque<Gait> gaitSequence = {Gait{1.0, {}, {42}}, Gait{1.0, {}, {21}}, Gait{0.5, {0.66}, {0, 1}}};

  // Add the gait schedule in two parts
  GaitSchedule gaitSchedule(t0, gaitSequence[0]);
  gaitSchedule.setGaitSequenceAfterCurrentGait({gaitSequence[1], gaitSequence[2]});

  // Check that the resulting mode schedule is equal
  auto checkModeSchedule = getModeSchedule(gaitSchedule.getCurrentPhase(), t0, timeHorizon, gaitSequence.begin(), gaitSequence.end());
  auto modeSchedule = gaitSchedule.getModeSchedule(timeHorizon);
  ASSERT_EQ(modeSchedule.eventTimes, checkModeSchedule.eventTimes);
  ASSERT_EQ(modeSchedule.modeSequence, checkModeSchedule.modeSequence);
}

TEST(TestGaitSchedule, setGaitScheduleAtTime) {
  const double t0 = 1.0;
  const double tInsert = 9.5;

  // Start with multi mode gait
  GaitSchedule gaitSchedule(t0, Gait{1.0, {0.5}, {0, 1}});

  // Set a new gait way pass the end of the current schedule
  gaitSchedule.setGaitAtTime(Gait{1.5, {}, {21}}, tInsert);
  auto modeSchedule = gaitSchedule.getModeSchedule((tInsert - t0) + 1.5);

  // Last gait was shrunk in duration
  ASSERT_DOUBLE_EQ(*(modeSchedule.eventTimes.end() - 2), tInsert - 0.25);
  ASSERT_DOUBLE_EQ(*(modeSchedule.eventTimes.end() - 3), tInsert - 0.5);

  // New gait inserted at the correct time
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes.back(), tInsert);
  ASSERT_EQ(modeSchedule.modeSequence.back(), 21);
}

TEST(TestGaitSchedule, setGaitScheduleAtTime_twice_current_gait) {
  const double t0 = 1.0;
  const double tGaitDuration = 1.0;
  const double tInsert = t0 + 2.0 * tGaitDuration;

  // Start with multi mode gait
  GaitSchedule gaitSchedule(t0, Gait{tGaitDuration, {0.5}, {0, 1}});

  // Set a new gait at twice the current gait
  gaitSchedule.setGaitAtTime(Gait{1.5, {}, {21}}, tInsert);
  auto modeSchedule = gaitSchedule.getModeSchedule((tInsert - t0) + 1.5);

  // Check expected mode schedule
  ASSERT_EQ(modeSchedule.modeSequence[0], 0);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes[0], t0 + 0.5);
  ASSERT_EQ(modeSchedule.modeSequence[1], 1);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes[1], t0 + 1.0);
  ASSERT_EQ(modeSchedule.modeSequence[2], 0);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes[2], t0 + 1.5);
  ASSERT_EQ(modeSchedule.modeSequence[3], 1);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes[3], t0 + 2.0);
  ASSERT_EQ(modeSchedule.modeSequence[4], 21);
}

TEST(TestGaitSchedule, setGaitScheduleAtTime_currentTime) {
  const double t0 = 1.0;
  const double tInsert = 1.0;

  // Start with multi mode gait
  GaitSchedule gaitSchedule(t0, Gait{1.0, {0.5}, {0, 1}});

  // Set a new gait at t0
  gaitSchedule.setGaitAtTime(Gait{1.5, {}, {21}}, tInsert);
  auto modeSchedule = gaitSchedule.getModeSchedule((tInsert - t0) + 1.5);

  // New gait is all that is left
  ASSERT_TRUE(modeSchedule.eventTimes.empty());
  ASSERT_EQ(modeSchedule.modeSequence.front(), 21);
}

TEST(TestGaitSchedule, setGaitScheduleAtTime_shortenCurrentGait) {
  const double T = 1.0;        // Gait repeats at t = 0.0, 1.0, 2.0
  const double Tinsert = 0.5;  // period of inserted gait
  const double tcurr = 0.7;
  const double tInsert = 0.9;  // insert before the current gait ends

  // Start with multi mode gait
  GaitSchedule gaitSchedule(0.0, Gait{T, {0.5}, {0, 1}});

  // Advance to get a non-zero current phase
  gaitSchedule.advanceToTime(tcurr);

  // Set a new gait by interrupting the current gait
  gaitSchedule.setGaitAtTime(Gait{Tinsert, {}, {21}}, tInsert);
  auto modeSchedule = gaitSchedule.getModeSchedule(Tinsert);

  // Change happens at requested time
  ASSERT_DOUBLE_EQ(gaitSchedule.timeLeftInCurrentGait(), tInsert - tcurr);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes.front(), tInsert);
}

TEST(TestGaitSchedule, setGaitScheduleAfterTime) {
  const double t0 = 18.6;
  const double tInsert = 20.5 + 1.0;

  // Start with multi mode gait
  GaitSchedule gaitSchedule(t0, Gait{2.0, {}, {0}});

  // Set a new gait way pass the end of the current schedule
  gaitSchedule.setGaitAfterTime(Gait{1.0, {}, {1}}, tInsert);
  auto modeSchedule = gaitSchedule.getModeSchedule((tInsert - t0) + 1.5);

  // New gait inserted at the correct time
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes.back(), 22.6);
}

TEST(TestGaitSchedule, setGaitScheduleAfterTimeWithNonzeroPhase) {
  const double t0 = 0.0;
  const double tadvance = 1.4;
  const double tInsert = 2.4;

  // Start with multi mode gait
  GaitSchedule gaitSchedule(t0, Gait{1.0, {}, {0}});

  // Advance
  gaitSchedule.advanceToTime(tadvance);

  // Set a new gait way pass the end of the current schedule
  gaitSchedule.setGaitAfterTime(Gait{1.0, {}, {1}}, tInsert);

  auto modeSchedule = gaitSchedule.getModeSchedule(2.0);

  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes.size(), 1);
  ASSERT_DOUBLE_EQ(modeSchedule.eventTimes.front(), 3.0);  // First insert opportunity
  ASSERT_EQ(modeSchedule.modeSequence.front(), 0);
  ASSERT_EQ(modeSchedule.modeSequence.back(), 1);
}