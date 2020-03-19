//
// Created by rgrandia on 17.03.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"

using namespace switched_model;

TEST(TestFootPlanner, extractPhases_SinglePhase) {
  std::vector<double> eventTimes = {};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true};
  auto footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.front().type, FeetPlannerBase::FootPhaseType::Stance);

  // Single swing phase
  contactFlags = {false};
  footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.front().type, FeetPlannerBase::FootPhaseType::Swing);
}

TEST(TestFootPlanner, extractPhases_MergePhases) {
  std::vector<double> eventTimes = {0.5};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true, true};
  auto footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.size(), 1);
  ASSERT_EQ(footPhases.front().type, FeetPlannerBase::FootPhaseType::Stance);

  // Single swing phase
  contactFlags = {false, false};
  footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.size(), 1);
  ASSERT_EQ(footPhases.front().type, FeetPlannerBase::FootPhaseType::Swing);
}

TEST(TestFootPlanner, extractPhases_timing) {
  std::vector<double> eventTimes;
  std::vector<bool> contactFlags;

  // Single phase
  eventTimes = {};
  contactFlags = {true};
  auto footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  ASSERT_TRUE(std::isnan(footPhases.front().startTime));
  ASSERT_TRUE(std::isnan(footPhases.front().endTime));

  // Switched phase
  eventTimes = {0.5};
  contactFlags = {false, true};
  footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  ASSERT_TRUE(std::isnan(footPhases.front().startTime));
  ASSERT_DOUBLE_EQ(footPhases.front().endTime, eventTimes.front());
  ASSERT_DOUBLE_EQ(footPhases.back().startTime, eventTimes.front());
  ASSERT_TRUE(std::isnan(footPhases.back().endTime));

  // Switched phase
  eventTimes = {1.0, 2.0, 3.0, 4.0};
  contactFlags = {true, true, false, false, true};
  footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.size(), 3);
  // Phase 0
  ASSERT_TRUE(std::isnan(footPhases[0].startTime));
  ASSERT_DOUBLE_EQ(footPhases[0].endTime, eventTimes[1]);
  // Phase 1
  ASSERT_DOUBLE_EQ(footPhases[1].startTime, eventTimes[1]);
  ASSERT_DOUBLE_EQ(footPhases[1].endTime, eventTimes[3]);
  // Phase 2
  ASSERT_DOUBLE_EQ(footPhases[2].startTime, eventTimes[3]);
  ASSERT_TRUE(std::isnan(footPhases.back().endTime));
}

TEST(TestFootPlanner, filterOutShortSwingPhases) {
  // stance, short swing, short stance, long swing, stance
  std::vector<double> eventTimes = {0.5, 0.6, 0.7, 1.0};
  std::vector<bool> contactFlags = {true, false, true, false, true};

  // Short and long swing phase
  auto footPhases = FeetPlannerBase::extractFootPhases(eventTimes, contactFlags);
  auto filteredPhases = FeetPlannerBase::filterOutShortSwingPhases(footPhases, 0.2);

  // Should filter out only the short swing phase
  ASSERT_EQ(filteredPhases.size(), 3);
  ASSERT_EQ(filteredPhases[0].type, FeetPlannerBase::FootPhaseType::Stance);
  ASSERT_EQ(filteredPhases[1].type, FeetPlannerBase::FootPhaseType::Swing);
  ASSERT_EQ(filteredPhases[2].type, FeetPlannerBase::FootPhaseType::Stance);

  ASSERT_DOUBLE_EQ(filteredPhases[0].endTime, 0.7);
  ASSERT_DOUBLE_EQ(filteredPhases[1].startTime, 0.7);
  ASSERT_DOUBLE_EQ(filteredPhases[1].endTime, 1.0);
  ASSERT_DOUBLE_EQ(filteredPhases[2].startTime, 1.0);
}
