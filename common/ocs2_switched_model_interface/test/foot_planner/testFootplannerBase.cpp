//
// Created by rgrandia on 17.03.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

using namespace switched_model;

TEST(TestFootPlanner, extractPhases_SinglePhase) {
  std::vector<double> eventTimes = {};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true};
  auto footPhases = SwingTrajectoryPlanner::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.front().type, SwingTrajectoryPlanner::FootPhaseType::Stance);

  // Single swing phase
  contactFlags = {false};
  footPhases = SwingTrajectoryPlanner::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.front().type, SwingTrajectoryPlanner::FootPhaseType::Swing);
}

TEST(TestFootPlanner, extractPhases_MergePhases) {
  std::vector<double> eventTimes = {0.5};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true, true};
  auto footPhases = SwingTrajectoryPlanner::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.size(), 1);
  ASSERT_EQ(footPhases.front().type, SwingTrajectoryPlanner::FootPhaseType::Stance);

  // Single swing phase
  contactFlags = {false, false};
  footPhases = SwingTrajectoryPlanner::extractFootPhases(eventTimes, contactFlags);
  ASSERT_EQ(footPhases.size(), 1);
  ASSERT_EQ(footPhases.front().type, SwingTrajectoryPlanner::FootPhaseType::Swing);
}

TEST(TestFootPlanner, extractPhases_timing) {
  std::vector<double> eventTimes;
  std::vector<bool> contactFlags;

  // Single phase
  eventTimes = {};
  contactFlags = {true};
  auto footPhases = SwingTrajectoryPlanner::extractFootPhases(eventTimes, contactFlags);
  ASSERT_TRUE(std::isnan(footPhases.front().startTime));
  ASSERT_TRUE(std::isnan(footPhases.front().endTime));

  // Switched phase
  eventTimes = {0.5};
  contactFlags = {false, true};
  footPhases = SwingTrajectoryPlanner::extractFootPhases(eventTimes, contactFlags);
  ASSERT_TRUE(std::isnan(footPhases.front().startTime));
  ASSERT_DOUBLE_EQ(footPhases.front().endTime, eventTimes.front());
  ASSERT_DOUBLE_EQ(footPhases.back().startTime, eventTimes.front());
  ASSERT_TRUE(std::isnan(footPhases.back().endTime));

  // Switched phase
  eventTimes = {1.0, 2.0, 3.0, 4.0};
  contactFlags = {true, true, false, false, true};
  footPhases = SwingTrajectoryPlanner::extractFootPhases(eventTimes, contactFlags);
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
