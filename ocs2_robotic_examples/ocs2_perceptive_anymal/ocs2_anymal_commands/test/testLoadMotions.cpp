//
// Created by rgrandia on 05.10.21.
//

#include <gtest/gtest.h>

#include <experimental/filesystem>

#include "ocs2_anymal_commands/LoadMotions.h"

using namespace switched_model;

inline std::string getAbsolutePathToConfigurationFile(const std::string& fileName) {
  const std::experimental::filesystem::path pathToTest = std::experimental::filesystem::path(__FILE__);
  return std::string(pathToTest.parent_path()) + "/" + fileName;
}

TEST(testLoadMotions, loadcsv) {
  const auto file = getAbsolutePathToConfigurationFile("data/testCsv.txt");
  const auto result = readCsv(file);
  const auto& header = result.header;
  ASSERT_EQ(header[0], "t");
  ASSERT_EQ(header[1], "x0");
  ASSERT_EQ(header[2], "x1");

  const auto& data = result.data;
  ASSERT_EQ(data.size(), 2);
  ASSERT_EQ(data[0].size(), 3);
  ASSERT_EQ(data[1].size(), 3);
  ASSERT_DOUBLE_EQ(data[0][0], 0.0);
  ASSERT_DOUBLE_EQ(data[0][1], 1.0);
  ASSERT_DOUBLE_EQ(data[0][2], 2.0);
  ASSERT_DOUBLE_EQ(data[1][0], 1.0);
  ASSERT_DOUBLE_EQ(data[1][1], 2.0);
  ASSERT_DOUBLE_EQ(data[1][2], 3.0);
}

TEST(testLoadMotions, loadmotion) {
  const auto file = getAbsolutePathToConfigurationFile("data/testMotion.txt");
  const auto csvData = readCsv(file);
  const auto result = readMotion(csvData);
  const auto& gait = result.second;

  ASSERT_TRUE(isValidGait(gait));
}

TEST(testLoadMotions, loadAnimatedmotion) {
  const auto file = getAbsolutePathToConfigurationFile("data/animatedMotion.txt");
  const auto csvData = readCsv(file);
  const auto result = readMotion(csvData);
  const auto& gait = result.second;

  ASSERT_TRUE(isValidGait(gait));
}

TEST(testLoadMotions, loadAnimatedCartesianMotion) {
  const auto file = getAbsolutePathToConfigurationFile("data/cartesianMotion.txt");
  const auto csvData = readCsv(file);
  const auto result = readCartesianMotion(csvData);
  const auto& gait = result.second;

  ASSERT_TRUE(isValidGait(gait));
}
