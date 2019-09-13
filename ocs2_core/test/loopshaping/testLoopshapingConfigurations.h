

#pragma once

#include <gtest/gtest.h>
#include <experimental/filesystem>

namespace ocs2 {

struct TestConfiguration_r_filter {
  static constexpr size_t SYSTEM_STATE_DIM = 2;
  static constexpr size_t SYSTEM_INPUT_DIM = 3;
  static constexpr size_t FILTER_STATE_DIM = 4;
  static constexpr size_t FILTER_INPUT_DIM = 3;
  static constexpr size_t FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = SYSTEM_INPUT_DIM;
  static const std::string fileName;
};
const std::string TestConfiguration_r_filter::fileName = "loopshaping_r.conf";

struct TestConfiguration_r_simple_filter {
  static constexpr size_t SYSTEM_STATE_DIM = 2;
  static constexpr size_t SYSTEM_INPUT_DIM = 3;
  static constexpr size_t FILTER_STATE_DIM = 0;
  static constexpr size_t FILTER_INPUT_DIM = 3;
  static constexpr size_t FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = SYSTEM_INPUT_DIM;
  static const std::string fileName;
};
const std::string TestConfiguration_r_simple_filter::fileName = "loopshaping_r_simple.conf";

struct TestConfiguration_r_ballbot_filter {
  static constexpr size_t SYSTEM_STATE_DIM = 16;
  static constexpr size_t SYSTEM_INPUT_DIM = 6;
  static constexpr size_t FILTER_STATE_DIM = 3;
  static constexpr size_t FILTER_INPUT_DIM = 6;
  static constexpr size_t FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = SYSTEM_INPUT_DIM;
  static const std::string fileName;
};
const std::string TestConfiguration_r_ballbot_filter::fileName = "loopshaping_r_ballbot.conf";

struct TestConfiguration_s_filter {
  static constexpr size_t SYSTEM_STATE_DIM = 2;
  static constexpr size_t SYSTEM_INPUT_DIM = 3;
  static constexpr size_t FILTER_STATE_DIM = 4;
  static constexpr size_t FILTER_INPUT_DIM = 3;
  static constexpr size_t FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = SYSTEM_INPUT_DIM + FILTER_INPUT_DIM;
  static const std::string fileName;
};
const std::string TestConfiguration_s_filter::fileName = "loopshaping_s.conf";

struct TestConfiguration_s_simple_filter {
  static constexpr size_t SYSTEM_STATE_DIM = 2;
  static constexpr size_t SYSTEM_INPUT_DIM = 3;
  static constexpr size_t FILTER_STATE_DIM = 0;
  static constexpr size_t FILTER_INPUT_DIM = 3;
  static constexpr size_t FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = SYSTEM_INPUT_DIM + FILTER_INPUT_DIM;
  static const std::string fileName;
};
const std::string TestConfiguration_s_simple_filter::fileName = "loopshaping_s_simple.conf";

struct TestConfiguration_s_eliminate_filter {
  static constexpr size_t SYSTEM_STATE_DIM = 2;
  static constexpr size_t SYSTEM_INPUT_DIM = 3;
  static constexpr size_t FILTER_STATE_DIM = 4;
  static constexpr size_t FILTER_INPUT_DIM = 3;
  static constexpr size_t FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = FILTER_INPUT_DIM;
  static const std::string fileName;
};
const std::string TestConfiguration_s_eliminate_filter::fileName = "loopshaping_s_eliminate.conf";

struct TestConfiguration_s_simple_eliminate_filter {
  static constexpr size_t SYSTEM_STATE_DIM = 2;
  static constexpr size_t SYSTEM_INPUT_DIM = 3;
  static constexpr size_t FILTER_STATE_DIM = 0;
  static constexpr size_t FILTER_INPUT_DIM = 3;
  static constexpr size_t FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = FILTER_INPUT_DIM;
  static const std::string fileName;
};
const std::string TestConfiguration_s_simple_eliminate_filter::fileName = "loopshaping_s_simple_eliminate.conf";

// Add configurations here:
typedef ::testing::Types<
    TestConfiguration_r_filter,
    TestConfiguration_r_simple_filter,
    TestConfiguration_r_ballbot_filter,
    TestConfiguration_s_filter,
    TestConfiguration_s_simple_filter,
    TestConfiguration_s_eliminate_filter,
    TestConfiguration_s_simple_eliminate_filter>
    FilterConfigurations;

std::string getAbsolutePathToConfigurationFile(const std::string& fileName){
  const std::experimental::filesystem::path pathToTest = std::experimental::filesystem::path(__FILE__);
  return std::string(pathToTest.parent_path()) + "/" + fileName;
}



} // namespace ocs2

