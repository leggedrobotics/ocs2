//
// Created by rgrandia on 10.04.19.
//

#pragma once

#include <gtest/gtest.h>

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

// Add configurations here:
typedef ::testing::Types<
    TestConfiguration_r_filter,
    TestConfiguration_r_simple_filter,
    TestConfiguration_s_filter,
    TestConfiguration_s_simple_filter>
    FilterConfigurations;

} // namespace ocs2

