//
// Created by rgrandia on 04.04.19.
//

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <experimental/filesystem>

#include "ocs2_core/dynamics/LinearSystemDynamics.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingDynamics.h"

namespace ocs2 {


class TestLoopShapingDynamics_r_filter : public ::testing::Test {
 protected:
  static constexpr size_t FULL_STATE_DIM = 6;
  static constexpr size_t FULL_INPUT_DIM = 3;
  static constexpr size_t SYSTEM_STATE_DIM = 2;
  static constexpr size_t SYSTEM_INPUT_DIM = 3;
  static constexpr size_t FILTER_STATE_DIM = 4;
  static constexpr size_t FILTER_INPUT_DIM = 3;

  using TestSystem = LinearSystemDynamics<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using TestLoopshapingDynamics = LoopshapingDynamics<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  void SetUp() override {
    // Select the loopshaping file
    const std::experimental::filesystem::path pathToTest(__FILE__);
    const std::string settingsFile = std::string(pathToTest.parent_path()) + "/loopshaping_r.conf";

    // Load loopshaping definition
    loopshapingDefinition_.reset(new LoopshapingDefinition());
    loopshapingDefinition_->loadSettings(settingsFile);

    // Create system dynamics
    TestSystem::state_matrix_t A, G;
    TestSystem::state_input_matrix_t B, H;
    A.setRandom();
    G.setRandom();
    B.setRandom();
    H.setRandom();
    testSystem.reset(new TestSystem(A, B, G, H));

    // Create Loopshaping Dynamics
    testLoopshapingDynamics.reset(new TestLoopshapingDynamics(*testSystem, loopshapingDefinition_));

    // Set up state and input
    setRandomTimeStateInput();
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<TestSystem> testSystem;
  std::unique_ptr<TestLoopshapingDynamics> testLoopshapingDynamics;

  double t;
  TestLoopshapingDynamics::state_vector_t x;
  TestLoopshapingDynamics::input_vector_t u;
  TestSystem::state_vector_t x_sys;
  TestSystem::input_vector_t u_sys;
  TestLoopshapingDynamics::filter_state_vector_t x_filter;
  TestLoopshapingDynamics::filter_input_vector_t u_filter;

  void setRandomTimeStateInput() {
    t = 0.5;
    x_sys.setRandom();
    u_sys.setRandom();
    x_filter.setRandom();
    u_filter.setRandom();
    loopshapingDefinition_->concatenateSystemAndFilterState(x_sys, x_filter, x);
    loopshapingDefinition_->concatenateSystemAndFilterInput(u_sys, u_filter, u);
  }
};

}; // namespace ocs2