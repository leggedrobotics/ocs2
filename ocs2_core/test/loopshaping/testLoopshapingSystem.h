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
#include "ocs2_core/loopshaping/LoopshapingDynamicsDerivative.h"

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
  using TestLoopshapingDynamicsDerivative = LoopshapingDynamicsDerivative<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using state_vector_t = TestLoopshapingDynamics::state_vector_t;
  using input_vector_t = TestLoopshapingDynamics::input_vector_t;
  using system_state_vector_t = TestSystem::state_vector_t;
  using system_input_vector_t = TestSystem::input_vector_t;
  using filter_state_vector_t = TestLoopshapingDynamics::filter_state_vector_t;
  using filter_input_vector_t = TestLoopshapingDynamics::filter_input_vector_t;

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

    // Create Loopshaping Derivatives
    testLoopshapingDynamicsDerivative.reset(new TestLoopshapingDynamicsDerivative(*testSystem, loopshapingDefinition_));

    // Set up state and input
    t = 0.5;
    getRandomStateInput(x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<TestSystem> testSystem;
  std::unique_ptr<TestLoopshapingDynamics> testLoopshapingDynamics;
  std::unique_ptr<TestLoopshapingDynamicsDerivative> testLoopshapingDynamicsDerivative;

  double t;
  state_vector_t x_;
  input_vector_t u_;
  system_state_vector_t x_sys_;
  system_input_vector_t u_sys_;
  filter_state_vector_t x_filter_;
  filter_input_vector_t u_filter_;

  void getRandomStateInput(system_state_vector_t &x_sys,
                           system_input_vector_t &u_sys,
                           filter_state_vector_t &x_filter,
                           filter_input_vector_t &u_filter,
                           state_vector_t &x,
                           input_vector_t &u,
                           double range = 1.0) {
    // Set random state
    x_sys.setRandom();
    u_sys.setRandom();
    x_filter.setRandom();
    u_filter.setRandom();

    // Scale the randomness
    x_sys *= range;
    u_sys *= range;
    x_filter *= range;
    u_filter *= range;

    // Get total state
    loopshapingDefinition_->concatenateSystemAndFilterState(x_sys, x_filter, x);
    loopshapingDefinition_->concatenateSystemAndFilterInput(u_sys, u_filter, u);
  }

};

}; // namespace ocs2