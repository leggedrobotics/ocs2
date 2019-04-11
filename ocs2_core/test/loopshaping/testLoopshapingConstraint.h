//
// Created by rgrandia on 11.04.19.
//

#pragma once

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <experimental/filesystem>

#include "ocs2_core/constraint/LinearConstraint.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingConstraint.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template<class CONFIG>
class TestFixtureLoopShapingConstraint : public ::testing::Test {
 protected:
  static constexpr size_t FULL_STATE_DIM = CONFIG::FULL_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = CONFIG::FULL_INPUT_DIM;
  static constexpr size_t SYSTEM_STATE_DIM = CONFIG::SYSTEM_STATE_DIM;
  static constexpr size_t SYSTEM_INPUT_DIM = CONFIG::SYSTEM_INPUT_DIM;
  static constexpr size_t FILTER_STATE_DIM = CONFIG::FILTER_STATE_DIM;
  static constexpr size_t FILTER_INPUT_DIM = CONFIG::FILTER_INPUT_DIM;

  using TestSystemConstraint = LinearConstraint<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using TestLoopshapingConstraint = LoopshapingConstraint<FULL_STATE_DIM,
                                                          FULL_INPUT_DIM,
                                                          SYSTEM_STATE_DIM,
                                                          SYSTEM_INPUT_DIM,
                                                          FILTER_STATE_DIM,
                                                          FILTER_INPUT_DIM>;

  using scalar_t = typename TestLoopshapingConstraint::scalar_t;
  using state_vector_t = typename TestLoopshapingConstraint::state_vector_t;
  using input_vector_t = typename TestLoopshapingConstraint::input_vector_t;
  using state_matrix_t = typename TestLoopshapingConstraint::state_matrix_t;
  using input_matrix_t = typename TestLoopshapingConstraint::input_matrix_t;
  using state_input_matrix_t = typename TestLoopshapingConstraint::state_input_matrix_t;
  using input_state_matrix_t = typename TestLoopshapingConstraint::input_state_matrix_t;
  using system_state_vector_t = typename TestSystemConstraint::state_vector_t;
  using system_input_vector_t = typename TestSystemConstraint::input_vector_t;
  using system_state_matrix_t = typename TestSystemConstraint::state_matrix_t;
  using system_input_matrix_t = typename TestSystemConstraint::input_matrix_t;
  using system_input_state_matrix_t = typename TestSystemConstraint::input_state_matrix_t;
  using system_state_input_matrix_t = typename TestSystemConstraint::state_input_matrix_t;
  using filter_state_vector_t = typename TestLoopshapingConstraint::filter_state_vector_t;
  using filter_input_vector_t = typename TestLoopshapingConstraint::filter_input_vector_t;

  using system_constraint1_vector_t = typename TestSystemConstraint::constraint1_vector_t;
  using system_constraint1_state_matrix_t = typename TestSystemConstraint::constraint1_state_matrix_t;
  using system_constraint1_input_matrix_t = typename TestSystemConstraint::constraint1_input_matrix_t;
  using system_constraint2_vector_t = typename TestSystemConstraint::constraint2_vector_t;
  using system_constraint2_state_matrix_t = typename TestSystemConstraint::constraint2_state_matrix_t;

  using constraint1_vector_t = typename TestLoopshapingConstraint::constraint1_vector_t;
  using constraint1_state_matrix_t = typename TestLoopshapingConstraint::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename TestLoopshapingConstraint::constraint1_input_matrix_t;
  using constraint2_vector_t = typename TestLoopshapingConstraint::constraint2_vector_t;
  using constraint2_state_matrix_t = typename TestLoopshapingConstraint::constraint2_state_matrix_t;

  void SetUp() override {
    const std::experimental::filesystem::path pathToTest = std::experimental::filesystem::path(__FILE__);
    const std::string settingsFile = std::string(pathToTest.parent_path()) + "/" + CONFIG::fileName;

    // Load loopshaping definition
    loopshapingDefinition_.reset(new LoopshapingDefinition());
    loopshapingDefinition_->loadSettings(settingsFile);

    // Set up state and input
    t = 0.5;
    getRandomStateInput(x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);

    // Create system costs
    size_t numStateInputConstraint = SYSTEM_INPUT_DIM - 1;
    system_constraint1_vector_t e;
    system_constraint1_state_matrix_t C;
    system_constraint1_input_matrix_t D;
    e.setRandom();
    C.setRandom();
    D.setRandom();
    size_t numStateOnlyConstraint = SYSTEM_STATE_DIM - 1;
    system_constraint2_vector_t h;
    system_constraint2_state_matrix_t F;
    h.setRandom();
    F.setRandom();
    size_t numStateOnlyFinalConstraint = SYSTEM_STATE_DIM - 1;
    system_constraint2_vector_t h_f;
    system_constraint2_state_matrix_t F_f;
    h_f.setRandom();
    F_f.setRandom();

    // Make system Constraint
    testSystemConstraint.reset(new TestSystemConstraint(numStateInputConstraint,
                                                        e,
                                                        C,
                                                        D,
                                                        numStateOnlyConstraint,
                                                        h,
                                                        F,
                                                        numStateOnlyFinalConstraint,
                                                        h_f,
                                                        F_f));

    // Create Loopshaping constraint
    testLoopshapingConstraint.reset(new TestLoopshapingConstraint(*testSystemConstraint, loopshapingDefinition_));
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<TestSystemConstraint> testSystemConstraint;
  std::unique_ptr<TestLoopshapingConstraint> testLoopshapingConstraint;

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

} // namespace ocs2