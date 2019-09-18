

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <experimental/filesystem>

#include "ocs2_core/dynamics/LinearSystemDynamics.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h"
#include "ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivative.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template <class CONFIG>
class TestFixtureLoopShapingDynamics : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 protected:
  static constexpr size_t FULL_STATE_DIM = CONFIG::FULL_STATE_DIM;
  static constexpr size_t FULL_INPUT_DIM = CONFIG::FULL_INPUT_DIM;
  static constexpr size_t SYSTEM_STATE_DIM = CONFIG::SYSTEM_STATE_DIM;
  static constexpr size_t SYSTEM_INPUT_DIM = CONFIG::SYSTEM_INPUT_DIM;
  static constexpr size_t FILTER_STATE_DIM = CONFIG::FILTER_STATE_DIM;
  static constexpr size_t FILTER_INPUT_DIM = CONFIG::FILTER_INPUT_DIM;

  using TestSystem = LinearSystemDynamics<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using TestLoopshapingDynamics = LoopshapingDynamics<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using TestLoopshapingDynamicsDerivative = LoopshapingDynamicsDerivative<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using state_vector_t = typename TestLoopshapingDynamics::state_vector_t;
  using input_vector_t = typename TestLoopshapingDynamics::input_vector_t;
  using state_matrix_t = typename TestLoopshapingDynamicsDerivative::state_matrix_t;
  using state_input_matrix_t = typename TestLoopshapingDynamicsDerivative::state_input_matrix_t;
  using system_state_vector_t = typename TestSystem::state_vector_t;
  using system_input_vector_t = typename TestSystem::input_vector_t;
  using system_state_matrix_t = typename TestSystem::state_matrix_t;
  using system_state_input_matrix_t = typename TestSystem::state_input_matrix_t;
  using filter_state_vector_t = typename TestLoopshapingDynamics::filter_state_vector_t;
  using filter_input_vector_t = typename TestLoopshapingDynamics::filter_input_vector_t;

  void SetUp() override {

    // Load loopshaping definition
    const std::string settingsFile = getAbsolutePathToConfigurationFile(CONFIG::fileName);
    loopshapingDefinition_ = loopshaping_property_tree::load(settingsFile);

    // Create system dynamics
    system_state_matrix_t A, G;
    system_state_input_matrix_t B, H;
    A.setRandom();
    G.setRandom();
    B.setRandom();
    H.setRandom();
    testSystem.reset(new TestSystem(A, B, G, H));

    // Create Loopshaping Dynamics
    testLoopshapingDynamics = TestLoopshapingDynamics::create(*testSystem, loopshapingDefinition_);

    // Create Loopshaping Derivatives
    testLoopshapingDynamicsDerivative = TestLoopshapingDynamicsDerivative::create(*testSystem, loopshapingDefinition_);

    // Set up state and input
    t = 0.5;
    const double eps = 1e-2;
    getRandomStateInput(x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);
    getRandomStateInput(x_sys_disturbance_,
                        u_sys_disturbance_,
                        x_filter_disturbance_,
                        u_filter_disturbance_,
                        x_disturbance_,
                        u_disturbance_,
                        eps);
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<TestSystem> testSystem;
  std::unique_ptr<TestLoopshapingDynamics> testLoopshapingDynamics;
  std::unique_ptr<TestLoopshapingDynamicsDerivative> testLoopshapingDynamicsDerivative;

  const double tol = 1e-9;

  double t;
  state_vector_t x_;
  input_vector_t u_;
  system_state_vector_t x_sys_;
  system_input_vector_t u_sys_;
  filter_state_vector_t x_filter_;
  filter_input_vector_t u_filter_;

  state_vector_t x_disturbance_;
  input_vector_t u_disturbance_;
  system_state_vector_t x_sys_disturbance_;
  system_input_vector_t u_sys_disturbance_;
  filter_state_vector_t x_filter_disturbance_;
  filter_input_vector_t u_filter_disturbance_;

  void getRandomStateInput(system_state_vector_t &x_sys,
                           system_input_vector_t &u_sys,
                           filter_state_vector_t &x_filter,
                           filter_input_vector_t &u_filter,
                           state_vector_t &x,
                           input_vector_t &u,
                           double range = 1.0) {
    // Set random state
    x.setRandom();
    u.setRandom();

    // Scale the randomness
    x *= range;
    u *= range;

    // Retreive system and filter state
    loopshapingDefinition_->getSystemState(x, x_sys);
    loopshapingDefinition_->getSystemInput(x, u, u_sys);
    loopshapingDefinition_->getFilterState(x, x_filter);
    loopshapingDefinition_->getFilteredInput(x, u, u_filter);
  }

};

}; // namespace ocs2