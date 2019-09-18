

#pragma once

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "ocs2_core/constraint/LinearConstraint.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraint.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template<class CONFIG>
class TestFixtureLoopShapingConstraint : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  using system_scalar_array_t = typename TestSystemConstraint::scalar_array_t;
  using system_state_vector_array_t = typename TestSystemConstraint::state_vector_array_t;
  using system_input_vector_array_t = typename TestSystemConstraint::input_vector_array_t;
  using system_state_matrix_array_t = typename TestSystemConstraint::state_matrix_array_t;
  using system_input_matrix_array_t = typename TestSystemConstraint::input_matrix_array_t;
  using system_input_state_matrix_array_t = typename TestSystemConstraint::input_state_matrix_array_t;

  using constraint1_vector_t = typename TestLoopshapingConstraint::constraint1_vector_t;
  using constraint1_state_matrix_t = typename TestLoopshapingConstraint::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename TestLoopshapingConstraint::constraint1_input_matrix_t;
  using constraint2_vector_t = typename TestLoopshapingConstraint::constraint2_vector_t;
  using constraint2_state_matrix_t = typename TestLoopshapingConstraint::constraint2_state_matrix_t;
  using scalar_array_t = typename TestLoopshapingConstraint::scalar_array_t;
  using state_vector_array_t = typename TestLoopshapingConstraint::state_vector_array_t;
  using input_vector_array_t = typename TestLoopshapingConstraint::input_vector_array_t;
  using state_matrix_array_t = typename TestLoopshapingConstraint::state_matrix_array_t;
  using input_matrix_array_t = typename TestLoopshapingConstraint::input_matrix_array_t;
  using input_state_matrix_array_t = typename TestLoopshapingConstraint::input_state_matrix_array_t;

  void SetUp() override {
    const std::string settingsFile = getAbsolutePathToConfigurationFile(CONFIG::fileName);
    loopshapingDefinition_ = loopshaping_property_tree::load(settingsFile);

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

    // Create system costs
    size_t numStateInputConstraint = SYSTEM_INPUT_DIM - 1;
    system_constraint1_vector_t e;
    system_constraint1_state_matrix_t C;
    system_constraint1_input_matrix_t D;
    e.setRandom();
    C.setRandom();
    D.setRandom();

    size_t numStateOnlyConstraint = SYSTEM_INPUT_DIM - 1; // max number of state constraints is system_input_dim
    system_constraint2_vector_t h;
    system_constraint2_state_matrix_t F;
    h.setRandom();
    F.setRandom();

    size_t numStateOnlyFinalConstraint = SYSTEM_INPUT_DIM - 1;
    system_constraint2_vector_t h_f;
    system_constraint2_state_matrix_t F_f;
    h_f.setRandom();
    F_f.setRandom();

    size_t numInequalityConstraint = 10;
    system_scalar_array_t h0;
    system_state_vector_array_t dhdx;
    system_input_vector_array_t dhdu;
    system_state_matrix_array_t ddhdxdx;
    system_input_matrix_array_t ddhdudu;
    system_input_state_matrix_array_t ddhdudx;
    for (size_t i=0; i<numInequalityConstraint; i++){
      h0.push_back(i);
      dhdx.push_back(system_state_vector_t::Random());
      dhdu.push_back(system_input_vector_t::Random());
      ddhdxdx.push_back(system_state_matrix_t::Random());
      ddhdudu.push_back(system_input_matrix_t::Random());
      ddhdudx.push_back(system_input_state_matrix_t::Random());
      // Make symmetric
      ddhdxdx[i] = (ddhdxdx[i].transpose()*ddhdxdx[i]).eval();
      ddhdudu[i] = (ddhdudu[i].transpose()*ddhdudu[i]).eval();
    }

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
                                                        F_f,
                                                        numInequalityConstraint,
                                                        h0,
                                                        dhdx,
                                                        dhdu,
                                                        ddhdxdx,
                                                        ddhdudu,
                                                        ddhdudx));

    // Create Loopshaping constraint
    testLoopshapingConstraint = TestLoopshapingConstraint::create(*testSystemConstraint, loopshapingDefinition_);
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<TestSystemConstraint> testSystemConstraint;
  std::unique_ptr<TestLoopshapingConstraint> testLoopshapingConstraint;

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

} // namespace ocs2