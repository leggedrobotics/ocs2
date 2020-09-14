

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "ocs2_core/constraint/LinearConstraint.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraint.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template <class CONFIG>
class TestFixtureLoopShapingConstraint : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string settingsFile = getAbsolutePathToConfigurationFile(CONFIG::fileName);
    loopshapingDefinition_ = loopshaping_property_tree::load(settingsFile);

    // Set up state and input
    t = 0.5;
    const scalar_t eps = 1e-2;
    getRandomStateInput(x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);
    getRandomStateInput(x_sys_disturbance_, u_sys_disturbance_, x_filter_disturbance_, u_filter_disturbance_, x_disturbance_,
                        u_disturbance_, eps);

    // Create system costs
    size_t numStateInputConstraint = CONFIG::SYSTEM_INPUT_DIM - 1;
    vector_t e;
    matrix_t C;
    matrix_t D;
    e.setRandom(numStateInputConstraint);
    C.setRandom(numStateInputConstraint, CONFIG::SYSTEM_STATE_DIM);
    D.setRandom(numStateInputConstraint, CONFIG::SYSTEM_INPUT_DIM);

    size_t numStateOnlyConstraint = CONFIG::SYSTEM_INPUT_DIM - 1;  // max number of state constraints is system_input_dim
    vector_t h;
    matrix_t F;
    h.setRandom(numStateOnlyConstraint);
    F.setRandom(numStateOnlyConstraint, CONFIG::SYSTEM_STATE_DIM);

    size_t numStateOnlyFinalConstraint = CONFIG::SYSTEM_INPUT_DIM - 1;
    vector_t h_f;
    matrix_t F_f;
    h_f.setRandom(numStateOnlyFinalConstraint);
    F_f.setRandom(numStateOnlyFinalConstraint, CONFIG::SYSTEM_STATE_DIM);

    size_t numInequalityConstraint = 10;
    vector_t h0(numInequalityConstraint);
    matrix_t dhdx = matrix_t::Random(numInequalityConstraint, CONFIG::SYSTEM_STATE_DIM);
    matrix_t dhdu = matrix_t::Random(numInequalityConstraint, CONFIG::SYSTEM_INPUT_DIM);
    matrix_array_t dhdxx;
    matrix_array_t dhduu;
    matrix_array_t dhdux;
    for (size_t i = 0; i < numInequalityConstraint; i++) {
      h0(i) = i;
      dhdxx.push_back(matrix_t::Random(CONFIG::SYSTEM_STATE_DIM, CONFIG::SYSTEM_STATE_DIM));
      dhduu.push_back(matrix_t::Random(CONFIG::SYSTEM_INPUT_DIM, CONFIG::SYSTEM_INPUT_DIM));
      dhdux.push_back(matrix_t::Random(CONFIG::SYSTEM_INPUT_DIM, CONFIG::SYSTEM_STATE_DIM));
      // Make symmetric
      dhdxx[i] = (dhdxx[i].transpose() * dhdxx[i]).eval();
      dhduu[i] = (dhduu[i].transpose() * dhduu[i]).eval();
    }

    // Make system Constraint
    testSystemConstraint.reset(new LinearConstraint(e, C, D, h, F, h_f, F_f, h0, dhdx, dhdu, dhdxx, dhduu, dhdux));

    // Create Loopshaping constraint
    testLoopshapingConstraint = LoopshapingConstraint::create(*testSystemConstraint, loopshapingDefinition_);
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<LinearConstraint> testSystemConstraint;
  std::unique_ptr<LoopshapingConstraint> testLoopshapingConstraint;

  const scalar_t tol = 1e-9;

  scalar_t t;
  vector_t x_{CONFIG::FULL_STATE_DIM};
  vector_t u_{CONFIG::FULL_INPUT_DIM};
  vector_t x_sys_{CONFIG::SYSTEM_STATE_DIM};
  vector_t u_sys_{CONFIG::SYSTEM_INPUT_DIM};
  vector_t x_filter_{CONFIG::FILTER_STATE_DIM};
  vector_t u_filter_{CONFIG::FILTER_INPUT_DIM};

  vector_t x_disturbance_{CONFIG::FULL_STATE_DIM};
  vector_t u_disturbance_{CONFIG::FULL_INPUT_DIM};
  vector_t x_sys_disturbance_{CONFIG::SYSTEM_STATE_DIM};
  vector_t u_sys_disturbance_{CONFIG::SYSTEM_INPUT_DIM};
  vector_t x_filter_disturbance_{CONFIG::FILTER_STATE_DIM};
  vector_t u_filter_disturbance_{CONFIG::FILTER_INPUT_DIM};

  void getRandomStateInput(vector_t& x_sys, vector_t& u_sys, vector_t& x_filter, vector_t& u_filter, vector_t& x, vector_t& u,
                           scalar_t range = 1.0) {
    // Set random state
    x.setRandom(CONFIG::FULL_STATE_DIM);
    u.setRandom(CONFIG::FULL_INPUT_DIM);

    // Scale the randomness
    x *= range;
    u *= range;

    // Retreive system and filter state
    x_sys = loopshapingDefinition_->getSystemState(x);
    u_sys = loopshapingDefinition_->getSystemInput(x, u);
    x_filter = loopshapingDefinition_->getFilterState(x);
    u_filter = loopshapingDefinition_->getFilteredInput(x, u);
  }
};

}  // namespace ocs2