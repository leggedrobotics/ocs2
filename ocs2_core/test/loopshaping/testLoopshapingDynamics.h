

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <experimental/filesystem>

#include "ocs2_core/dynamics/LinearSystemDynamics.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"
#include "ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template <class CONFIG>
class TestFixtureLoopShapingDynamics : public ::testing::Test {
 protected:
  void SetUp() override {
    // Load loopshaping definition
    const std::string settingsFile = getAbsolutePathToConfigurationFile(CONFIG::fileName);
    loopshapingDefinition_ = loopshaping_property_tree::load(settingsFile);

    // Create system dynamics
    matrix_t A, B, G;
    A.setRandom(CONFIG::SYSTEM_STATE_DIM, CONFIG::SYSTEM_STATE_DIM);
    B.setRandom(CONFIG::SYSTEM_STATE_DIM, CONFIG::SYSTEM_INPUT_DIM);
    G.setRandom(CONFIG::SYSTEM_STATE_DIM, CONFIG::SYSTEM_STATE_DIM);
    testSystem.reset(new LinearSystemDynamics(A, B, G));

    // Create Loopshaping Dynamics
    testLoopshapingDynamics = LoopshapingDynamics::create(*testSystem, loopshapingDefinition_);

    // Set up state and input
    t = 0.5;
    const scalar_t eps = 1e-2;
    getRandomStateInput(x_sys_, u_sys_, x_filter_, u_filter_, x_, u_);
    getRandomStateInput(x_sys_disturbance_, u_sys_disturbance_, x_filter_disturbance_, u_filter_disturbance_, x_disturbance_,
                        u_disturbance_, eps);

    preComp_sys_.reset(new PreComputation);
    preComp_.reset(new LoopshapingPreComputation(*preComp_sys_, loopshapingDefinition_));
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<LinearSystemDynamics> testSystem;
  std::unique_ptr<LoopshapingDynamics> testLoopshapingDynamics;
  std::unique_ptr<PreComputation> preComp_sys_;
  std::unique_ptr<LoopshapingPreComputation> preComp_;

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

};  // namespace ocs2
