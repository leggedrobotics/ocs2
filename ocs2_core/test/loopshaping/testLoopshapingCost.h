

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <experimental/filesystem>

#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"
#include "ocs2_core/loopshaping/cost/LoopshapingCost.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template <class CONFIG>
class TestFixtureLoopShapingCost : public ::testing::Test {
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
    matrix_t Q, Q_final, R, P;
    Q.setRandom(CONFIG::SYSTEM_STATE_DIM, CONFIG::SYSTEM_STATE_DIM);
    Q_final.setRandom(CONFIG::SYSTEM_STATE_DIM, CONFIG::SYSTEM_STATE_DIM);
    R.setRandom(CONFIG::SYSTEM_INPUT_DIM, CONFIG::SYSTEM_INPUT_DIM);
    P.setRandom(CONFIG::SYSTEM_INPUT_DIM, CONFIG::SYSTEM_STATE_DIM);

    // Make symmetric
    Q_final = (0.5 * Q_final.transpose() + 0.5 * Q_final).eval();
    Q = (0.5 * Q.transpose() + 0.5 * Q).eval();
    R = (0.5 * R.transpose() + 0.5 * R).eval();
    testSystemCost.reset(new QuadraticCostFunction(Q, R, Q_final, P));

    targetTrajectories_ = TargetTrajectories({0.0}, {x_sys_}, {u_sys_});
    testSystemCost->setTargetTrajectoriesPtr(&targetTrajectories_);

    // Create Loopshaping costs
    testLoopshapingCost = LoopshapingCost::create(*testSystemCost, loopshapingDefinition_);
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<QuadraticCostFunction> testSystemCost;
  std::unique_ptr<LoopshapingCost> testLoopshapingCost;

  const scalar_t tol = 1e-9;

  scalar_t t;
  vector_t x_{CONFIG::FULL_STATE_DIM};
  vector_t u_{CONFIG::FULL_INPUT_DIM};
  vector_t x_sys_{CONFIG::SYSTEM_STATE_DIM};
  vector_t u_sys_{CONFIG::SYSTEM_INPUT_DIM};
  vector_t x_filter_{CONFIG::FILTER_STATE_DIM};
  vector_t u_filter_{CONFIG::FILTER_INPUT_DIM};
  TargetTrajectories targetTrajectories_;

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
