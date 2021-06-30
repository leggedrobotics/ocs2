

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <experimental/filesystem>

#include "ocs2_core/cost/QuadraticStateCost.h"
#include "ocs2_core/cost/QuadraticStateInputCost.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/LoopshapingPreComputation.h"
#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"
#include "ocs2_core/loopshaping/cost/LoopshapingCost.h"

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

template <class CONFIG>
class TestFixtureLoopShapingCost : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string settingsFile = getAbsolutePathToConfigurationFile(CONFIG::fileName);
    loopshapingDefinition = loopshaping_property_tree::load(settingsFile);

    // Set up state and input
    t = 0.5;
    const scalar_t eps = 1e-2;
    getRandomStateInput(x_sys, u_sys, x_filter, u_filter, x, u);
    getRandomStateInput(x_sys_disturbance, u_sys_disturbance, x_filter_disturbance, u_filter_disturbance, x_disturbance, u_disturbance,
                        eps);

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
    systemCost.reset(new QuadraticStateInputCost(Q, R, P));
    systemStateCost.reset(new QuadraticStateCost(Q_final));

    costDesiredTrajectories = CostDesiredTrajectories({0.0}, {x_sys}, {u_sys});

    StateInputCostCollection systemCostCollection;
    StateCostCollection systemStateCostCollection;
    systemCostCollection.add("", std::unique_ptr<StateInputCost>(systemCost->clone()));
    systemStateCostCollection.add("", std::unique_ptr<StateCost>(systemStateCost->clone()));

    // Create Loopshaping cost collection wrappers
    loopshapingCost = LoopshapingCost::create(systemCostCollection, loopshapingDefinition);
    loopshapingStateCost = LoopshapingCost::create(systemStateCostCollection, loopshapingDefinition);

    preComputation.reset(new LoopshapingPreComputation(PreComputation(), loopshapingDefinition));
  };

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition;
  std::unique_ptr<LoopshapingPreComputation> preComputation;
  std::unique_ptr<StateInputCost> systemCost;
  std::unique_ptr<StateCost> systemStateCost;
  std::unique_ptr<StateInputCostCollection> loopshapingCost;
  std::unique_ptr<StateCostCollection> loopshapingStateCost;

  const scalar_t tol = 1e-9;

  scalar_t t;
  vector_t x{CONFIG::FULL_STATE_DIM};
  vector_t u{CONFIG::FULL_INPUT_DIM};
  vector_t x_sys{CONFIG::SYSTEM_STATE_DIM};
  vector_t u_sys{CONFIG::SYSTEM_INPUT_DIM};
  vector_t x_filter{CONFIG::FILTER_STATE_DIM};
  vector_t u_filter{CONFIG::FILTER_INPUT_DIM};
  CostDesiredTrajectories costDesiredTrajectories;

  vector_t x_disturbance{CONFIG::FULL_STATE_DIM};
  vector_t u_disturbance{CONFIG::FULL_INPUT_DIM};
  vector_t x_sys_disturbance{CONFIG::SYSTEM_STATE_DIM};
  vector_t u_sys_disturbance{CONFIG::SYSTEM_INPUT_DIM};
  vector_t x_filter_disturbance{CONFIG::FILTER_STATE_DIM};
  vector_t u_filter_disturbance{CONFIG::FILTER_INPUT_DIM};

  void getRandomStateInput(vector_t& x_sys, vector_t& u_sys, vector_t& x_filter, vector_t& u_filter, vector_t& x, vector_t& u,
                           scalar_t range = 1.0) {
    // Set random state
    x.setRandom(CONFIG::FULL_STATE_DIM);
    u.setRandom(CONFIG::FULL_INPUT_DIM);

    // Scale the randomness
    x *= range;
    u *= range;

    // Retreive system and filter state
    x_sys = loopshapingDefinition->getSystemState(x);
    u_sys = loopshapingDefinition->getSystemInput(x, u);
    x_filter = loopshapingDefinition->getFilterState(x);
    u_filter = loopshapingDefinition->getFilteredInput(x, u);
  }
};

};  // namespace ocs2
