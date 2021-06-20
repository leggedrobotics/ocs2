//
// Created by ruben on 02.11.18.
//

#include <gtest/gtest.h>
#include <experimental/filesystem>

#include <ocs2_core/loopshaping/LoopshapingPreComputation.h>
#include <ocs2_core/loopshaping/LoopshapingPropertyTree.h>

#include "testLoopshapingConfigurations.h"

using namespace ocs2;

template <typename CONFIG>
class testLoopshapingPreComputation : public testing::Test {
 public:
  void SetUp() override {
    // Load loopshaping definition
    const std::string settingsFile = getAbsolutePathToConfigurationFile(CONFIG::fileName);
    loopshapingDefinition_ = loopshaping_property_tree::load(settingsFile);
  }
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  const vector_t x_ = vector_t::Random(CONFIG::FULL_STATE_DIM);
  const vector_t u_ = vector_t::Random(CONFIG::FULL_INPUT_DIM);
};

TYPED_TEST_SUITE(testLoopshapingPreComputation, FilterConfigurations);

TYPED_TEST(testLoopshapingPreComputation, getCachedStateInput) {
  LoopshapingPreComputation preComp(PreComputation(), this->loopshapingDefinition_);

  const auto t = 0.0;
  const auto& x = this->x_;
  const auto& u = this->u_;

  const auto x_system = this->loopshapingDefinition_->getSystemState(x);
  const auto u_system = this->loopshapingDefinition_->getSystemInput(x, u);
  const auto x_filter = this->loopshapingDefinition_->getFilterState(x);
  const auto u_filter = this->loopshapingDefinition_->getFilteredInput(x, u);

  preComp.request(Request::Cost, t, x, u);

  EXPECT_TRUE(x_system.isApprox(preComp.getSystemState()));
  EXPECT_TRUE(u_system.isApprox(preComp.getSystemInput()));
  EXPECT_TRUE(x_filter.isApprox(preComp.getFilterState()));
  EXPECT_TRUE(u_filter.isApprox(preComp.getFilteredInput()));
}
