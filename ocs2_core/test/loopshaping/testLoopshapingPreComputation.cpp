//
// Created by ruben on 02.11.18.
//

#include "testLoopshapingConfigurations.h"

using namespace ocs2;

class TestFixturePreComputation : LoopshapingTestConfiguration {
 public:
  TestFixturePreComputation(const std::string& configName) : LoopshapingTestConfiguration(configName) {}

  void testCachedStateInput() {
    const auto t = 0.0;
    const auto& x = x_;
    const auto& u = u_;

    const auto x_system = loopshapingDefinition_->getSystemState(x);
    const auto u_system = loopshapingDefinition_->getSystemInput(x, u);
    const auto x_filter = loopshapingDefinition_->getFilterState(x);
    const auto u_filter = loopshapingDefinition_->getFilteredInput(x, u);

    preComp_->request(Request::Cost, t, x, u);

    EXPECT_TRUE(x_system.isApprox(preComp_->getSystemState()));
    EXPECT_TRUE(u_system.isApprox(preComp_->getSystemInput()));
    EXPECT_TRUE(x_filter.isApprox(preComp_->getFilterState()));
    EXPECT_TRUE(u_filter.isApprox(preComp_->getFilteredInput()));
  }
};

TEST(testLoopshapingPreComputation, getCachedStateInput) {
  for (const auto config : configNames) {
    TestFixturePreComputation test(config);
    test.testCachedStateInput();
  }
}
