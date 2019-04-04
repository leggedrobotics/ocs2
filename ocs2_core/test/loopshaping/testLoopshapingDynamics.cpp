//
// Created by rgrandia on 04.04.19.
//

#include <gtest/gtest.h>
#include "testLoopshapingSystem.h"

using namespace ocs2;

TEST_F(TestLoopShapingDynamics_r_filter, evaluateDynamics) {
  // Evaluate system
  TestSystem::state_vector_t dx_sys;
  testSystem->computeFlowMap(t, x_sys, u_sys, dx_sys);

  // Evaluate loopshaping system
  TestLoopshapingDynamics::state_vector_t dx;
  testLoopshapingDynamics->computeFlowMap(t, x, u, dx);

  // System part of the flowmap should stay the same
  ASSERT_TRUE( dx_sys.isApprox( dx.segment(0, SYSTEM_STATE_DIM) ));
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}