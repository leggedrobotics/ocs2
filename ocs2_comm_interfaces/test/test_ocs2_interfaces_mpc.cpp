/*
 * test_ocs2_interfaces_mpc.cpp
 *
 *  Created on: April 5, 2019
 *      Author: Johannes
 */

#include <gtest/gtest.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>
#include <ocs2_mpc/MPC_SLQ.h>
#include "ocs2_core/logic/rules/NullLogicRules.h"

using namespace ocs2;

TEST(testOcs2InterfacesMpc, instantiation) {
  MPC_SLQ<1, 1> mpcSlq;
  NullLogicRules nullLogicRules;
  MPC_Interface<1, 1> mpcInterface(mpcSlq, nullLogicRules);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}