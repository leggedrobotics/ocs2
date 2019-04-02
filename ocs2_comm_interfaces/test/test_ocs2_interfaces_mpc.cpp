/*
 * testFindIndex.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: farbod
 */

#include <gtest/gtest.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>

using namespace ocs2;

TEST(testOcs2InterfacesMpc, instantiationDefaultContructor)
{
  MPC_Interface<1, 1> mpcInterface;
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}