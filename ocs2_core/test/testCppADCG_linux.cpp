/*
 * testCppADCG_linux.cpp
 *
 *  Created on: Apr 24, 2018
 *      Author: farbod
 */

#include "cppad_cg/testCppADCG_Simple_linux.h"
#include "cppad_cg/testCppADCG_NN_linux.h"

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


