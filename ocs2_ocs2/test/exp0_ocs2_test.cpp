/*
 * exp0_ocs2_test.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: farbod
 */

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <gtest/gtest.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
#include <ocs2_slq/test/EXP0.h>

#include <ocs2_ocs2/GSLQ.h>
#include "ocs2_ocs2/OCS2Projected.h"

using namespace ocs2;

enum
{
	STATE_DIM = 2,
	INPUT_DIM = 1
};

TEST(exp0_ocs2_test, exp0_ocs2_test)
{
	// system dynamics
	EXP0_System systemDynamics;

	// system derivatives
	EXP0_SystemDerivative systemDerivative;

	// system constraints
	EXP0_SystemConstraint systemConstraint;

	// system cost functions
	EXP0_CostFunction systemCostFunction;

	// system operatingTrajectories
	Eigen::Matrix<double,STATE_DIM,1> stateOperatingPoint = Eigen::Matrix<double,STATE_DIM,1>::Zero();
	Eigen::Matrix<double,INPUT_DIM,1> inputOperatingPoint = Eigen::Matrix<double,INPUT_DIM,1>::Zero();
	EXP0_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	SLQ_Settings slqSettings;
	slqSettings.displayInfo_ = false;
	slqSettings.displayShortSummary_ = true;
	slqSettings.absTolODE_ = 1e-10;
	slqSettings.relTolODE_ = 1e-7;
	slqSettings.maxNumStepsPerSecond_ = 10000;
	slqSettings.nThreads_ = 3;
	slqSettings.maxNumIterationsSLQ_ = 50;
	slqSettings.maxIterationGradientDescent_ = 5;
	slqSettings.warmStartGSLQ_ = false;
	slqSettings.useLQForDerivatives_ = false;
	slqSettings.displayGradientDescent_ = false;
	slqSettings.useLQForDerivatives_ = false;
	slqSettings.minLearningRateNLP_ = 0.01;
	slqSettings.acceptableTolGradientDescent_ = 1e-3;
	slqSettings.useAscendingLineSearchNLP_ = true;
	slqSettings.minEventTimeDifference_ = 0.01;

	// switching times
	std::vector<double> initSwitchingTimes {1.0};
	EXP0_LogicRules logicRules(initSwitchingTimes);

	double startTime = 0.0;
	double finalTime = 2.0;

	// partitioning times
	std::vector<double> partitioningTimes;
	partitioningTimes.push_back(startTime);
	partitioningTimes.push_back(1.0);
	partitioningTimes.push_back(finalTime);

	Eigen::Vector2d initState(0.0, 2.0);


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/

//	// setup single core version
//	OCS2Projected<2,1,2,2> ocs2 (subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
//			stateOperatingPoints, inputOperatingPoints, systemStockIndex, slqSettings);
//
//	// setup multi core version
//	slqSettings.useMultiThreading_ = true;
//	slqSettings.nThreads_ = 2;
//	OCS2Projected<2,1,2,2> ocs2_mp (subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
//				stateOperatingPoints, inputOperatingPoints, systemStockIndex, slqSettings);
//
//	ocs2.run(initSwitchingTimes[0], initState, initSwitchingTimes);
//	ocs2_mp.run(initSwitchingTimes[0], initState, initSwitchingTimes);
//
//	std::vector<double> resultingSwitchingTimes,resultingSwitchingTimes_mp;
//	ocs2.getSwitchingTimes(resultingSwitchingTimes);
//	ocs2_mp.getSwitchingTimes(resultingSwitchingTimes_mp);
//
//	double cost, cost_mp;
//	ocs2.getCostFunction(cost);
//	ocs2_mp.getCostFunction(cost_mp);
//
//	std::cout << "resulting costs: " << cost << "  " << cost_mp << std::endl;
//
//	ASSERT_LT(fabs(cost - cost_mp), slqSettings.minRelCostGSLQ_);
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

