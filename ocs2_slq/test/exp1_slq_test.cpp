/*
 * exp1_slq_test.cpp
 *
 *  Created on: Dec 11, 2017
 *      Author: farbod
 */

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <gtest/gtest.h>

#include "ocs2_slq//SLQ.h"
#include "ocs2_slq/SLQ_MP.h"

#include "test_include/EXP1.h"

using namespace ocs2;

enum
{
	STATE_DIM = 2,
	INPUT_DIM = 1
};

TEST(exp1_slq_test, Exp1_slq_test)
{

	// system dynamics
	EXP1_System systemDynamics;

	// system derivatives
	EXP1_SystemDerivative systemDerivative;

	// system constraints
	EXP1_SystemConstraint systemConstraint;

	// system cost functions
	EXP1_CostFunction systemCostFunction;

	// system operatingTrajectories
	Eigen::Matrix<double,2,1> stateOperatingPoint = Eigen::Matrix<double,2,1>::Zero();
	Eigen::Matrix<double,1,1> inputOperatingPoint = Eigen::Matrix<double,1,1>::Zero();
	EXP1_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	SLQ_Settings slqOptions;
	slqOptions.displayInfo_ = false;
	slqOptions.displayShortSummary_ = true;
	slqOptions.absTolODE_ = 1e-10;
	slqOptions.relTolODE_ = 1e-7;
	slqOptions.maxNumStepsPerSecond_ = 10000;
	slqOptions.nThreads_ = 3;
	slqOptions.maxNumIterationsSLQ_ = 30;
	slqOptions.lsStepsizeGreedy_ = true;
	slqOptions.noStateConstraints_ = true;

	// switching times
	std::vector<double> switchingTimes {0.2262, 1.0176};
	EXP1_LogicRules logicRules(switchingTimes);

	double startTime = 0.0;
	double finalTime = 3.0;

	// partitioning times
	std::vector<double> partitioningTimes;
	partitioningTimes.push_back(startTime);
	partitioningTimes.push_back(switchingTimes[0]);
	partitioningTimes.push_back(switchingTimes[1]);
	partitioningTimes.push_back(finalTime);

	Eigen::Vector2d initState(2.0, 3.0);


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	// SLQ - single core version
	SLQ<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slq(
			&systemDynamics, &systemDerivative,
			&systemConstraint, &systemCostFunction,
			&operatingTrajectories, slqOptions, &logicRules);

	// GSLQ MP version
	SLQ_MP<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slq_mp(
			&systemDynamics, &systemDerivative,
			&systemConstraint, &systemCostFunction,
			&operatingTrajectories, slqOptions, &logicRules);

	// run single core SLQ
	if (slqOptions.displayInfo_ || slqOptions.displayShortSummary_)
		std::cerr << "\n>>> single-core SLQ" << std::endl;
	slq.run(startTime, initState, finalTime, partitioningTimes);

	// run multi-core SLQ
	if (slqOptions.displayInfo_ || slqOptions.displayShortSummary_)
		std::cerr << "\n>>> multi-core SLQ" << std::endl;
	slq_mp.run(startTime, initState, finalTime, partitioningTimes);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	// get controller
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP1_LogicRules>::controller_array_t controllersStock = slq.getController();
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP1_LogicRules>::controller_array_t controllersStock_mp = slq_mp.getController();

	// get performance indeces
	double totalCost, totalCost_mp;
	double constraint1ISE, constraint1ISE_mp;
	double constraint2ISE, constraint2ISE_mp;
	slq.getPerformanceIndeces(totalCost, constraint1ISE, constraint2ISE);
	slq_mp.getPerformanceIndeces(totalCost_mp, constraint1ISE_mp, constraint2ISE_mp);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	const double expectedCost = 5.4399;
	ASSERT_LT(fabs(totalCost - expectedCost), 10*slqOptions.minRelCostGSLQP_) <<
			"MESSAGE: SLQ failed in the EXP1's cost test!";
	ASSERT_LT(fabs(totalCost_mp - expectedCost), 10*slqOptions.minRelCostGSLQP_) <<
			"MESSAGE: SLQ_MP failed in the EXP1's cost test!";

	const double expectedISE1 = 0.0;
	ASSERT_LT(fabs(constraint1ISE - expectedISE1), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP1's type-1 constraint ISE test!";
	ASSERT_LT(fabs(constraint1ISE_mp - expectedISE1), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP1's type-1 constraint ISE test!";

	const double expectedISE2 = 0.0;
	ASSERT_LT(fabs(constraint2ISE - expectedISE2), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP1's type-2 constraint ISE test!";
	ASSERT_LT(fabs(constraint2ISE_mp - expectedISE2), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP1's type-2 constraint ISE test!";
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}



