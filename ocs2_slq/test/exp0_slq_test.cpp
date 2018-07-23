/*
 * A unit test
 *
 *  Created on: Sept 13, 2017
 *      Author: farbod
 */

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <gtest/gtest.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
#include <ocs2_slq/test/EXP0.h>

using namespace ocs2;

enum
{
	STATE_DIM = 2,
	INPUT_DIM = 1
};

TEST(exp0_slq_test, exp0_slq_test)
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
	Eigen::Matrix<double,2,1> stateOperatingPoint = Eigen::Matrix<double,2,1>::Zero();
	Eigen::Matrix<double,1,1> inputOperatingPoint = Eigen::Matrix<double,1,1>::Zero();
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
	slqSettings.maxNumIterationsSLQ_ = 30;
	slqSettings.lsStepsizeGreedy_ = true;
	slqSettings.noStateConstraints_ = true;
	slqSettings.minLearningRateGSLQP_ = 0.0001;
	slqSettings.minRelCostGSLQP_ = 5e-4;

	// switching times
	std::vector<double> switchingTimes {0.1897};
	EXP0_LogicRules logicRules(switchingTimes);

	double startTime = 0.0;
	double finalTime = 2.0;

	// partitioning times
	std::vector<double> partitioningTimes;
	partitioningTimes.push_back(startTime);
	partitioningTimes.push_back(switchingTimes[0]);
	partitioningTimes.push_back(finalTime);

	Eigen::Vector2d initState(0.0, 2.0);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	// SLQ - single core version
	SLQ<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq(
			&systemDynamics, &systemDerivative,
			&systemConstraint, &systemCostFunction,
			&operatingTrajectories, slqSettings, &logicRules);

	// GSLQ MP version
	SLQ_MP<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq_mp(
			&systemDynamics, &systemDerivative,
			&systemConstraint, &systemCostFunction,
			&operatingTrajectories, slqSettings, &logicRules);

	// run single core SLQ
	if (slqSettings.displayInfo_ || slqSettings.displayShortSummary_)
		std::cerr << "\n>>> single-core SLQ" << std::endl;
	slq.run(startTime, initState, finalTime, partitioningTimes);

	// run multi-core SLQ
	if (slqSettings.displayInfo_ || slqSettings.displayShortSummary_)
		std::cerr << "\n>>> multi-core SLQ" << std::endl;
	slq_mp.run(startTime, initState, finalTime, partitioningTimes);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	// get controller
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::controller_array_t controllersStock = slq.getController();
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::controller_array_t controllersStock_mp = slq_mp.getController();

	// get performance indices
	double totalCost, totalCost_mp;
	double constraint1ISE, constraint1ISE_mp;
	double constraint2ISE, constraint2ISE_mp;
	slq.getPerformanceIndeces(totalCost, constraint1ISE, constraint2ISE);
	slq_mp.getPerformanceIndeces(totalCost_mp, constraint1ISE_mp, constraint2ISE_mp);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	const double expectedCost = 9.7667;
	ASSERT_LT(fabs(totalCost - expectedCost), 10*slqSettings.minRelCostGSLQP_) <<
			"MESSAGE: SLQ failed in the EXP0's cost test!";
	ASSERT_LT(fabs(totalCost_mp - expectedCost), 10*slqSettings.minRelCostGSLQP_) <<
			"MESSAGE: SLQ_MP failed in the EXP0's cost test!";

	const double expectedISE1 = 0.0;
	ASSERT_LT(fabs(constraint1ISE - expectedISE1), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP0's type-1 constraint ISE test!";
	ASSERT_LT(fabs(constraint1ISE_mp - expectedISE1), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP0's type-1 constraint ISE test!";

	const double expectedISE2 = 0.0;
	ASSERT_LT(fabs(constraint2ISE - expectedISE2), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP0's type-2 constraint ISE test!";
	ASSERT_LT(fabs(constraint2ISE_mp - expectedISE2), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP0's type-2 constraint ISE test!";
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
