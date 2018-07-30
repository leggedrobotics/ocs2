/*
 * A unit test
 *
 *  Created on: Sept 20, 2016
 *      Author: farbod
 */

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <gtest/gtest.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
#include <ocs2_slq/test/EXP1.h>

#include <ocs2_ocs2/OCS2Projected.h>

using namespace ocs2;

enum
{
	STATE_DIM = 2,
	INPUT_DIM = 1
};

TEST(exp1_gslq_test, exp1_gslq_test)
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
	Eigen::Matrix<double,STATE_DIM,1> stateOperatingPoint = Eigen::Matrix<double,STATE_DIM,1>::Zero();
	Eigen::Matrix<double,INPUT_DIM,1> inputOperatingPoint = Eigen::Matrix<double,INPUT_DIM,1>::Zero();
	EXP1_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);


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
	slqSettings.useLQForDerivatives_ = false;

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
	// GSLQ - single core version
	OCS2Projected<STATE_DIM, INPUT_DIM, EXP1_LogicRules> ocs2(
			&systemDynamics, &systemDerivative,
			&systemConstraint, &systemCostFunction,
			&operatingTrajectories, slqSettings, &logicRules);


	// run ocs2
	ocs2.run(startTime, initState, finalTime, partitioningTimes);


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
