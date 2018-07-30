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

#include <ocs2_ocs2/GSLQ.h>

using namespace ocs2;

enum
{
	STATE_DIM = 2,
	INPUT_DIM = 1
};

TEST(exp1_gslq_test, optimum_gradient_test)
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
	slqSettings.displayShortSummary_ = false;
	slqSettings.absTolODE_ = 1e-10;
	slqSettings.relTolODE_ = 1e-7;
	slqSettings.maxNumStepsPerSecond_ = 10000;
	slqSettings.maxNumIterationsSLQ_ = 30;
	slqSettings.lsStepsizeGreedy_ = true;
	slqSettings.noStateConstraints_ = true;
	slqSettings.useLQForDerivatives_ = false;
	slqSettings.minRelCostGSLQP_ = 1e-3;

	// switching times
	std::vector<double> optimumEventTimes {0.2262, 1.0176};
	EXP1_LogicRules logicRules(optimumEventTimes);

	double startTime = 0.0;
	double finalTime = 3.0;

	// partitioning times
	std::vector<double> partitioningTimes;
	partitioningTimes.push_back(startTime);
	partitioningTimes.push_back(optimumEventTimes[0]);
	partitioningTimes.push_back(optimumEventTimes[1]);
	partitioningTimes.push_back(finalTime);

	Eigen::Vector2d initState(2.0, 3.0);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	// SLQ - single core version
	SLQ<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slq(
			&systemDynamics, &systemDerivative,
			&systemConstraint, &systemCostFunction,
			&operatingTrajectories, slqSettings, &logicRules);
	// GSLQ
	GSLQ<STATE_DIM, INPUT_DIM, EXP1_LogicRules> gslq(slq);

	// run GSLQ using LQ
	slq.settings().useLQForDerivatives_ = true;
	slq.run(startTime, initState, finalTime, partitioningTimes);
	gslq.run();
	// cost derivative
	Eigen::Matrix<double,2,1> costFunctionDerivative_LQ;
	gslq.getCostFuntionDerivative(costFunctionDerivative_LQ);

	// run GSLQ using BVP
	slq.settings().useLQForDerivatives_ = false;
	slq.run(startTime, initState, finalTime, partitioningTimes);
	gslq.run();
	// cost derivative
	Eigen::Matrix<double,2,1> costFunctionDerivative_BVP;
	gslq.getCostFuntionDerivative(costFunctionDerivative_BVP);

	// cost
	double costFunction, constraint1ISE, constraint2ISE;
	slq.getPerformanceIndeces(costFunction, constraint1ISE, constraint2ISE);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	std::cout << "### Optimum event times are: [" << optimumEventTimes[0] << ", ";
	for (size_t i=1; i<optimumEventTimes.size()-1; i++)
		std::cout << optimumEventTimes[i] << ", ";
	std::cout << optimumEventTimes.back() << "]\n";

	std::cout << "### Optimum cost is: " << costFunction << "\n";

	std::cout << "### Optimum cost derivative LQ method:  [" << costFunctionDerivative_LQ(0) << ", ";
	for (size_t i=1; i<costFunctionDerivative_LQ.size()-1; i++)
		std::cout << costFunctionDerivative_LQ(i) << ", ";
	std::cout << costFunctionDerivative_LQ.tail<1>()(0) << "]\n";

	std::cout << "### Optimum cost derivative BVP method: [" << costFunctionDerivative_BVP(0) << ", ";
	for (size_t i=1; i<costFunctionDerivative_BVP.size()-1; i++)
		std::cout << costFunctionDerivative_BVP(i) << ", ";
	std::cout << costFunctionDerivative_BVP.tail<1>()(0) << "]\n";

	ASSERT_LT(costFunctionDerivative_LQ.norm()/fabs(costFunction), 10*slqSettings.minRelCostGSLQP_) <<
			"MESSAGE: GSLQ failed in the EXP1's cost derivative LQ test!";

	ASSERT_LT(costFunctionDerivative_BVP.norm()/fabs(costFunction), 10*slqSettings.minRelCostGSLQP_) <<
			"MESSAGE: GSLQ failed in the EXP1's cost derivative BVP test!";
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
