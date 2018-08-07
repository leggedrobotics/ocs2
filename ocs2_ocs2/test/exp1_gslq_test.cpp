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

#include <ocs2_ocs2/GSLQ_BASE.h>

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
	slqSettings.absTolODE_ = 1e-12;
	slqSettings.relTolODE_ = 1e-9;
	slqSettings.maxNumStepsPerSecond_ = 50000;
	slqSettings.nThreads_ = 3;
	slqSettings.maxNumIterationsSLQ_ = 30;
	slqSettings.lsStepsizeGreedy_ = true;
	slqSettings.noStateConstraints_ = true;
	slqSettings.useLQForDerivatives_ = false;
	slqSettings.minRelCostGSLQP_ = 1e-3;

	// event times
	std::vector<double> optimumEventTimes {0.2262, 1.0176};
	EXP1_LogicRules logicRules(optimumEventTimes);

	double startTime = 0.0;
	double finalTime = 3.0;

	// partitioning times
	std::vector<double> partitioningTimes;
	partitioningTimes.push_back(startTime);
	partitioningTimes.push_back(1.0);
	partitioningTimes.push_back(2.0);
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
	// SLQ data collector
	SLQ_DataCollector<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slqDataCollector;
	// GSLQ
	GSLQ_BASE<STATE_DIM, INPUT_DIM, EXP1_LogicRules> gslq(slqSettings);

	// run GSLQ using LQ
	slq.settings().useLQForDerivatives_ = true;
	gslq.settings().useLQForDerivatives_ = true;
	slq.run(startTime, initState, finalTime, partitioningTimes);
	slqDataCollector.collect(&slq);
	gslq.run(optimumEventTimes, &slqDataCollector);
	// cost derivative
	Eigen::Matrix<double,1,1> costFunctionDerivative_LQ;
	gslq.getCostFuntionDerivative(costFunctionDerivative_LQ);

	// run GSLQ using BVP
	slq.settings().useLQForDerivatives_ = false;
	gslq.settings().useLQForDerivatives_ = false;
	slq.reset();
	slq.run(startTime, initState, finalTime, partitioningTimes);
	slqDataCollector.collect(&slq);
	gslq.run(optimumEventTimes, &slqDataCollector);
	// cost derivative
	Eigen::Matrix<double,1,1> costFunctionDerivative_BVP;
	gslq.getCostFuntionDerivative(costFunctionDerivative_BVP);

	// cost
	double costFunction, constraint1ISE, constraint2ISE;
	slq.getPerformanceIndeces(costFunction, constraint1ISE, constraint2ISE);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	std::cerr << "### Optimum event times are: [" << optimumEventTimes[0] << ", ";
	for (size_t i=1; i<optimumEventTimes.size()-1; i++)
		std::cerr << optimumEventTimes[i] << ", ";
	std::cerr << optimumEventTimes.back() << "]\n";

	std::cerr << "### Optimum cost is: " << costFunction << "\n";

	std::cerr << "### Optimum cost derivative LQ method:  [" << costFunctionDerivative_LQ(0) << ", ";
	for (size_t i=1; i<costFunctionDerivative_LQ.size()-1; i++)
		std::cerr << costFunctionDerivative_LQ(i) << ", ";
	std::cerr << costFunctionDerivative_LQ.tail<1>()(0) << "]\n";

	std::cerr << "### Optimum cost derivative BVP method: [" << costFunctionDerivative_BVP(0) << ", ";
	for (size_t i=1; i<costFunctionDerivative_BVP.size()-1; i++)
		std::cerr << costFunctionDerivative_BVP(i) << ", ";
	std::cerr << costFunctionDerivative_BVP.tail<1>()(0) << "]\n";

	ASSERT_LT(costFunctionDerivative_LQ.norm()/fabs(costFunction), 50*slqSettings.minRelCostGSLQP_ /*0.05*/) <<
			"MESSAGE: GSLQ failed in the EXP1's cost derivative LQ test!";

	ASSERT_LT(costFunctionDerivative_BVP.norm()/fabs(costFunction), 50*slqSettings.minRelCostGSLQP_ /*0.05*/) <<
			"MESSAGE: GSLQ failed in the EXP1's cost derivative BVP test!";
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
