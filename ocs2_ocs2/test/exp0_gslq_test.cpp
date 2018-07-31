/*
 * A unit test for example system 0
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
#include <ocs2_slq/test/EXP0.h>

#include <ocs2_ocs2/GSLQ.h>

using namespace ocs2;

enum
{
	STATE_DIM = 2,
	INPUT_DIM = 1
};

TEST(exp0_gslq_test, exp0_gslq_test)
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
	slqSettings.maxNumIterationsSLQ_ = 30;
	slqSettings.lsStepsizeGreedy_ = true;
	slqSettings.noStateConstraints_ = true;
	slqSettings.useLQForDerivatives_ = true;

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
	// GSLQ - single core version
	SLQ<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq(
			&systemDynamics, &systemDerivative,
			&systemConstraint, &systemCostFunction,
			&operatingTrajectories, slqSettings, &logicRules);

	GSLQ<STATE_DIM, INPUT_DIM, EXP0_LogicRules> gslq(slq);


//	// GSLQ MP version
//	SLQ_MP<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq_mp(
//			&systemDynamics, &systemDerivative,
//			&systemConstraint, &systemCostFunction,
//			&operatingTrajectories, slqSettings, &logicRules);

	// run single core GSLQ
	if (slqSettings.displayInfo_ || slqSettings.displayShortSummary_)
	slq.run(startTime, initState, finalTime, partitioningTimes);
	gslq.run();

//	// run multi-core GSLQ
//	if (slqSettings.displayInfo_ || slqSettings.displayShortSummary_)
//		std::cerr << "\n>>> multi-core SLQ" << std::endl;
//	slq_mp.run(startTime, initState, finalTime, partitioningTimes);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
//	// run both the mp and the single core version
//	gslq.run(switchingTimes[0], initState, switchingTimes);
//	gslq_mp.run(switchingTimes_mp[0], initState, switchingTimes_mp);
//
//	// get controller
//	gslq.getController(controllersStock);
//	gslq_mp.getController(controllersStock_mp);
//
//	// rollout both versions
//	std::vector<GSLQ<2,1,2,3>::scalar_array_t> timeTrajectoriesStock, timeTrajectoriesStock_mp;
//	std::vector<GSLQ<2,1,2,3>::state_vector_array_t> stateTrajectoriesStock, stateTrajectoriesStock_mp;
//	std::vector<GSLQ<2,1,2,3>::control_vector_array_t> controlTrajectoriesStock, controlTrajectoriesStock_mp;
//	gslq.rollout(switchingTimes[0], initState, controllersStock, timeTrajectoriesStock, stateTrajectoriesStock, controlTrajectoriesStock);
//	gslq_mp.rollout(switchingTimes_mp[0], initState, controllersStock_mp, timeTrajectoriesStock_mp, stateTrajectoriesStock_mp, controlTrajectoriesStock_mp);
//
//	// compute cost for both versions
//	double rolloutCost, rolloutCost_mp;
//	gslq.calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, controlTrajectoriesStock, rolloutCost);
//	gslq_mp.calculateCostFunction(timeTrajectoriesStock_mp, stateTrajectoriesStock_mp, controlTrajectoriesStock_mp, rolloutCost_mp);
//
//	// value function for both versions
//	double totalCost;
//	double totalCost_mp;
//	gslq.getValueFuntion(0.0, initState, totalCost);
//	gslq_mp.getValueFuntion(0.0, initState, totalCost_mp);
//
	// value function derivative for both versions
	Eigen::Matrix<double,1,1> costFunctionDerivative, costFunctionDerivative_mp;
	gslq.getCostFuntionDerivative(costFunctionDerivative);
//	gslq_mp.getCostFuntionDerivative(costFunctionDerivative_mp);


//	/******************************************************************************************************/
//	/******************************************************************************************************/
//	/******************************************************************************************************/
	std::cout << "Single core switching times are: [" << switchingTimes[0] << "]\n";

//	std::cout << "MP switching times are: [" << switchingTimes_mp[0] << ", ";
//	for (size_t i=1; i<switchingTimes_mp.size()-1; i++)
//		std::cout << switchingTimes_mp[i] << ", ";
//	std::cout << switchingTimes_mp.back() << "]\n";
//
//	for (size_t i=0; i<switchingTimes_mp.size(); i++)
//			ASSERT_LT(fabs(switchingTimes_mp[i]-switchingTimes[i]), 1e-4);
//
//	std::cout << "The single core total cost in the test rollout: " << rolloutCost << std::endl;
//	std::cout << "The MP total cost in the test rollout: " << rolloutCost_mp << std::endl;
//	ASSERT_LT(fabs(rolloutCost_mp - rolloutCost), 10*gslqOptions.minRelCostGSLQ_);
//
	std::cout << "The single core total cost derivative: " << costFunctionDerivative.transpose() << std::endl;
//	std::cout << "The MP total cost derivative: " << costFunctionDerivative_mp.transpose() << std::endl;
////	for (size_t i=0; i<costFunctionDerivative_mp.size(); i++)
////		ASSERT_LT(fabs(costFunctionDerivative[i]-costFunctionDerivative_mp[i]), gslqOptions.minRelCostGSLQ_*10);
//
//	std::cout << "SLQ_MP cost			" << totalCost_mp << std::endl;
//	std::cout << "single core cost 		" << totalCost << std::endl;
//	ASSERT_LT(fabs(totalCost_mp - totalCost), 10*gslqOptions.minRelCostGSLQ_);
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
