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

#include "ocs2_slq//SLQ.h"
#include "ocs2_slq/SLQ_MP.h"

#include "test_include/EXP0.h"

using namespace ocs2;

enum
{
	STATE_DIM = 2,
	INPUT_DIM = 1
};

TEST(exp0_slq_test, exp0_slq_test)
{
	// subsystem dynamics
	ControlledSystemBase<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::Ptr subsystemDynamicsPtr =
			std::allocate_shared<EXP0_System, Eigen::aligned_allocator<EXP0_System>>( Eigen::aligned_allocator<EXP0_System>() );

	// subsystem derivatives
	DerivativesBase<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::Ptr subsystemDerivativePtr =
			std::allocate_shared<EXP0_SystemDerivative, Eigen::aligned_allocator<EXP0_SystemDerivative>>( Eigen::aligned_allocator<EXP0_SystemDerivative>() );

	// subsystem constraints
	ConstraintBase<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::Ptr subsystemConstraintPtr =
			std::allocate_shared<EXP0_SystemConstraint, Eigen::aligned_allocator<EXP0_SystemConstraint>>( Eigen::aligned_allocator<EXP0_SystemConstraint>() );

	// subsystem cost functions
	CostFunctionBase<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::Ptr subsystemCostFunctionPtr =
			std::allocate_shared<EXP0_CostFunction, Eigen::aligned_allocator<EXP0_CostFunction>>( Eigen::aligned_allocator<EXP0_CostFunction>() );


	Eigen::Matrix<double,2,1> stateOperatingPoint = Eigen::Matrix<double,2,1>::Zero();
	Eigen::Matrix<double,1,1> inputOperatingPoint = Eigen::Matrix<double,1,1>::Zero();
	SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::Ptr operatingTrajectoriesPtr =
			std::allocate_shared<EXP0_SystemOperatingTrajectories, Eigen::aligned_allocator<EXP0_SystemOperatingTrajectories>>(
					Eigen::aligned_allocator<EXP0_SystemOperatingTrajectories>(), stateOperatingPoint, inputOperatingPoint );


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
	slqOptions.minLearningRateGSLQP_ = 0.0001;
	slqOptions.minRelCostGSLQP_ = 5e-4;

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
			subsystemDynamicsPtr.get(), subsystemDerivativePtr.get(),
			subsystemConstraintPtr.get(), subsystemCostFunctionPtr.get(),
			operatingTrajectoriesPtr.get(), slqOptions, logicRules);

	// GSLQ MP version
	SLQ_MP<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq_mp(
			subsystemDynamicsPtr.get(), subsystemDerivativePtr.get(),
			subsystemConstraintPtr.get(), subsystemCostFunctionPtr.get(),
			operatingTrajectoriesPtr.get(), slqOptions, logicRules);

	// run single core SLQ
	if (slqOptions.displayInfo_ || slqOptions.displayShortSummary_)
		std::cerr << "\n>>> single-core SLQ" << std::endl;
	slq.run(startTime, initState, finalTime, partitioningTimes,
			std::vector<SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::scalar_array_t>(),
			SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::state_vector_array2_t());

	// run multi-core SLQ
	if (slqOptions.displayInfo_ || slqOptions.displayShortSummary_)
		std::cerr << "\n>>> multi-core SLQ" << std::endl;
	slq_mp.run(startTime, initState, finalTime, partitioningTimes,
			std::vector<SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::scalar_array_t>(),
			SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::state_vector_array2_t());

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	// get controller
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::controller_array_t controllersStock;
	slq.getController(controllersStock);
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::controller_array_t controllersStock_mp;
	slq_mp.getController(controllersStock_mp);

	// get performance indeces
	double totalCost, totalCost_mp;
	double constraint1ISE, constraint1ISE_mp;
	double constraint2ISE, constraint2ISE_mp;
	slq.getPerformanceIndeces(totalCost, constraint1ISE, constraint2ISE);
	slq_mp.getPerformanceIndeces(totalCost_mp, constraint1ISE_mp, constraint2ISE_mp);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	const double expectedCost = 9.7667;
	ASSERT_LT(fabs(totalCost - expectedCost), 10*slqOptions.minRelCostGSLQP_) <<
			"MESSAGE: SLQ failed in the EXP0's cost test!";
	ASSERT_LT(fabs(totalCost_mp - expectedCost), 10*slqOptions.minRelCostGSLQP_) <<
			"MESSAGE: SLQ_MP failed in the EXP0's cost test!";

	const double expectedISE1 = 0.0;
	ASSERT_LT(fabs(constraint1ISE - expectedISE1), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP0's type-1 constraint ISE test!";
	ASSERT_LT(fabs(constraint1ISE_mp - expectedISE1), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP0's type-1 constraint ISE test!";

	const double expectedISE2 = 0.0;
	ASSERT_LT(fabs(constraint2ISE - expectedISE2), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP0's type-2 constraint ISE test!";
	ASSERT_LT(fabs(constraint2ISE_mp - expectedISE2), 10*slqOptions.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP0's type-2 constraint ISE test!";
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
