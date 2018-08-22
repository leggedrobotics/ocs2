/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <gtest/gtest.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
#include <ocs2_slq/test/EXP1.h>

using namespace ocs2;

#include <algorithm>
void adjustController(
		const std::vector<double>& eventTimes,
		Dimensions<2,1>::controller_array_t& controllerStock) {

	// vector of (partition, index). -1 means end()
	std::vector<std::pair<int,int>> eventsIndices(eventTimes.size(), std::pair<int,int>(-1, -1));

	size_t indexStart = 0;
	size_t p = 0; // partitionStart

	for (size_t j=0; j<eventTimes.size(); j++) {

		const Dimensions<2,1>::scalar_t& te = eventTimes[j];
		std::pair<int,int>& ie = eventsIndices[j]; // (partition, index)

		for (; p<controllerStock.size(); p++) {

			auto beginItr = (j>0) ? controllerStock[p].time_.begin() + eventsIndices[j-1].second
					: controllerStock[p].time_.begin();

			auto lower = std::lower_bound(beginItr, controllerStock[p].time_.end(), te);

			if (lower != controllerStock[p].time_.end()) {
				ie.first = p;
				ie.second = lower - controllerStock[p].time_.begin();
				break;
			}

		} // end of p loop

	} // end of j loop

	std::vector<std::pair<int,int>> eventsIndicesOld(eventTimes.size(), std::pair<int,int>(-1, -1));
	std::pair<int,int> expectedOldEventIndex = std::pair<int,int>(0, 0);
	for (size_t j=0; j<eventTimes.size(); j++) {

		// events before the controller start time
		if (eventsIndices[j] == std::pair<int,int>(0,0)) {
			eventsIndicesOld[j] = std::pair<int,int>(0, 0);
			continue;
		}
		// events after the controller final time
		if (eventsIndices[j] == std::pair<int,int>(-1,-1)) {
			break;
		}

		for (size_t p=expectedOldEventIndex.first; p<controllerStock.size(); p++) {
			for (size_t k=expectedOldEventIndex.second; k<controllerStock[p].time_.size()-1; k++) {

				if (std::abs(controllerStock[p].time_[k]-controllerStock[p].time_[k+1]) < 1e-6) {
					eventsIndicesOld[j] = expectedOldEventIndex;
				}
			} // end of k loop
		} // end of p loop

	} // end of j loop


	for (size_t j=0; j<eventTimes.size(); j++) {

		// events before the controller start time
		if (eventsIndices[j] == std::pair<int,int>(0,0)) {
			continue;
		}
		// events after the controller final time
		if (eventsIndices[j] == std::pair<int,int>(-1,-1)) {
			break;
		}

		std::pair<int,int> startIndex;
		std::pair<int,int> finalIndex;
		Dimensions<2,1>::input_vector_t uff;
		Dimensions<2,1>::input_state_matrix_t K;

		if (eventsIndices[j].first < eventsIndicesOld[j].first) {

			startIndex = eventsIndices[j];
			finalIndex = eventsIndicesOld[j];

		} else if (eventsIndices[j].first > eventsIndicesOld[j].first) {

			startIndex = eventsIndicesOld[j];
			finalIndex = eventsIndices[j];

		} else if (eventsIndices[j].first == eventsIndicesOld[j].first) {

			startIndex.first = eventsIndices[j].first;
			finalIndex.first = eventsIndices[j].first;

			if (eventsIndices[j].second < eventsIndicesOld[j].second) {
				uff = controllerStock[startIndex.first+1].uff_.front();
				K = controllerStock[startIndex.first+1].k_.front();
			} else {
				uff = controllerStock[startIndex.first].uff_.back();
				K = controllerStock[startIndex.first].k_.back();
			}
			startIndex.second = std::min(eventsIndices[j].second, eventsIndicesOld[j].second);
			finalIndex.second = std::max(eventsIndices[j].second, eventsIndicesOld[j].second);
		}


		for (size_t i=startIndex.first; i<finalIndex.first; i++)
			for (size_t k=startIndex.first; k<finalIndex.first; k++) {

				controllerStock[i].k_[k] = K;
				controllerStock[i].uff_[k] = uff;
			}


	} // end of j loop

}

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
			&operatingTrajectories, slqSettings, &logicRules);

	// GSLQ MP version
	SLQ_MP<STATE_DIM, INPUT_DIM, EXP1_LogicRules> slq_mp(
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
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP1_LogicRules>::controller_array_t controllersStock = slq.getController();
	SLQ_BASE<STATE_DIM, INPUT_DIM, EXP1_LogicRules>::controller_array_t controllersStock_mp = slq_mp.getController();

	// get performance indices
	double totalCost, totalCost_mp;
	double constraint1ISE, constraint1ISE_mp;
	double constraint2ISE, constraint2ISE_mp;
	slq.getPerformanceIndeces(totalCost, constraint1ISE, constraint2ISE);
	slq_mp.getPerformanceIndeces(totalCost_mp, constraint1ISE_mp, constraint2ISE_mp);

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	const double expectedCost = 5.4399;
	ASSERT_LT(fabs(totalCost - expectedCost), 10*slqSettings.minRelCostGSLQP_) <<
			"MESSAGE: SLQ failed in the EXP1's cost test!";
	ASSERT_LT(fabs(totalCost_mp - expectedCost), 10*slqSettings.minRelCostGSLQP_) <<
			"MESSAGE: SLQ_MP failed in the EXP1's cost test!";

	const double expectedISE1 = 0.0;
	ASSERT_LT(fabs(constraint1ISE - expectedISE1), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP1's type-1 constraint ISE test!";
	ASSERT_LT(fabs(constraint1ISE_mp - expectedISE1), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP1's type-1 constraint ISE test!";

	const double expectedISE2 = 0.0;
	ASSERT_LT(fabs(constraint2ISE - expectedISE2), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ failed in the EXP1's type-2 constraint ISE test!";
	ASSERT_LT(fabs(constraint2ISE_mp - expectedISE2), 10*slqSettings.minRelConstraint1ISE_) <<
			"MESSAGE: SLQ_MP failed in the EXP1's type-2 constraint ISE test!";
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}



