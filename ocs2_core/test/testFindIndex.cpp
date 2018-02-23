/*
 * testFindIndex.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: farbod
 */

#include <iostream>

#include "ocs2_core/misc/FindActiveIntervalIndex.h"

#include <gtest/gtest.h>

using namespace ocs2;

TEST(testFindIndex, testFindIndex)
{
	std::vector<double> timeIntervals;
	timeIntervals.push_back(0.0);
	timeIntervals.push_back(1.0);
	timeIntervals.push_back(2.0);
	timeIntervals.push_back(3.0);

	std::cout << "timeIntervalsSplits = {";
	for (const int& i : timeIntervals)
		std::cout << i << ", ";
	std::cout << "} " << std::endl;

	// test times
	std::vector<double> testTimes {-0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
	// correct solution
	std::vector<int> activeIntervalIndeces {-1, 0, 0, 0, 1, 1, 2, 2, 3};

	bool resultsGood = true;
	for (size_t i=0; i<testTimes.size(); i++) {

		int index = findActiveIntervalIndex(timeIntervals, testTimes[i], 0);

		resultsGood = resultsGood && (index==activeIntervalIndeces[i]);

		std::cout << "time: " << testTimes[i] << " ==> activeIntervalIndex: "
				<< index << "\t correct solution is: " << activeIntervalIndeces[i] << std::endl;
	}

	ASSERT_TRUE(resultsGood);
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

