/*
 * testFindIndex.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: farbod
 */

#include <limits>
#include <cstddef>
#include <iostream>
#include <algorithm>

#include "ocs2_core/misc/FindActiveIntervalIndex.h"

#include <gtest/gtest.h>

using namespace ocs2;

TEST(testFindIndex, single_event_time)
{
	std::vector<double> timeIntervals;
	timeIntervals.push_back(std::numeric_limits<double>::lowest());
	timeIntervals.push_back(1.0);
	timeIntervals.push_back(std::numeric_limits<double>::max());

	std::cout << "timeIntervalsSplits = {";
	for (const int& i : timeIntervals)
		std::cout << i << ", ";
	std::cout << "\b\b} " << std::endl;

	// test times
	std::vector<double> testTimes {0.0, 1.0, 2.0};
	// correct solution
	std::vector<int> activeIntervalIndeces {0, 0, 1};

	bool resultsGood = true;
	int index = 0;
	// Moving forward test
	std::cout << "### Moving forward test" << std::endl;
	for (size_t i=0; i<testTimes.size(); i++) {

		index = std::max(index, 0);
		index = std::min(index, (int)timeIntervals.size()-2);

		index = findActiveIntervalIndex(timeIntervals, testTimes[i], index);

		std::cout << "time: " << testTimes[i] << " \t activeIntervalIndex: "
				<< index << " \t correct solution is: " << activeIntervalIndeces[i] << std::endl;

		resultsGood = resultsGood && (index==activeIntervalIndeces[i]);
	}

	// Moving backward test
	std::cout << "### Moving backward test" << std::endl;
	for (int i=testTimes.size()-1; i>=0; i--) {

		index = std::max(index, 0);
		index = std::min(index, (int)timeIntervals.size()-2);

		index = findActiveIntervalIndex(timeIntervals, testTimes[i], index);

		std::cout << "time: " << testTimes[i] << " \t activeIntervalIndex: "
				<< index << " \t correct solution is: " << activeIntervalIndeces[i] << std::endl;

		resultsGood = resultsGood && (index==activeIntervalIndeces[i]);
	}

	ASSERT_TRUE(resultsGood);
}

TEST(testFindIndex, testFindIndex_ceiling)
{
	std::vector<double> timeIntervals;
	timeIntervals.push_back(0.0);
	timeIntervals.push_back(1.0);
	timeIntervals.push_back(2.0);
	timeIntervals.push_back(3.0);

	std::cout << "timeIntervalsSplits = {";
	for (const int& i : timeIntervals)
		std::cout << i << ", ";
	std::cout << "\b\b} " << std::endl;

	// test times
	std::vector<double> testTimes {-0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
	// correct solution
	std::vector<int> activeIntervalIndeces {-1, 0, 0, 0, 1, 1, 2, 2, 3};

	bool resultsGood = true;
	int index = 0;
	// Moving forward test
	std::cout << "### Moving forward test" << std::endl;
	for (size_t i=0; i<testTimes.size(); i++) {

		index = std::max(index, 0);
		index = std::min(index, (int)timeIntervals.size()-2);

		index = findActiveIntervalIndex(timeIntervals, testTimes[i], index);

		std::cout << "time: " << testTimes[i] << " \t activeIntervalIndex: "
				<< index << " \t correct solution is: " << activeIntervalIndeces[i] << std::endl;

		resultsGood = resultsGood && (index==activeIntervalIndeces[i]);
	}

	// Moving backward test
	std::cout << "### Moving backward test" << std::endl;
	for (int i=testTimes.size()-1; i>=0; i--) {

		index = std::max(index, 0);
		index = std::min(index, (int)timeIntervals.size()-2);

		index = findActiveIntervalIndex(timeIntervals, testTimes[i], index);

		std::cout << "time: " << testTimes[i] << " \t activeIntervalIndex: "
				<< index << " \t correct solution is: " << activeIntervalIndeces[i] << std::endl;

		resultsGood = resultsGood && (index==activeIntervalIndeces[i]);
	}

	ASSERT_TRUE(resultsGood);
}

TEST(testFindIndex, testFindIndex_floor)
{
	std::vector<double> timeIntervals;
	timeIntervals.push_back(0.0);
	timeIntervals.push_back(1.0);
	timeIntervals.push_back(2.0);
	timeIntervals.push_back(3.0);

	std::cout << "timeIntervalsSplits = {";
	for (const int& i : timeIntervals)
		std::cout << i << ", ";
	std::cout << "\b\b} " << std::endl;

	// test times
	std::vector<double> testTimes {-0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
	// correct solution
	std::vector<int> activeIntervalIndeces {-1, 0, 0, 1, 1, 2, 2, 2, 3};

	bool resultsGood = true;
	int index = 0;
	// Moving forward test
	std::cout << "### Moving forward test" << std::endl;
	for (size_t i=0; i<testTimes.size(); i++) {

		index = std::max(index, 0);
		index = std::min(index, (int)timeIntervals.size()-2);

		index = findActiveIntervalIndex(timeIntervals, testTimes[i], index,
				-OCS2NumericTraits<double>::week_epsilon());

		std::cout << "time: " << testTimes[i] << " \t activeIntervalIndex: "
				<< index << " \t correct solution is: " << activeIntervalIndeces[i] << std::endl;

		resultsGood = resultsGood && (index==activeIntervalIndeces[i]);
	}

	// Moving backward test
	std::cout << "### Moving backward test" << std::endl;
	for (int i=testTimes.size()-1; i>=0; i--) {

		index = std::max(index, 0);
		index = std::min(index, (int)timeIntervals.size()-2);

		index = findActiveIntervalIndex(timeIntervals, testTimes[i], index,
				-OCS2NumericTraits<double>::week_epsilon());

		std::cout << "time: " << testTimes[i] << " \t activeIntervalIndex: "
				<< index << " \t correct solution is: " << activeIntervalIndeces[i] << std::endl;

		resultsGood = resultsGood && (index==activeIntervalIndeces[i]);
	}

	ASSERT_TRUE(resultsGood);
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

