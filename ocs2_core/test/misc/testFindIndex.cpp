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
				-OCS2NumericTraits<double>::weakEpsilon());

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
				-OCS2NumericTraits<double>::weakEpsilon());

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

