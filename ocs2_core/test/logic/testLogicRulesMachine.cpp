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

#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>

#include <gtest/gtest.h>

using namespace ocs2;

class TestLogicRules : public HybridLogicRules
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = HybridLogicRules;
	using BASE::scalar_t;
	using BASE::scalar_array_t;
	using BASE::size_array_t;

	TestLogicRules() = default;

	virtual ~TestLogicRules() = default;

	void set(const scalar_array_t& eventTimes) {
		eventTimes_ = eventTimes;
	}

	void update() override
	{}

	void rewind(const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) override
	{}

 protected:
   void insertModeSequenceTemplate(
		  const logic_template_type& modeSequenceTemplate,
		  const scalar_t& startTime,
		  const scalar_t& finalTime) override {};

private:

};

/**
 * Check solution
 * @param logicRulesMachine
 * @param eventTimesStockResult
 * @param switchedSystemIDsStockResult
 * @return
 */
bool checkSolution(
		const HybridLogicRulesMachine& logicRulesMachine,
		const std::vector<std::vector<double>>& eventTimesStockResult,
		const std::vector<std::vector<size_t>>& switchedSystemIDsStockResult) {

	size_t numPartitions = eventTimesStockResult.size();

	bool testPass = true;
	for (size_t i=0; i<numPartitions; i++) {
		if (eventTimesStockResult[i]!=logicRulesMachine.getEventTimes(i))
			if (eventTimesStockResult[i].empty() && logicRulesMachine.getSwitchingTimes(i).empty() )
				continue;
			else {
				testPass = false;
			}
	}
	for (size_t i=0; i<numPartitions; i++) {
		if (switchedSystemIDsStockResult[i] != logicRulesMachine.getEventCounters(i))
			testPass = false;
	}

	return testPass;
}


TEST(testLogicRulesMachine, LogicRulesMachine)
{
	std::shared_ptr<TestLogicRules> logicRules(new TestLogicRules());
	HybridLogicRulesMachine logicRulesMachine(logicRules);

	std::vector<double> partitioningTimes{0,1,2,3};

	std::vector<double> logicRulesEventTimes;

	std::cerr << std::endl;

	// result
	bool testPass;
	std::vector<std::vector<double>> eventTimesStockResult(3);
	std::vector<std::vector<size_t>> switchedSystemIDsStockResult(3);

	// No switch
	logicRulesEventTimes = std::vector<double>{};
	logicRules->set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### No switch:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{0};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{0};

	eventTimesStockResult[2] = std::vector<double>{};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{0};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);

	// switches at the end of partitions
	logicRulesEventTimes = std::vector<double>{0, 1, 2, 3};
	logicRules->set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### Switches at the end of partitions:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult[0] = std::vector<double>{1};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{1};

	eventTimesStockResult[1] = std::vector<double>{2};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{2};

	eventTimesStockResult[2] = std::vector<double>{3};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{3};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);

	// swiches after time interval
	logicRulesEventTimes = std::vector<double>{3, 4, 5, 6};
	logicRules->set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### Switches after time interval:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{0};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{0};

	eventTimesStockResult[2] = std::vector<double>{3};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{0};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);

	// switches before time interval
	logicRulesEventTimes = std::vector<double>{-3, -2, -1, 0};
	logicRules->set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### Switches before time interval:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{4};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{4};

	eventTimesStockResult[2] = std::vector<double>{};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{4};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);

	// switches in the middle
	logicRulesEventTimes = std::vector<double>{0, 0.5, 1.5, 2.5};
	logicRules->set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### Switches in the middle:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult[0] = std::vector<double>{0.5};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{1,2};

	eventTimesStockResult[1] = std::vector<double>{1.5};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{2,3};

	eventTimesStockResult[2] = std::vector<double>{2.5};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{3,4};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);

	// no switch in middle partition
	logicRulesEventTimes = std::vector<double>{0.5, 2.5};
	logicRules->set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### No switch in middle partition:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult[0] = std::vector<double>{0.5};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{0,1};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{1};

	eventTimesStockResult[2] = std::vector<double>{2.5};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{1,2};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);
}

TEST(testLogicRulesMachine, shortPartition)
{
	std::shared_ptr<TestLogicRules> logicRules(new TestLogicRules());
	HybridLogicRulesMachine logicRulesMachine(logicRules);

	std::vector<double> partitioningTimes;

	std::vector<double> logicRulesEventTimes{0.5, 2.5};
	logicRules->set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);

	std::cerr << std::endl;

	// result
	bool testPass;
	std::vector<std::vector<double>> eventTimesStockResult;
	std::vector<std::vector<size_t>> switchedSystemIDsStockResult;

	// Partially overlapping with one event
	partitioningTimes = std::vector<double>{1.5, 2.0, 3.0};
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### Partially overlapping with one event:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult.resize(2);
	switchedSystemIDsStockResult.resize(2);

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{1};

	eventTimesStockResult[1] = std::vector<double>{2.5};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{1, 2};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);

	// Partially overlapping with no event
	partitioningTimes = std::vector<double>{0.5, 1.5, 2.5};
	logicRulesMachine.updateLogicRules(partitioningTimes);

	std::cerr << std::endl << "======================" << std::endl;
	std::cerr << "### Partially overlapping with no event:" << std::endl;
	logicRulesMachine.display();

	eventTimesStockResult.resize(2);
	switchedSystemIDsStockResult.resize(2);

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{1};

	eventTimesStockResult[1] = std::vector<double>{2.5};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{1};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
	ASSERT_TRUE(testPass);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
