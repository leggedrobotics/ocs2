/*
 * testLogicRulesMachine.cpp
 *
 *  Created on: Dec 19, 2017
 *      Author: farbod
 */


#include <ocs2_core/logic/LogicRulesMachine.h>

#include <gtest/gtest.h>

using namespace ocs2;

template <size_t STATE_DIM, size_t INPUT_DIM>
class TestLogicRules : public LogicRulesBase<STATE_DIM,INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase<STATE_DIM,INPUT_DIM> BASE;
	typedef typename BASE::scalar_array_t scalar_array_t;
	typedef typename BASE::size_array_t size_array_t;
	typedef typename BASE::controller_t controller_t;

	TestLogicRules() {}

	virtual ~TestLogicRules() {}

	void adjustController(controller_t& controller) const override
			{}

	void set(const scalar_array_t& logicRulesSwitchingTimes) {
		BASE::logicRulesSwitchingTimes_ = logicRulesSwitchingTimes;
	}

private:

};

/**
 * Check solution
 * @param logicRulesMachine
 * @param eventTimesStockResult
 * @param switchedSystemIDsStockResult
 * @return
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
bool checkSolution(const LogicRulesMachine<STATE_DIM,INPUT_DIM,TestLogicRules<STATE_DIM,INPUT_DIM>>& logicRulesMachine,
		const std::vector<std::vector<double>>& eventTimesStockResult,
		const std::vector<std::vector<size_t>>& switchedSystemIDsStockResult) {

	size_t numPartitionings = eventTimesStockResult.size();

	bool testPass = true;
	for (size_t i=0; i<numPartitionings; i++) {
		if (eventTimesStockResult[i]!=logicRulesMachine.getSwitchingTimes(i))
			if (eventTimesStockResult[i].empty() && logicRulesMachine.getSwitchingTimes(i).empty() )
				continue;
			else {
				testPass = false;
			}
	}
	for (size_t i=0; i<numPartitionings; i++) {
		if (switchedSystemIDsStockResult[i] != logicRulesMachine.getSwitchedSystemIDs(i))
			testPass = false;
	}

	return testPass;
}


TEST(testLogicRulesMachine, LogicRulesMachine)
{
	bool resultsGood = true;

	TestLogicRules<1,1> logicRules;
	LogicRulesMachine<1,1,TestLogicRules<1,1>> logicRulesMachine(logicRules);

	LogicRulesMachine<1,1,TestLogicRules<1,1>>::controller_array_t controllerStock;

	std::vector<double> partitioningTimes{0,1,2,3};
	controllerStock.resize(partitioningTimes.size()-1);

	std::vector<double> logicRulesSwitchingTimes;

	std::cout << std::endl;

	// result
	bool testPass;
	std::vector<std::vector<double>> eventTimesStockResult(3);
	std::vector<std::vector<size_t>> switchedSystemIDsStockResult(3);

	// No switch
	logicRulesSwitchingTimes = std::vector<double>{};
	logicRules.set(logicRulesSwitchingTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	std::cout << "### No switch:" << std::endl;
	logicRulesMachine.displaySwitchedSystemsDistribution();

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{0};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{0};

	eventTimesStockResult[2] = std::vector<double>{};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{0};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
//	std::cout << "Test Pass: " <<  testPass << std::endl;
	resultsGood = resultsGood && testPass;

	// switches at the end of partitions
	logicRulesSwitchingTimes = std::vector<double>{0, 1, 2, 3};
	logicRules.set(logicRulesSwitchingTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	std::cout << "### Switches at the end of partitions:" << std::endl;
	logicRulesMachine.displaySwitchedSystemsDistribution();

	eventTimesStockResult[0] = std::vector<double>{1};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{1};

	eventTimesStockResult[1] = std::vector<double>{2};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{2};

	eventTimesStockResult[2] = std::vector<double>{3};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{3};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
//	std::cout << "Test Pass: " <<  testPass << std::endl;
	resultsGood = resultsGood && testPass;

	// swiches after time interval
	logicRulesSwitchingTimes = std::vector<double>{3, 4, 5, 6};
	logicRules.set(logicRulesSwitchingTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	std::cout << "### Switches after time interval:" << std::endl;
	logicRulesMachine.displaySwitchedSystemsDistribution();

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{0};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{0};

	eventTimesStockResult[2] = std::vector<double>{3};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{0};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
//	std::cout << "Test Pass: " <<  testPass << std::endl;
	resultsGood = resultsGood && testPass;

	// switches before time interval
	logicRulesSwitchingTimes = std::vector<double>{-3, -2, -1, 0};
	logicRules.set(logicRulesSwitchingTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	std::cout << "### Switches before time interval:" << std::endl;
	logicRulesMachine.displaySwitchedSystemsDistribution();

	eventTimesStockResult[0] = std::vector<double>{};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{4};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{4};

	eventTimesStockResult[2] = std::vector<double>{};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{4};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
//	std::cout << "Test Pass: " <<  testPass << std::endl;
	resultsGood = resultsGood && testPass;

	// switches in the middle
	logicRulesSwitchingTimes = std::vector<double>{0, 0.5, 1.5, 2.5};
	logicRules.set(logicRulesSwitchingTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	std::cout << "### Switches in the middle:" << std::endl;
	logicRulesMachine.displaySwitchedSystemsDistribution();

	eventTimesStockResult[0] = std::vector<double>{0.5};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{1,2};

	eventTimesStockResult[1] = std::vector<double>{1.5};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{2,3};

	eventTimesStockResult[2] = std::vector<double>{2.5};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{3,4};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
//	std::cout << "Test Pass: " <<  testPass << std::endl;
	resultsGood = resultsGood && testPass;

	// no switch in middle partition
	logicRulesSwitchingTimes = std::vector<double>{0.5, 2.5};
	logicRules.set(logicRulesSwitchingTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	std::cout << "### No switch in middle partition:" << std::endl;
	logicRulesMachine.displaySwitchedSystemsDistribution();

	eventTimesStockResult[0] = std::vector<double>{0.5};
	switchedSystemIDsStockResult[0] = std::vector<size_t>{0,1};

	eventTimesStockResult[1] = std::vector<double>{};
	switchedSystemIDsStockResult[1] = std::vector<size_t>{1};

	eventTimesStockResult[2] = std::vector<double>{2.5};
	switchedSystemIDsStockResult[2] = std::vector<size_t>{1,2};

	testPass = checkSolution(logicRulesMachine, eventTimesStockResult, switchedSystemIDsStockResult);
//	std::cout << "Test Pass: " <<  testPass << std::endl;
	resultsGood = resultsGood && testPass;

	ASSERT_TRUE(resultsGood);
}


int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


