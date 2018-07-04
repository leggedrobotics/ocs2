/*
 * testEventTimeIndexer.cpp
 *
 *  Created on: Dec 19, 2017
 *      Author: farbod
 */


#include <ocs2_ocs2/EventTimeIndexer.h>

#include <gtest/gtest.h>

using namespace ocs2;

template <size_t STATE_DIM, size_t INPUT_DIM>
class TestLogicRules : public LogicRulesBase<STATE_DIM,INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase<STATE_DIM,INPUT_DIM> BASE;
	typedef typename BASE::scalar_t scalar_t;
	typedef typename BASE::scalar_array_t scalar_array_t;
	typedef typename BASE::size_array_t size_array_t;
	typedef typename BASE::controller_t controller_t;

	TestLogicRules() {}

	virtual ~TestLogicRules() {}

	void adjustController(controller_t& controller) const override
	{}

	void set(const scalar_array_t& eventTimes) {
		BASE::eventTimes_ = eventTimes;
	}

	void update() override
	{}

	void rewind(const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) override
	{}

private:

};


TEST(testEventTimeIndexer, test_0)
{
	TestLogicRules<1,1> logicRules;
	LogicRulesMachine<1,1,TestLogicRules<1,1>> logicRulesMachine(logicRules);

	LogicRulesMachine<1,1,TestLogicRules<1,1>>::controller_array_t controllerStock;

	// Times
	std::vector<double> partitioningTimes{0.5,1.5,2.5};
	controllerStock.resize(partitioningTimes.size()-1);
	std::vector<double> logicRulesEventTimes = std::vector<double>{0.25, 0.75, 1.25, 1.75, 2.25, 2.75};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	// expected result
	EventTimeIndexer::size_array2_t subsystemsIndecesResult(logicRulesEventTimes.size()+1);
	EventTimeIndexer::int_array2_t partitionsDistResult(logicRulesEventTimes.size()+1);

	partitionsDistResult[0] = EventTimeIndexer::int_array_t{-1};
	subsystemsIndecesResult[0] = EventTimeIndexer::size_array_t{};

	partitionsDistResult[1] = EventTimeIndexer::int_array_t{0};
	subsystemsIndecesResult[1] = EventTimeIndexer::size_array_t{0};

	partitionsDistResult[2] = EventTimeIndexer::int_array_t{0};
	subsystemsIndecesResult[2] = EventTimeIndexer::size_array_t{1};

	partitionsDistResult[3] = EventTimeIndexer::int_array_t{0, 1};
	subsystemsIndecesResult[3] = EventTimeIndexer::size_array_t{2, 0};

	partitionsDistResult[4] = EventTimeIndexer::int_array_t{1};
	subsystemsIndecesResult[4] = EventTimeIndexer::size_array_t{1};

	partitionsDistResult[5] = EventTimeIndexer::int_array_t{1};
	subsystemsIndecesResult[5] = EventTimeIndexer::size_array_t{2};

	partitionsDistResult[6] = EventTimeIndexer::int_array_t{2};
	subsystemsIndecesResult[6] = EventTimeIndexer::size_array_t{};

	// display
	std::cerr << std::endl;
	std::cerr << std::endl << "======================" << std::endl;
	logicRulesMachine.display();

	EventTimeIndexer eventTimeIndexer;
	eventTimeIndexer.set(logicRulesMachine);
	eventTimeIndexer.display();

	bool testPass = true;
	for (size_t i=0; i<eventTimeIndexer.numSubsystems(); i++) {
		if (partitionsDistResult[i] != eventTimeIndexer.partitionsDistribution(i))
			testPass = false;
		if (subsystemsIndecesResult[i] != eventTimeIndexer.subsystemIndeces(i))
			testPass = false;
	}

	ASSERT_TRUE(testPass);
}


TEST(testEventTimeIndexer, test_1)
{
	TestLogicRules<1,1> logicRules;
	LogicRulesMachine<1,1,TestLogicRules<1,1>> logicRulesMachine(logicRules);

	LogicRulesMachine<1,1,TestLogicRules<1,1>>::controller_array_t controllerStock;

	// Times
	std::vector<double> partitioningTimes{1, 2, 3};
	controllerStock.resize(partitioningTimes.size()-1);
	std::vector<double> logicRulesEventTimes = std::vector<double>{0, 1, 2, 3, 4};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	// expected result
	EventTimeIndexer::size_array2_t subsystemsIndecesResult(logicRulesEventTimes.size()+1);
	EventTimeIndexer::int_array2_t partitionsDistResult(logicRulesEventTimes.size()+1);

	partitionsDistResult[0] = EventTimeIndexer::int_array_t{-1};
	subsystemsIndecesResult[0] = EventTimeIndexer::size_array_t{};

	partitionsDistResult[1] = EventTimeIndexer::int_array_t{-1};
	subsystemsIndecesResult[1] = EventTimeIndexer::size_array_t{};

	partitionsDistResult[2] = EventTimeIndexer::int_array_t{0};
	subsystemsIndecesResult[2] = EventTimeIndexer::size_array_t{0};

	partitionsDistResult[3] = EventTimeIndexer::int_array_t{1};
	subsystemsIndecesResult[3] = EventTimeIndexer::size_array_t{0};

	partitionsDistResult[4] = EventTimeIndexer::int_array_t{2};
	subsystemsIndecesResult[4] = EventTimeIndexer::size_array_t{};

	partitionsDistResult[5] = EventTimeIndexer::int_array_t{2};
	subsystemsIndecesResult[5] = EventTimeIndexer::size_array_t{};

	// display
	std::cerr << std::endl;
	std::cerr << std::endl << "======================" << std::endl;
	logicRulesMachine.display();

	EventTimeIndexer eventTimeIndexer;
	eventTimeIndexer.set(logicRulesMachine);
	eventTimeIndexer.display();

	bool testPass = true;
	for (size_t i=0; i<eventTimeIndexer.numSubsystems(); i++) {
		if (partitionsDistResult[i] != eventTimeIndexer.partitionsDistribution(i))
			testPass = false;
		if (subsystemsIndecesResult[i] != eventTimeIndexer.subsystemIndeces(i))
			testPass = false;
	}

	ASSERT_TRUE(testPass);
}


TEST(testEventTimeIndexer, test_2)
{
	TestLogicRules<1,1> logicRules;
	LogicRulesMachine<1,1,TestLogicRules<1,1>> logicRulesMachine(logicRules);

	LogicRulesMachine<1,1,TestLogicRules<1,1>>::controller_array_t controllerStock;

	// Times
	std::vector<double> partitioningTimes{1, 2, 3, 4, 5};
	controllerStock.resize(partitioningTimes.size()-1);
	std::vector<double> logicRulesEventTimes = std::vector<double>{0, 4.1};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	// expected result
	EventTimeIndexer::size_array2_t subsystemsIndecesResult(logicRulesEventTimes.size()+1);
	EventTimeIndexer::int_array2_t partitionsDistResult(logicRulesEventTimes.size()+1);

	partitionsDistResult[0] = EventTimeIndexer::int_array_t{-1};
	subsystemsIndecesResult[0] = EventTimeIndexer::size_array_t{};

	partitionsDistResult[1] = EventTimeIndexer::int_array_t{0, 1, 2, 3};
	subsystemsIndecesResult[1] = EventTimeIndexer::size_array_t{0, 0, 0, 0};

	partitionsDistResult[2] = EventTimeIndexer::int_array_t{3};
	subsystemsIndecesResult[2] = EventTimeIndexer::size_array_t{1};

	// display
	std::cerr << std::endl;
	std::cerr << std::endl << "======================" << std::endl;
	logicRulesMachine.display();

	EventTimeIndexer eventTimeIndexer;
	eventTimeIndexer.set(logicRulesMachine);
	eventTimeIndexer.display();

	bool testPass = true;
	for (size_t i=0; i<eventTimeIndexer.numSubsystems(); i++) {
		if (partitionsDistResult[i] != eventTimeIndexer.partitionsDistribution(i))
			testPass = false;
		if (subsystemsIndecesResult[i] != eventTimeIndexer.subsystemIndeces(i))
			testPass = false;
	}

	ASSERT_TRUE(testPass);
}


TEST(testEventTimeIndexer, test_3)
{
	TestLogicRules<1,1> logicRules;
	LogicRulesMachine<1,1,TestLogicRules<1,1>> logicRulesMachine(logicRules);

	LogicRulesMachine<1,1,TestLogicRules<1,1>>::controller_array_t controllerStock;

	// Times
	std::vector<double> partitioningTimes{1, 2.5, 3, 5};
	controllerStock.resize(partitioningTimes.size()-1);
	std::vector<double> logicRulesEventTimes = std::vector<double>{1.0, 1.5, 2.0, 3.5, 4.0};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes, controllerStock);

	// expected result
	EventTimeIndexer::size_array2_t subsystemsIndecesResult(logicRulesEventTimes.size()+1);
	EventTimeIndexer::int_array2_t partitionsDistResult(logicRulesEventTimes.size()+1);

	partitionsDistResult[0] = EventTimeIndexer::int_array_t{-1};
	subsystemsIndecesResult[0] = EventTimeIndexer::size_array_t{};

	partitionsDistResult[1] = EventTimeIndexer::int_array_t{0};
	subsystemsIndecesResult[1] = EventTimeIndexer::size_array_t{0};

	partitionsDistResult[2] = EventTimeIndexer::int_array_t{0};
	subsystemsIndecesResult[2] = EventTimeIndexer::size_array_t{1};

	partitionsDistResult[3] = EventTimeIndexer::int_array_t{0, 1, 2};
	subsystemsIndecesResult[3] = EventTimeIndexer::size_array_t{2, 0, 0};

	partitionsDistResult[4] = EventTimeIndexer::int_array_t{2};
	subsystemsIndecesResult[4] = EventTimeIndexer::size_array_t{1};

	partitionsDistResult[5] = EventTimeIndexer::int_array_t{2};
	subsystemsIndecesResult[5] = EventTimeIndexer::size_array_t{2};

	// display
	std::cerr << std::endl;
	std::cerr << std::endl << "======================" << std::endl;
	logicRulesMachine.display();

	EventTimeIndexer eventTimeIndexer;
	eventTimeIndexer.set(logicRulesMachine);
	eventTimeIndexer.display();

	bool testPass = true;
	for (size_t i=0; i<eventTimeIndexer.numSubsystems(); i++) {

		if (partitionsDistResult[i] != eventTimeIndexer.partitionsDistribution(i))
			testPass = false;
		if (subsystemsIndecesResult[i] != eventTimeIndexer.subsystemIndeces(i))
			testPass = false;
	}

	ASSERT_TRUE(testPass);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


