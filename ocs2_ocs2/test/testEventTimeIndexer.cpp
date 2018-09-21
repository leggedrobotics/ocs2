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

#include <ocs2_ocs2/EventTimeIndexer.h>

#include <gtest/gtest.h>

using namespace ocs2;

class TestLogicRules : public LogicRulesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase BASE;
	typedef LogicRulesBase::scalar_t scalar_t;
	typedef LogicRulesBase::scalar_array_t scalar_array_t;
	typedef LogicRulesBase::size_array_t size_array_t;

	TestLogicRules() {}

	virtual ~TestLogicRules() {}

	void set(const scalar_array_t& eventTimes) {
		eventTimes_ = eventTimes;
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
	TestLogicRules logicRules;
	LogicRulesMachine<TestLogicRules> logicRulesMachine(logicRules);

	// Times
	std::vector<double> partitioningTimes{0.5,1.5,2.5};
	std::vector<double> logicRulesEventTimes = std::vector<double>{0.25, 0.75, 1.25, 1.75, 2.25, 2.75};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

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
	TestLogicRules logicRules;
	LogicRulesMachine<TestLogicRules> logicRulesMachine(logicRules);

	// Times
	std::vector<double> partitioningTimes{1, 2, 3};
	std::vector<double> logicRulesEventTimes = std::vector<double>{0, 1, 2, 3, 4};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

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
	TestLogicRules logicRules;
	LogicRulesMachine<TestLogicRules> logicRulesMachine(logicRules);

	// Times
	std::vector<double> partitioningTimes{1, 2, 3, 4, 5};
	std::vector<double> logicRulesEventTimes = std::vector<double>{0, 4.1};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

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
	TestLogicRules logicRules;
	LogicRulesMachine<TestLogicRules> logicRulesMachine(logicRules);

	// Times
	std::vector<double> partitioningTimes{1, 2.5, 3, 5};
	std::vector<double> logicRulesEventTimes = std::vector<double>{1.0, 1.5, 2.0, 3.5, 4.0};

	// Set logic
	logicRules.set(logicRulesEventTimes);
	logicRulesMachine.setLogicRules(logicRules);
	logicRulesMachine.updateLogicRules(partitioningTimes);

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


