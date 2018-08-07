/*
 * MatyasTest.cpp
 *
 *  Created on: Jul 11, 2016
 *      Author: farbod
 */



#include <iostream>
#include <gtest/gtest.h>

#include "ocs2_frank_wolfe/GradientDescent.h"

using namespace ocs2;
using namespace nlp;

/*
 * refer to: https://en.wikipedia.org/wiki/Test_functions_for_optimization
 */
class MatyasGradientDescent : public GradientDescent<double>
{
public:

	MatyasGradientDescent() {}
	~MatyasGradientDescent() {}

	bool calculateCost(const size_t& id, const Eigen::VectorXd& parameters, double& cost) override {

		cost = 0.26 * (pow(parameters(0),2) + pow(parameters(1),2)) - 0.48 * parameters(0) * parameters(1);
		return true;
	}

	void calculateLinearEqualityConstraint(Eigen::MatrixXd& Am, Eigen::VectorXd& Bm) override {
		const double maxX = 10.0;
		const double minX = -10.0;

		Am.resize(2*numParameters(),numParameters());
		Bm.resize(2*numParameters());

		Am.topRows(numParameters())    =  Eigen::MatrixXd::Identity(numParameters(), numParameters());
		Am.bottomRows(numParameters()) = -Eigen::MatrixXd::Identity(numParameters(), numParameters());

		Bm.head(numParameters()) = -maxX * Eigen::VectorXd::Ones(numParameters());
		Bm.tail(numParameters()) =  minX * Eigen::VectorXd::Ones(numParameters());

		std::cout << "A\n" << Am << std::endl;
		std::cout << "B\n" << Bm << std::endl;
	}

};

TEST(MatyasTest, MatyasTest)
{

	MatyasGradientDescent matyasGradientDescent;

	Eigen::Vector2d parameters = 2*Eigen::Vector2d::Ones();

	matyasGradientDescent.run(parameters);

	double cost;
	matyasGradientDescent.getCost(cost);
	matyasGradientDescent.getParameters(parameters);

	std::cout << "cost: " << cost << std::endl;
	std::cout << "parameters: " << parameters.transpose() << std::endl;

	const double optimalCost = 0.0;
	const Eigen::Vector2d optimalParameters = Eigen::Vector2d::Ones();

	ASSERT_NEAR(cost, optimalCost, 1e-3) <<
			"MESSAGE: Frank_Wolfe failed in the Quadratic test!";
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


