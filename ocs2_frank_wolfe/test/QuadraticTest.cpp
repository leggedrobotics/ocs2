/*
 * QuadraticTest.cpp
 *
 *  Created on: Jul 11, 2016
 *      Author: farbod
 */

#include <iostream>
#include <gtest/gtest.h>

#include "ocs2_frank_wolfe/GradientDescent.h"

using namespace ocs2;
using namespace nlp;

class QuadraticGradientDescent : public GradientDescent<double>
{
public:

	QuadraticGradientDescent() {}
	~QuadraticGradientDescent() {}

	bool calculateCost(const size_t& id, const Eigen::VectorXd& parameters, double& cost) override {

		cost = 0.5 * parameters.squaredNorm();
		return true;
	}

	void calculateLinearEqualityConstraint(Eigen::MatrixXd& Am, Eigen::VectorXd& Bm) override {
		const double maxX = 3.0;
		const double minX = 1.0;

		Am.resize(2*numParameters(),numParameters());
		Bm.resize(2*numParameters());

		Am.topRows(numParameters())    =  Eigen::MatrixXd::Identity(numParameters(), numParameters());
		Am.bottomRows(numParameters()) = -Eigen::MatrixXd::Identity(numParameters(), numParameters());

		Bm.head(numParameters()) = -maxX * Eigen::VectorXd::Ones(numParameters());
		Bm.tail(numParameters()) =  minX * Eigen::VectorXd::Ones(numParameters());

//		std::cout << "A\n" << Am << std::endl;
//		std::cout << "B\n" << Bm << std::endl;
	}

};

TEST(QuadraticTest, QuadraticTest)
{

	QuadraticGradientDescent quadraticGradientDescent;

	Eigen::Vector2d parameters = 2*Eigen::Vector2d::Ones();

	quadraticGradientDescent.run(parameters);

	double cost;
	quadraticGradientDescent.getCost(cost);
	quadraticGradientDescent.getParameters(parameters);

	std::cout << "cost: " << cost << std::endl;
	std::cout << "parameters: " << parameters.transpose() << std::endl;

	const double optimalCost = 1.0;
	const Eigen::Vector2d optimalParameters = Eigen::Vector2d::Ones();

	ASSERT_NEAR(cost, optimalCost, 1e-3) <<
			"MESSAGE: Frank_Wolfe failed in the Quadratic test!";
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

