/*
 * QuadraticTest.cpp
 *
 *  Created on: Jul 11, 2016
 *      Author: farbod
 */

#include <iostream>

#include "ocs2_frank_wolfe/GradientDescent.h"

using namespace nlp;

class QuadraticGradientDescent : public GradientDescent
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

		Am.resize(2*numParameters_,numParameters_);
		Bm.resize(2*numParameters_);

		Am.topRows(numParameters_)    =  Eigen::MatrixXd::Identity(numParameters_, numParameters_);
		Am.bottomRows(numParameters_) = -Eigen::MatrixXd::Identity(numParameters_, numParameters_);

		Bm.head(numParameters_) = -maxX * Eigen::VectorXd::Ones(numParameters_);
		Bm.tail(numParameters_) =  minX * Eigen::VectorXd::Ones(numParameters_);
	}

};


int main( int argc, char* argv[] )
{

	QuadraticGradientDescent quadraticGradientDescent;

	Eigen::Vector2d parameters = 2*Eigen::Vector2d::Ones();

	quadraticGradientDescent.run(parameters);

	double cost;
	quadraticGradientDescent.getCost(cost);
	quadraticGradientDescent.getParameters(parameters);

	std::cout << "cost: " << cost << std::endl;
	std::cout << "parameters: " << parameters.transpose() << std::endl;
}
