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

	void calculateLinearInequalityConstraint(Eigen::MatrixXd& Cm, Eigen::VectorXd& Dv) override {
		const double maxX = 3.0;
		const double minX = 1.0;

		Cm.resize(2*numParameters(),numParameters());
		Dv.resize(2*numParameters());

		Cm.topRows(numParameters())    =  Eigen::MatrixXd::Identity(numParameters(), numParameters());
		Cm.bottomRows(numParameters()) = -Eigen::MatrixXd::Identity(numParameters(), numParameters());

		Dv.head(numParameters()) = -maxX * Eigen::VectorXd::Ones(numParameters());
		Dv.tail(numParameters()) =  minX * Eigen::VectorXd::Ones(numParameters());

		std::cout << "C\n" << Cm << std::endl;
		std::cout << "D\n" << Dv << std::endl;
	}

};

TEST(QuadraticTest, QuadraticTest)
{

	QuadraticGradientDescent quadraticGradientDescent;

	quadraticGradientDescent.nlpSettings().displayGradientDescent_ = true;
	quadraticGradientDescent.nlpSettings().maxIterations_ 	 = 3;
	quadraticGradientDescent.nlpSettings().minRelCost_    	 = 1e-6;
	quadraticGradientDescent.nlpSettings().maxLearningRate_  = 1.0;
	quadraticGradientDescent.nlpSettings().minLearningRate_  = 0.05;
	quadraticGradientDescent.nlpSettings().minDisToBoundary_ = 0.0;
	quadraticGradientDescent.nlpSettings().useAscendingLineSearchNLP_ = false;

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

