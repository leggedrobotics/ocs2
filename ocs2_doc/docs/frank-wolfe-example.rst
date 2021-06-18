.. index:: pair: page; OCS2 Frank Wolfe Example
.. _doxid-ocs2_frank_wolfe_example:

OCS2 Frank Wolfe Example
========================



.. _doxid-ocs2_frank_wolfe_example_1ocs2_frank_wolfe_matyastest:

Matyas Test
~~~~~~~~~~~

You can use the following code to run the Matyas Test:

.. code-block:: cpp

	#include <iostream>
	
	#include "ocs2_frank_wolfe/GradientDescent.h"
	
	using namespace nlp;
	
	/*
	 * refer to: https://en.wikipedia.org/wiki/Test_functions_for_optimization
	 */
	class MatyasGradientDescent : public GradientDescent
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
	
	        MatyasGradientDescent matyasGradientDescent;
	
	        Eigen::Vector2d parameters = 2*Eigen::Vector2d::Ones();
	
	        matyasGradientDescent.run(parameters);
	
	        double cost;
	        matyasGradientDescent.getCost(cost);
	        matyasGradientDescent.getParameters(parameters);
	
	        std::cout << "cost: " << cost << std::endl;
	        std::cout << "parameters: " << parameters.transpose() << std::endl;
	}





.. _doxid-ocs2_frank_wolfe_example_1ocs2_frank_wolfe_quadratictest:

Quadratic Test
~~~~~~~~~~~~~~

You can use the following code to run the Quadratic Test:

.. code-block:: cpp

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

