/*
 * GradientDescent.h
 *
 *  Created on: Jul 10, 2016
 *      Author: farbod
 */

#ifndef OCS2_GRADIENTDESCENT_H_
#define OCS2_GRADIENTDESCENT_H_

#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#include <glpk.h>

namespace nlp {
/**
 * Settings for the NLP
 */
struct NlpOptions
{
public:
	NlpOptions() {}

	bool displayGradientDescent_ = true;
	size_t maxIterations_ = 1000;
	double minRelCost_    = 1e-6;
	double maxLearningRate_ = 1.0;
	double minLearningRate_ = 0.05;
	double minDisToBoundary_ = 0.01;
	bool useAscendingLineSearchNLP_ = true;
};

/**
 * Gradient Descent class
 */
class GradientDescent
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<GradientDescent> Ptr;
	const double plusInf_  =  1e12;
	const double minusInf_ = -1e12;
	typedef std::vector<Eigen::Matrix<double,1,1>, Eigen::aligned_allocator<Eigen::Matrix<double, 1, 1>> > eigen_scalar_array_t;

	/**
	 * Constructor
	 */
	GradientDescent() {
		adjustOptions();
		CleanFmtDisplay_ = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");
	}

	~GradientDescent() {}

	/**
	 * Gets the cost
	 * @param [out] cost
	 */
	void getCost(double& cost) { cost = cost_; }

	/**
	 * gets the parameters
	 * @tparam Derived
	 * @param [out] parameters
	 */
	template <typename Derived>
	void getParameters(Eigen::MatrixBase<Derived>& parameters) const { parameters = parameters_; }

	/**
	 *
	 * @param [in] initParameters
	 */
	void run(const Eigen::VectorXd& initParameters);

	/**
	 * Calculates the linear equality constraint
	 * @param [out] Am
	 * @param [out] Bv
	 */
	virtual void calculateLinearEqualityConstraint(Eigen::MatrixXd& Am, Eigen::VectorXd& Bv)  {
		Am.resize(0,numParameters_);
		Bv.resize(0);
	}

	/**
	 * Calculates the gradient
	 * @param [in] id
	 * @param [in] parameters
	 * @param [in] gradient
	 * @return boolean
	 */
	virtual bool calculateGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient) {
		bool status = calculateNumericalGradient(id, parameters, gradient);
		return status;
	}

	/**
	 * Gets the solution
	 * @param [in] idStar
	 */
	virtual void getSolution(size_t idStar)  {}

	/**
	 * Gets the iteration log
	 * @param [out] iterationCost
	 */
	void getIterationsLog(eigen_scalar_array_t& iterationCost) const { iterationCost = iterationCost_; }

	/**
	 * Calculates the cost
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] cost
	 * @return boolean
	 */
	virtual bool calculateCost(const size_t& id, const Eigen::VectorXd& parameters, double& cost) = 0;


protected:
	/**
	 * Calculate the Frank Wolfe gradient
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] gradient
	 */
	void frankWolfeGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient);

	/**
	 * Line search to find the best learningStep
	 * @param [in] gradient
	 * @param [out] learningRateStar
	 */
	void ascendingLineSearch(const Eigen::VectorXd& gradient, double& learningRateStar);

	/**
	 * Line search to find the best learningStep
	 * @param [in] gradient
	 * @param [out] learningRateStar
	 */
	void decreasingLineSearch(const Eigen::VectorXd& gradient, double& learningRateStar);

	/**
	 * Initializes LP
	 */
	void setupLP();

	/**
	 * Calculates the numerical gradient
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] gradient
	 * @return boolean
	 */
	bool calculateNumericalGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient);

	/**
	 * Updates the learning rate and line search
	 */
	void adjustOptions();

	/*
	 * Variables
	 */
	NlpOptions nlpOptions_;

	Eigen::MatrixXd linearEqualityConstraintAm_;
	Eigen::VectorXd linearEqualityConstraintBv_;
	Eigen::VectorXd linearEqualityConstraintGv_;
	Eigen::MatrixXd linearEqualityConstraintAmNull_;

	double cost_;
	size_t numParameters_;
	size_t numLineSearch_;
	size_t numConstraints_;
	Eigen::VectorXd parameters_;
	Eigen::VectorXd gradient_;
	size_t id_;
	size_t numFuntionCall_ = 0;

	eigen_scalar_array_t iterationCost_;

	Eigen::IOFormat CleanFmtDisplay_;

private:
	glp_prob *lpPtr_;
};

}  // end of nlp namespace

#endif /* OCS2_GRADIENTDESCENT_H_ */
