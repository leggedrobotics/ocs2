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
 * This structure contains the settings for the gradient-descent algorithm.
 */
struct NlpOptions
{
public:
	NlpOptions() {}

	/** This value determines to display the log output.*/
	bool displayGradientDescent_ = true;
	/** This value determines the maximum number of algorithm iterations.*/
	size_t maxIterations_ = 1000;
	/** This value determines the termination condition based on the minimum relative changes of the cost.*/
	double minRelCost_    = 1e-6;
	/** This value determines the maximum step size for the line search scheme.*/
	double maxLearningRate_ = 1.0;
	/** This value determines the minimum step size for the line search scheme.*/
	double minLearningRate_ = 0.05;
	/**
	 * This value determines the line search scheme to be used. \n
	 * - \b Ascending: The step size eventually increases from the minimum value to the maximum. \n
	 * - \b Descending: The step size eventually decreases from the minimum value to the maximum.
	 * */
	bool useAscendingLineSearchNLP_ = true;
	/** This value determines the minimum allowable difference between to consecutive switching times.*/
	double minDisToBoundary_ = 0.01;
};

/**
 * This class implements the Frank-Wolfe algorithm which is an iterative first-order gradient descent
 * algorithm. For more discussion on this algorithm, the reader should refer to
 * \cite jaggi2013revisiting
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
	 * This is the default constructor.
	 */
	GradientDescent() {
		adjustOptions();
		CleanFmtDisplay_ = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");
	}

	/**
	 * This is the default destructor.
	 */
	~GradientDescent() {}

	/**
	 * Gets the cost.
	 * @param [out] cost value
	 */
	void getCost(double& cost) { cost = cost_; }

	/**
	 * Gets the parameter vector.
	 *
	 * @tparam Derived
	 * @param [out] parameters: the parameter vector
	 */
	template <typename Derived>
	void getParameters(Eigen::MatrixBase<Derived>& parameters) const { parameters = parameters_; }

	/**
	 * This method runt the Frank-Wolfe algorithm which the initial parameter \f$\theta_0\f$.
	 *
	 * @param [in] initParameters: The initial parameter vector (\f$\theta_0\f$)
	 */
	void run(const Eigen::VectorXd& initParameters);

	/**
	 * Calculates the coefficients of the linear equality constraints. \n
	 * \f$ A_m \theta + B_v = 0\f$
	 *
	 * @param [out] Am: The \f$ A_m\f$ matrix.
	 * @param [out] Bv: THe \f$ B_v \f$ vector.
	 */
	virtual void calculateLinearEqualityConstraint(Eigen::MatrixXd& Am, Eigen::VectorXd& Bv)  {
		Am.resize(0,numParameters_);
		Bv.resize(0);
	}

	/**
	 * Calculates the gradient direction.
	 *
	 * @param [in] id: solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [in] gradient: Gradient at the current parameter vector.
	 * @return The success flag.
	 */
	virtual bool calculateGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient) {
		bool status = calculateNumericalGradient(id, parameters, gradient);
		return status;
	}

	/**
	 * Gets the solution ID.
	 *
	 * @param [in] idStar: The ID of the best solution.
	 */
	virtual void getSolution(size_t idStar)  {}

	/**
	 * Gets the iteration cost log.
	 *
	 * @param [out] iterationCost: The cost value in each iteration.
	 */
	void getIterationsLog(eigen_scalar_array_t& iterationCost) const { iterationCost = iterationCost_; }

	/**
	 * Calculates the cost.
	 *
	 * @param [in] id: Solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [out] cost: Gradient at the current parameter vector.
	 * @return The success flag.
	 */
	virtual bool calculateCost(const size_t& id, const Eigen::VectorXd& parameters, double& cost) = 0;


protected:
	/**
	 * Calculate the Frank Wolfe gradient.
	 *
	 * @param [in] id: solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [out] gradient: The Frank-Wolfe gradient at the current parameter vector.
	 */
	void frankWolfeGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient);

	/**
	 * Line search to find the best learning rate using ascending scheme where the step size eventually increases
	 * from the minimum value to the maximum.
	 *
	 * @param [in] gradient: The current gradient.
	 * @param [out] learningRateStar: The best learning rate.
	 */
	void ascendingLineSearch(const Eigen::VectorXd& gradient, double& learningRateStar);

	/**
	 * Line search to find the best learning rate using decreasing scheme where the step size eventually decreases
	 * from the maximum value to the minimum.
	 *
	 * @param [in] gradient: The current gradient.
	 * @param [out] learningRateStar: The best learning rate.
	 */
	void decreasingLineSearch(const Eigen::VectorXd& gradient, double& learningRateStar);

	/**
	 * Set up the Linear Programming problem in the Frank-Wolfe algorithm.
	 */
	void setupLP();

	/**
	 * Calculate the gradient numerically
	 *
	 * @param [in] id: Solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [out] gradient: Gradient at the current parameter vector.
	 * @return The success flag.
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
