/*
 * GradientDescent.h
 *
 *  Created on: Jul 10, 2016
 *      Author: farbod
 */

#ifndef OCS2_GRADIENTDESCENT_OCS2_H_
#define OCS2_GRADIENTDESCENT_OCS2_H_

#include <vector>
#include <limits>
#include <cmath>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

// GNU Linear Programming Kit
#include <glpk.h>

#include <ocs2_frank_wolfe/NLP_Settings.h>

namespace ocs2 {
namespace nlp {


/**
 * This class implements the Frank-Wolfe algorithm which is an iterative first-order gradient descent
 * algorithm. For more discussion on this algorithm, the reader should refer to
 * \cite jaggi13 .
 *
 * @tparam SCALAR_T: Floating point type.
 */
template <typename SCALAR_T=double>
class GradientDescent
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<GradientDescent> Ptr;
	const SCALAR_T plusInf_  =  1e12;
	const SCALAR_T minusInf_ = -1e12;

	typedef SCALAR_T scalar_t;
	typedef Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>              dynamic_vector_t;
	typedef Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic> dynamic_matrix_t;
	typedef Eigen::Matrix<SCALAR_T, 1, 1>                                         eigen_scalar_t;
	typedef std::vector<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t>> eigen_scalar_array_t;

	/**
	 * Default constructor.
	 */
	GradientDescent();

	/**
	 * Default destructor.
	 */
	virtual ~GradientDescent() = default;

	/**
	 * Gets the cost.
	 * @param [out] cost value
	 */
	void getCost(SCALAR_T& cost);

	/**
	 * Gets the parameter vector.
	 *
	 * @tparam Derived
	 * @param [out] parameters: the parameter vector
	 */
	template <typename Derived>
	void getParameters(
			Eigen::MatrixBase<Derived>& parameters) const;

	/**
	 * Calculates the coefficients of the linear equality constraints. \n
	 * \f$ A_m \theta + B_v = 0\f$
	 *
	 * @param [out] Am: The \f$ A_m\f$ matrix.
	 * @param [out] Bv: THe \f$ B_v \f$ vector.
	 */
	virtual void calculateLinearEqualityConstraint(
			dynamic_matrix_t& Am,
			dynamic_vector_t& Bv);

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
	void getIterationsLog(eigen_scalar_array_t& iterationCost) const;

	/**
	 * This method runs the Frank-Wolfe algorithm which the initial parameter \f$\theta_0\f$.
	 *
	 * @param [in] initParameters: The initial parameter vector (\f$\theta_0\f$)
	 */
	void run(const dynamic_vector_t& initParameters);

	/**
	 * Calculates the cost.
	 *
	 * @param [in] id: Solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [out] cost: Gradient at the current parameter vector.
	 * @return boolean: The success flag.
	 */
	virtual bool calculateCost(
			const size_t& id,
			const dynamic_vector_t& parameters,
			SCALAR_T& cost) = 0;

	/**
	 * Calculates the gradient direction.
	 *
	 * @param [in] id: solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [in] gradient: Gradient at the current parameter vector.
	 * @return boolean: The success flag.
	 */
	virtual bool calculateGradient(
			const size_t& id,
			const dynamic_vector_t& parameters,
			dynamic_vector_t& gradient);

	NLP_Settings& nlpSettings() {

		return nlpSettings_;
	}

	size_t& numParameters() {

		return numParameters_;
	}

	size_t& numConstraints() {

		return numConstraints_;
	}

protected:
	/**
	 * Calculate the Frank Wolfe gradient.
	 *
	 * @param [in] id: solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [out] gradient: The Frank-Wolfe gradient at the current parameter vector.
	 */
	void frankWolfeGradient(
			const size_t& id,
			const dynamic_vector_t& parameters,
			dynamic_vector_t& gradient);

	/**
	 * Line search to find the best learning rate using ascending scheme where the step size eventually increases
	 * from the minimum value to the maximum.
	 *
	 * @param [in] gradient: The current gradient.
	 * @param [out] learningRateStar: The best learning rate.
	 */
	void ascendingLineSearch(
			const dynamic_vector_t& gradient,
			SCALAR_T& learningRateStar);

	/**
	 * Line search to find the best learning rate using decreasing scheme where the step size eventually decreases
	 * from the maximum value to the minimum.
	 *
	 * @param [in] gradient: The current gradient.
	 * @param [out] learningRateStar: The best learning rate.
	 */
	void decreasingLineSearch(
			const dynamic_vector_t& gradient,
			SCALAR_T& learningRateStar);

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
	 * @return boolean: The success flag.
	 */
	bool calculateNumericalGradient(
			const size_t& id,
			const dynamic_vector_t& parameters,
			dynamic_vector_t& gradient);

	/**
	 * Updates the learning rate and line search
	 */
	void adjustOptions();

	/**
	 * Formatting eigen display
	 */
	Eigen::IOFormat CleanFmtDisplay_;

protected:
	/*
	 * Variables
	 */
	NLP_Settings nlpSettings_;

	dynamic_matrix_t linearEqualityConstraintAm_;
	dynamic_vector_t linearEqualityConstraintBv_;
	dynamic_vector_t linearEqualityConstraintGv_;
	dynamic_matrix_t linearEqualityConstraintAmNull_;

	SCALAR_T cost_;
	size_t numParameters_;
	size_t numLineSearch_;
	size_t numConstraints_;
	dynamic_vector_t parameters_;
	dynamic_vector_t gradient_;
	size_t id_;
	size_t numFuntionCall_ = 0;

	eigen_scalar_array_t iterationCost_;

private:
	glp_prob *lpPtr_;
};

}  // end of nlp namespace
}  // end of ocs2 namespace

#include "implementation/GradientDescent.h"

#endif /* OCS2_GRADIENTDESCENT_OCS2_H_ */
