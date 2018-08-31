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
 * User should override three methods namely:
 * calculateCost, calculateGradient, calculateLinearEqualityConstraint, and getSolution.
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
	 * Gets the iteration cost log.
	 *
	 * @param [out] iterationCost: The cost value in each iteration.
	 */
	void getIterationsLog(eigen_scalar_array_t& iterationCost) const;

	/**
	 * Gets a constant reference to the optimal solver's ID.
	 *
	 * @return A constant reference to the optimal solver's ID.
	 */
	const size_t& optimalSolutionID() const;

	/**
	 * Gets a constant reference to the NLP of settings.
	 *
	 * @return A constant reference to the NLP of settings.
	 */
	NLP_Settings& nlpSettings();

	/**
	 * Gets a constant reference to the number of parameters.
	 *
	 * @return A constant reference to the number of parameters.
	 */
	const size_t& numParameters() const;

	/**
	 * Gets a constant reference to the number of constraints.
	 *
	 * @return A constant reference to the number of constraints.
	 */
	const size_t& numConstraints() const;

	/**
	 * Gets a constant reference to the maximum allowed number of line-searches.
	 *
	 * @return A constant reference to the maximum allowed number of line-searches.
	 */
	const size_t& numLineSearch() const;

	/**
	 * This method runs the Frank-Wolfe algorithm which the initial parameter \f$\theta_0\f$.
	 *
	 * @param [in] initParameters: The initial parameter vector (\f$\theta_0\f$)
	 */
	void run(const dynamic_vector_t& initParameters);

protected:
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
	 * Calculates the coefficients of the linear inequality constraints. \n
	 * \f$ C_m \theta + D_v < 0\f$
	 *
	 * @param [out] Cm: The \f$ C_m\f$ matrix.
	 * @param [out] Dv: THe \f$ D_v \f$ vector.
	 */
	virtual void calculateLinearInequalityConstraint(
			dynamic_matrix_t& Cm,
			dynamic_vector_t& Dv);

	/**
	 * Calculates the cost function.
	 *
	 * @param [in] id: Solver ID
	 * @param [in] parameters: The current parameter vector.
	 * @param [in] cost: The cost for the current parameter vector.
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

	/**
	 * Gets the solution ID.
	 *
	 * @param [in] idStar: The ID of the best solution.
	 */
	virtual void getSolution(size_t idStar)  {}

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
	 * Sets the linear equality and active inequality to GLPK.
	 *
	 * @param [in] parameters: The current parameter vector.
	 * @param [out] numActiveConstraints: The number of active constraints.
	 */
	void setConstraints(
			const dynamic_vector_t& parameters,
			size_t& numActiveConstraints);

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

private:
	/*
	 * Variables
	 */
	NLP_Settings nlpSettings_;

	dynamic_matrix_t linearEqualityConstraintAm_;
	dynamic_vector_t linearEqualityConstraintBv_;
	dynamic_matrix_t linearInequalityConstraintCm_;
	dynamic_vector_t linearInequalityConstraintDv_;

	size_t numParameters_;
	size_t numLineSearch_;
	size_t numActiveConstraints_;


	SCALAR_T optimizedCost_;
	size_t optimizedID_;
	dynamic_vector_t optimizedParameters_;
	dynamic_vector_t optimizedGradient_;
	size_t numFuntionCall_;

	eigen_scalar_array_t iterationCost_;

private:
	glp_prob* lpPtr_;
};

}  // end of nlp namespace
}  // end of ocs2 namespace

#include "implementation/GradientDescent.h"

#endif /* OCS2_GRADIENTDESCENT_OCS2_H_ */
