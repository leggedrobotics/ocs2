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

#ifndef FRANK_WOLFE_OCS2_H_
#define FRANK_WOLFE_OCS2_H_

#include <memory>

// GNU Linear Programming Kit
#include <glpk.h>

#include <ocs2_core/Dimensions.h>

namespace ocs2 {

/**
 * This class implements the Frank-Wolfe algorithm for computing the gradient
 * respecting linear equalities and inequalities. For more discussion on this
 * algorithm, the reader should refer to \cite jaggi13 .
 */
class FrankWolfeGradient
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using DIMENSIONS = Dimensions<0, 0>;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;
	using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
	using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;

	/**
	 * Constructor.
	 */
	FrankWolfeGradient();

	/**
	 * Default destructor.
	 */
	~FrankWolfeGradient() = default;

	/**
	 * Calculates the coefficients of the linear equality constraints. \n
	 * \f$ A_m \theta + B_v = 0\f$
	 *
	 * @param [out] Am: The \f$ A_m\f$ matrix.
	 * @param [out] Bv: THe \f$ B_v \f$ vector.
	 */
	virtual void calculateLinearEqualityConstraint(
			dynamic_matrix_t& Am,
			dynamic_vector_t& Bv) = 0;

	/**
	 * Calculates the coefficients of the linear inequality constraints. \n
	 * \f$ C_m \theta + D_v < 0\f$
	 *
	 * @param [out] Cm: The \f$ C_m\f$ matrix.
	 * @param [out] Dv: THe \f$ D_v \f$ vector.
	 */
	virtual void calculateLinearInequalityConstraint(
			dynamic_matrix_t& Cm,
			dynamic_vector_t& Dv) = 0;

	/**
	 * Calculates the Frank-Wolfe gradient.
	 *
	 * @param [in] parameter: The value of parameter vector.
	 * @param [in] gradient: The gradient at the current parameter vector.
	 * @param [out] fwGradient: The Frank-Wolfe gradient at the current parameter vector.
	 */
	void getFrankWolfeGradient(
			const dynamic_vector_t& parameter,
			const dynamic_vector_t& gradient,
			dynamic_vector_t& fwGradient);

private:
	/**
	 * Instantiates GLPK solver
	 */
	void instantiateGLPK();

	/**
	 * Sets up Frank-Wolfe linear program.
	 */
	void setupLP(
			const scalar_array_t& parameter,
			const dynamic_vector_t& gradient);

	/***********
	 * Variables
	 **********/
	std::unique_ptr<glp_prob, void(*)(glp_prob*)> lpPtr_;

};

}  // namespace ocs2

#include "implementation/FrankWolfeGradient.h"

#endif /* FRANK_WOLFE_OCS2_H_ */
