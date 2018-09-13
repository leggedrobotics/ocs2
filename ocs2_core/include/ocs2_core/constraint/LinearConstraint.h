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

#ifndef LINEARCONSTRAINT_H_
#define LINEARCONSTRAINT_OCS2_H_

#include "ocs2_core/constraint/ConstraintBase.h"

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class LinearConstraint : public ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<LinearConstraint<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const LinearConstraint<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef ConstraintBase<STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::scalar_t 				scalar_t;
	typedef typename BASE::state_vector_t   		state_vector_t;
	typedef typename BASE::input_vector_t	 		input_vector_t;
	typedef typename BASE::state_matrix_t   		state_matrix_t;
	typedef typename BASE::state_input_matrix_t 	state_input_matrix_t;
	typedef typename BASE::constraint1_vector_t 	constraint1_vector_t;
	typedef typename BASE::constraint2_vector_t 	constraint2_vector_t;
	typedef typename BASE::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename BASE::constraint1_input_matrix_t constraint1_input_matrix_t;
	typedef typename BASE::constraint2_state_matrix_t   constraint2_state_matrix_t;

	LinearConstraint(
			const size_t& numStateInputConstraint,
			const constraint1_vector_t& e,
			const constraint1_state_matrix_t& C,
			const constraint1_input_matrix_t& D,
			const size_t& numStateOnlyConstraint,
			const constraint2_vector_t& h,
			const constraint2_state_matrix_t& F,
			const size_t& numStateOnlyFinalConstraint,
			const constraint2_vector_t& h_f,
			const constraint2_state_matrix_t& F_f)

	: numStateInputConstraint_(numStateInputConstraint)
	, e_(e)
	, C_(C)
	, D_(D)
	, numStateOnlyConstraint_(numStateOnlyConstraint)
	, h_(h)
	, F_(F)
	, numStateOnlyFinalConstraint_(numStateOnlyFinalConstraint)
	, h_f_(h_f)
	, F_f_(F_f)
	{}

	virtual ~LinearConstraint() = default;

	/**
	 * Returns pointer to the base class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual LinearConstraint<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const override {

		return new LinearConstraint<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
	}

	/**
	 * Sets the current time, state, and control input.
	 *
	 * @param [in] t: Current time
	 * @param [in] x: Current state vector
	 * @param [in] u: Current input vector
	 */
	virtual void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) override {

		BASE::setCurrentStateAndControl(t, x, u);
	}

	/**
	 * Computes the state-input equality constraints.
	 *
	 * @param [out] g1: The state-input equality constraints value.
	 */
	virtual void getConstraint1(
			constraint1_vector_t& g1) override {

		g1 = e_ + C_*BASE::x_ + D_*BASE::u_;
	}

	/**
	 * Get the number of state-input active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-input active equality constraints.
	 */
	virtual size_t numStateInputConstraint(const scalar_t& time) override {

		return numStateInputConstraint_;
	}

	/**
	 * Compute the state-only equality constraints.
	 *
	 * @param [out] g2: The state-only equality constraints value.
	 */
	virtual void getConstraint2(
			constraint2_vector_t& g2) override {

		g2 = h_ + F_*BASE::x_;
	}

	/**
	 * Get the number of state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-only active equality constraints.
	 */
	virtual size_t numStateOnlyConstraint(const scalar_t& time) {

		return numStateOnlyConstraint_;
	}

	/**
	 * Compute the final state-only equality constraints.
	 *
	 * @param [out] g2Final: The final state-only equality constraints value.
	 */
	virtual void getFinalConstraint2(
			constraint2_vector_t& g2Final) override {

		g2Final = h_f_ + F_f_*BASE::x_;
	}

	/**
	 * Get the number of final state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of final state-only active equality constraints.
	 */
	virtual size_t numStateOnlyFinalConstraint(const scalar_t& time) {

		return numStateOnlyFinalConstraint_;
	}

	/**
	 * The C matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] C: \f$ C(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesState(
			constraint1_state_matrix_t& C) override {

		C = C_;
	}

	/**
	 * The D matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] D: \f$ D(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesControl(
			constraint1_input_matrix_t& D) override {

		D = D_;
	}

	/**
	 * The F matrix at a given operating point for the linearized state-only constraints,
	 * \f$ F(t) \delta x + h(t) = 0 \f$.
	 *
	 * @param [out] F: \f$ F(t) \f$ matrix.
	 */
	virtual void getConstraint2DerivativesState(
			constraint2_state_matrix_t& F) override {

		F = F_;
	}

	/**
	 * The F matrix at a given operating point for the linearized terminal state-only constraints,
	 * \f$ F_f(t) \delta x + h_f(t) = 0 \f$.
	 *
	 * @param [out] F_f: \f$ F_f(t) \f$ matrix.
	 */
	virtual void getFinalConstraint2DerivativesState(
			constraint2_state_matrix_t& F_f) override {

		F_f = F_f_;
	}

private:
	size_t numStateInputConstraint_;
	constraint1_vector_t 			e_;
	constraint1_state_matrix_t 		C_;
	constraint1_input_matrix_t 	D_;
	size_t numStateOnlyConstraint_;
	constraint2_vector_t 			h_;
	constraint2_state_matrix_t 		F_;
	size_t numStateOnlyFinalConstraint_;
	constraint2_vector_t 			h_f_;
	constraint2_state_matrix_t 		F_f_;
};

} // namespace ocs2

#endif /* LINEARCONSTRAINT_H_ */
