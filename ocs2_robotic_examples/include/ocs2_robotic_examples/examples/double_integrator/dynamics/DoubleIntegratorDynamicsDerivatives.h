/*
 * DoubleIntegratorDynamicsDerivatives.h
 *
 *  Created on: Jul 6, 2018
 *      Author: farbod
 */

#ifndef DOUBLE_INTEGRATOR_DYNAMICS_DERIVATIVES_H
#define DOUBLE_INTEGRATOR_DYNAMICS_DERIVATIVES_H

#include <ocs2_core/dynamics/DerivativesBase.h>
#include "ocs2_robotic_examples/examples/double_integrator/definitions.h"

namespace ocs2{

class DoubleIntegratorDynamicsDerivatives : public DerivativesBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<DoubleIntegratorDynamicsDerivatives> Ptr;
	typedef std::shared_ptr<const DoubleIntegratorDynamicsDerivatives> ConstPtr;

	typedef DerivativesBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> BASE;
	typedef typename BASE::scalar_t               scalar_t;
	typedef typename BASE::state_vector_t         state_vector_t;
	typedef typename BASE::state_matrix_t         state_matrix_t;
	typedef typename BASE::input_vector_t         input_vector_t;
	typedef typename BASE::state_input_matrix_t  	state_input_matrix_t;

	state_matrix_t A_;
	state_input_matrix_t B_;

	DoubleIntegratorDynamicsDerivatives(state_matrix_t A, state_input_matrix_t B)
	: A_(A)
	, B_(B)
	{}

	~DoubleIntegratorDynamicsDerivatives() = default;

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual DoubleIntegratorDynamicsDerivatives* clone() const {
		return new DoubleIntegratorDynamicsDerivatives(*this);
	}

	/**
	 * Sets the current time, state, and control input.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [in] u: Current input.
	 */
	virtual void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) {

		// BASE class method
		BASE::setCurrentStateAndControl(t, x, u);

	}

	/**
	 * TODO
	 * @param A
	 */
	void getFlowMapDerivativeState(
			state_matrix_t& A) {

		A = A_;
	}

	/**
	 * TODO
	 * @param B
	 */
	void getFlowMapDerivativeInput(
			state_input_matrix_t& B) {

		B = B_;
	}
};

} // namespace ocs2

#endif /* DOUBLE_INTEGRATOR_DYNAMICS_DERIVATIVES_H */
