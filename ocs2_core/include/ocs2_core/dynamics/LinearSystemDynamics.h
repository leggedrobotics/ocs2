/*
 * LinearSystemDynamics.h
 *
 *  Created on: May 2, 2018
 *      Author: farbod
 */

#ifndef LINEARSYSTEMDYNAMICS_OCS2_H_
#define LINEARSYSTEMDYNAMICS_OCS2_H_

#include "ocs2_core/dynamics/DerivativesBase.h"

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=ocs2::NullLogicRules<STATE_DIM,INPUT_DIM>>
class LinearSystemDynamics : public DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<LinearSystemDynamics<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const LinearSystemDynamics<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef DerivativesBase<STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::scalar_t       			scalar_t;
	typedef typename BASE::state_vector_t 			state_vector_t;
	typedef typename BASE::state_matrix_t 			state_matrix_t;
	typedef typename BASE::input_vector_t  			input_vector_t;
	typedef typename BASE::state_input_matrix_t	state_input_matrix_t;

	LinearSystemDynamics(
			const state_matrix_t& A,
			const state_input_matrix_t& B,
			const state_matrix_t& G,
			const state_input_matrix_t& H)
	: A_(A)
	, B_(B)
	, G_(G)
	, H_(H)
	{}

	virtual ~LinearSystemDynamics() = default;

	/**
	 * Returns pointer to the base class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual LinearSystemDynamics<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const override {

		return new LinearSystemDynamics<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
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
	 * Get the flow map value.
	 *
	 * @param [out] f: The flow map value.
	 */
	virtual void getFlowMap(state_vector_t& f) {

		f = A_ * BASE::x_ + B_ * BASE::u_;
	}

	/**
	 * Get the jump map value.
	 *
	 * @param [out] g: The jump map value.
	 */
	virtual void getJumpMap(state_vector_t& g) {

		g = G_ * BASE::x_ + H_ * BASE::u_;
	}

	/**
	 * The A matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeState(state_matrix_t& A) override {

		A = A_;
	}

	/**
	 * The B matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeInput(state_input_matrix_t& B) override {

		B = B_;
	}

	/**
	 * The G matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] G: \f$ G \f$ matrix.
	 */
	virtual void getJumpMapDerivativeState(state_matrix_t& G) override {

		G = G_;
	}

	/**
	 * The H matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] H: \f$ H \f$ matrix.
	 */
	virtual void getJumpMapDerivativeInput(state_input_matrix_t& H) override {

		H = H_;
	}

private:
	state_matrix_t 			A_;
	state_input_matrix_t 	B_;
	state_matrix_t 			G_;
	state_input_matrix_t 	H_;
};

} // namespace ocs2

#endif /* LINEARSYSTEMDYNAMICS_OCS2_H_ */
