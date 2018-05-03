/*
 * DerivativesBase.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef DERIVATIVESBASE_OCS2_H_
#define DERIVATIVESBASE_OCS2_H_

#include <memory>
#include <cstring>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"

namespace ocs2{

/**
 * Base class for the linearized system dynamics. \n
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class DerivativesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t scalar_t;
	typedef typename DIMENSIONS::state_vector_t   state_vector_t;
	typedef typename DIMENSIONS::control_vector_t input_vector_t;
	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_t control_gain_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_control_matrix_t constraint1_control_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t   constraint2_state_matrix_t;

	/**
	 * Default constructor
	 */
	DerivativesBase() = default;

	/**
	 * Default destructor
	 */
	virtual ~DerivativesBase() = default;

	/**
	 * Sets the current time, state, and control inout.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [in] u: Current input.
	 */
	virtual void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) {

		t_ = t;
		x_ = x;
		u_ = u;
	}

	/**
	 * Initializes the system dynamics derivatives.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(
			LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>& logicRulesMachine,
			const size_t& partitionIndex,
			const char* algorithmName=NULL)
	{}

	/**
	 * Get partial time derivative of the system flow map.
	 * \f$ \frac{\partial f}{\partial t}  \f$.
	 *
	 * @param [out] df: \f$ \frac{\partial f}{\partial t} \f$ matrix.
	 */
	virtual void getFlowMapDerivativeTime(state_vector_t& df) {

		df.setZero();
	}

	/**
	 * Get the A matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeState(state_matrix_t& A) = 0;

	/**
	 * Get the B matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeInput(control_gain_matrix_t& B) = 0;

	/**
	 * Get partial time derivative of the system jump map.
	 * \f$ \frac{\partial g}{\partial t}  \f$.
	 *
	 * @param [out] dg: \f$ \frac{\partial g}{\partial t} \f$ matrix.
	 */
	virtual void getJumpMapDerivativeTime(state_vector_t& dg) {

		dg.setZero();
	}

	/**
	 * Get the G matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] G: \f$ G \f$ matrix.
	 */
	virtual void getJumpMapDerivativeState(state_matrix_t& G) {

		G.setIdentity();
	}

	/**
	 * Get the H matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] H: \f$ H \f$ matrix.
	 */
	virtual void getJumpMapDerivativeInput(control_gain_matrix_t& H) {

		H.setZero();
	}

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const = 0;

protected:
	scalar_t t_;
	state_vector_t x_;
	input_vector_t u_;
};

} // namespace ocs2

#endif /* DERIVATIVESBASE_OCS2_H_ */
