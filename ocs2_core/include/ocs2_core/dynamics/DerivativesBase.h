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
#include "ocs2_core/logic/LogicRulesBase.h"

namespace ocs2{

/**
 * Base class for the linearized system dynamics. The linearized system dynamics are defined as: \n
 * - Linearized system:                        \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
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
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value, "LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;

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
	DerivativesBase() {}

	/**
	 * Default destructor
	 */
	virtual ~DerivativesBase() {}

	/**
	 * Sets the current time, state, and control inout.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [in] u: Current input.
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
			const input_vector_t& u) {
		t_ = t;
		x_ = x;
		u_ = u;
	}

	/**
	 * Initializes the system derivative.
	 *
	 * @param [in] logicRules: A class containing the logic rules.
	 * @param [in] activeSubSystemID: Current active subsystem index.
	 * @param [in] algorithmName: The algorithm that uses this class.
	 */
	virtual void initializeModel(const LOGIC_RULES_T& logicRules, const int& activeSubSystemID, const char* algorithmName=NULL)
	{}

	/**
	 * The A matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getDerivativeState(state_matrix_t& A) = 0;

	/**
	 * The B matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getDerivativesControl(control_gain_matrix_t& B) = 0;

	/**
	 * Returns pointer to DerivativesBase class.
	 *
	 * @return DerivativesBase*: a shared_ptr pointer.
	 */
	virtual std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > clone() const = 0;

protected:
	scalar_t t_;
	state_vector_t x_;
	input_vector_t u_;
};

} // namespace ocs2

#endif /* DERIVATIVESBASE_OCS2_H_ */
