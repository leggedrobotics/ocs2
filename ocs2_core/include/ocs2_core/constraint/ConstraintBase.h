/*
 * ConstraintBase.h
 *
 *  Created on: Dec 19, 2017
 *      Author: farbod
 */

#ifndef CONSTRAINTBASE_OCS2_H_
#define CONSTRAINTBASE_OCS2_H_

#include <memory>
#include <cstring>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"

namespace ocs2{

/**
 * Base class for the constraints and its Derivatives. The linearized constraints are defined as: \n
 *
 * - Linearized state-input constraints:       \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$ \n
 * - Linearized only-state constraints:        \f$ F(t) \delta x + h(t) = 0 \f$ \n
 * - Linearized only-state final constraints:  \f$ F_f(t) \delta x + h_f(t) = 0 \f$
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class ConstraintBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<LOGIC_RULES_T::state_dim, LOGIC_RULES_T::input_dim>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t scalar_t;
	typedef typename DIMENSIONS::state_vector_t   state_vector_t;
	typedef typename DIMENSIONS::input_vector_t input_vector_t;
	typedef typename DIMENSIONS::state_matrix_t   state_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_t 	state_input_matrix_t;
	typedef typename DIMENSIONS::constraint1_vector_t 	constraint1_vector_t;
	typedef typename DIMENSIONS::constraint2_vector_t 	constraint2_vector_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_t constraint1_input_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t   constraint2_state_matrix_t;

	/**
	 * Default constructor
	 */
	ConstraintBase() = default;

	/**
	 * Default copy constructor
	 */
	ConstraintBase(const ConstraintBase& rhs) = default;

	/**
	 * Default destructor
	 */
	virtual ~ConstraintBase() = default;

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
	 * Initializes the system Constraints.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(
			LogicRulesMachine<LOGIC_RULES_T::state_dim, LOGIC_RULES_T::input_dim, LOGIC_RULES_T>& logicRulesMachine,
			const size_t& partitionIndex,
			const char* algorithmName = nullptr)
	{}

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const {

		return new ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
	}

	/**
	 * Computes the state-input equality constraints.
	 *
	 * @param [out] e: The state-input equality constraints value.
	 */
	virtual void getConstraint1(constraint1_vector_t& e)  {}

	/**
	 * Get the number of state-input active equality constriants.
	 *
	 * @param [in] time: time.
	 * @return number of state-input active equality constriants.
	 */
	virtual size_t numStateInputConstraint(const scalar_t& time) {

		return 0;
	}

	/**
	 * get the state-only equality constraints.
	 *
	 * @param [out] h: The state-only equality constraints value.
	 */
	virtual void getConstraint2(constraint2_vector_t& h) {}

	/**
	 * Get the number of state-only active equality constriants.
	 *
	 * @param [in] time: time.
	 * @return number of state-only active equality constriants.
	 */
	virtual size_t numStateOnlyConstraint(const scalar_t& time) {

		return 0;
	}

	/**
	 * Compute the final state-only equality constraints.
	 *
	 * @param [out] h_f: The final state-only equality constraints value.
	 */
	virtual void getFinalConstraint2(constraint2_vector_t& h_f) {}

	/**
	 * Get the number of final state-only active equality constriants.
	 *
	 * @param [in] time: time.
	 * @return number of final state-only active equality constriants.
	 */
	virtual size_t numStateOnlyFinalConstraint(const scalar_t& time) {

		return 0;
	}

	/**
	 * The C matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] C: \f$ C(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {}

	/**
	 * The D matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] D: \f$ D(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) {}

	/**
	 * The F matrix at a given operating point for the linearized state-only constraints,
	 * \f$ F(t) \delta x + h(t) = 0 \f$.
	 *
	 * @param [out] F: \f$ F(t) \f$ matrix.
	 */
	virtual void getConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

	/**
	 * The F matrix at a given operating point for the linearized terminal state-only constraints,
	 * \f$ F_f(t) \delta x + h_f(t) = 0 \f$.
	 *
	 * @param [out] F_f: \f$ F_f(t) \f$ matrix.
	 */
	virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_f) {}

protected:
	scalar_t t_;
	state_vector_t x_;
	input_vector_t u_;
};

} // namespace ocs2

#endif /* CONSTRAINTBASE_OCS2_H_ */
