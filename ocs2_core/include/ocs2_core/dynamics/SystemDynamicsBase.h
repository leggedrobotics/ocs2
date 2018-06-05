/*
 * SystemDynamicsBase.h
 *
 *  Created on: Jun 5, 2018
 *      Author: farbod
 */

#ifndef SYSTEMDYNAMICSBASE_OCS2_H_
#define SYSTEMDYNAMICSBASE_OCS2_H_

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/dynamics/DerivativesBase.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"

namespace ocs2{

/**
 * The system dynamics interface.
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
class SystemDynamicsBase
		: public DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
		, public ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		state_dim_ 	= STATE_DIM,
		input_dim_ 	= INPUT_DIM,
		domain_dim_	= 1 + state_dim_ + input_dim_,
	};

	typedef std::shared_ptr<SystemDynamicsBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const SystemDynamicsBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> 		DEV_BASE;
	typedef ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> 	DYN_BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t 						scalar_t;
	typedef typename DIMENSIONS::state_vector_t   				state_vector_t;
	typedef typename DIMENSIONS::input_vector_t 				input_vector_t;
	typedef typename DIMENSIONS::state_matrix_t 				state_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_t 			state_input_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t   	constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_t 	constraint1_input_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t   	constraint2_state_matrix_t;

	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> 			dynamic_vector_t;
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, STATE_DIM> 	dynamic_state_vector_t;
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, INPUT_DIM> 	dynamic_input_vector_t;

	/**
	 * Default constructor
	 */
	SystemDynamicsBase() = default;

	/**
	 * Copy constructor
	 */
	SystemDynamicsBase(const SystemDynamicsBase& rhs) = default;

	/**
	 * Default destructor
	 */
	virtual ~SystemDynamicsBase() = default;

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
     * Returns pointer to the class.
     *
     * @return A raw pointer to the class.
     */
	virtual SystemDynamicsBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const = 0;


	/**
	 * Interface method to the state flow map of the hybrid system.
	 *
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	virtual void computeFlowMap(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input,
			state_vector_t& stateDerivative) = 0;

	/**
	 * Interface method to the state jump map of the hybrid system.
	 *
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] jumpedState: jumped state.
	 */
	virtual void computeJumpMap(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input,
			state_vector_t& jumpedState) = 0;

	/**
	 * Interface method to the guard surfaces.
	 *
	 * @param [in] time: transition time
	 * @param [in] state: transition state
	 * @param [out] guardSurfacesValue: An array of guard surfaces values
	 */
	virtual void computeGuardSurfaces(
			const scalar_t& time,
			const state_vector_t& state,
			dynamic_vector_t& guardSurfacesValue) = 0;


    /**
     * Sets the current time, state, and control input.
     *
     * @param [in] time: time
     * @param [in] state: state vector
     * @param [in] input: input vector
     */
	virtual void setCurrentStateAndControl(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) = 0;


	/**
	 * Partial time derivative of the system flow map.
	 * \f$ \frac{\partial f}{\partial t}  \f$.
	 *
	 * @param [out] df: \f$ \frac{\partial f}{\partial t} \f$ matrix.
	 */
	virtual void getFlowMapDerivativeTime(state_vector_t& df) = 0;

	/**
	 * The A matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeState(state_matrix_t& A) = 0;

	/**
	 * The B matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeInput(state_input_matrix_t& B) = 0;


	/**
	 * Partial time derivative of the system jump map.
	 * \f$ \frac{\partial g}{\partial t}  \f$.
	 *
	 * @param [out] dg: \f$ \frac{\partial g}{\partial t} \f$ matrix.
	 */
	virtual void getJumpMapDerivativeTime(state_vector_t& dg) = 0;

	/**
	 * The G matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] G: \f$ G \f$ matrix.
	 */
	virtual void getJumpMapDerivativeState(state_matrix_t& G) = 0;

	/**
	 * The H matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] H: \f$ H \f$ matrix.
	 */
	virtual void getJumpMapDerivativeInput(state_input_matrix_t& H) = 0;


	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_t_gamma: Derivative of the guard surfaces w.r.t. time.
	 */
	virtual void getGuardSurfacesDerivativeTime(dynamic_vector_t& D_t_gamma) = 0;

	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_x_gamma: Derivative of the guard surfaces w.r.t. state vector.
	 */
	virtual void getGuardSurfacesDerivativeState(dynamic_state_matrix_t& D_x_gamma) = 0;

	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_u_gamma: Derivative of the guard surfaces w.r.t. input vector.
	 */
	virtual void getGuardSurfacesDerivativeInput(dynamic_input_matrix_t& D_u_gamma) = 0;

protected:

};

} // namespace ocs2


#endif /* SYSTEMDYNAMICSBASE_OCS2_H_ */
