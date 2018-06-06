/*
 * ControlledSystemBase.h
 *
 *  Created on: Dec 29, 2015
 *      Author: farbod
 */

#ifndef CONTROLLEDSYSTEMBASE_OCS2_H_
#define CONTROLLEDSYSTEMBASE_OCS2_H_

#include <cstring>
#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Dense>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/integration/ODE_Base.h"
#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"

namespace ocs2{

/**
 * The base class for non-autonomous system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class ControlledSystemBase : public ODE_Base<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef ODE_Base<STATE_DIM> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t 				scalar_t;
	typedef typename DIMENSIONS::scalar_array_t 		scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 		state_vector_t;
	typedef typename DIMENSIONS::input_vector_t 		input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t 	input_vector_array_t;
	typedef typename DIMENSIONS::input_state_t 			input_state_t;
	typedef typename DIMENSIONS::input_state_array_t 	input_state_array_t;
	typedef typename DIMENSIONS::controller_t 			controller_t;
	typedef typename DIMENSIONS::constraint1_vector_t 	constraint1_vector_t;
	typedef typename DIMENSIONS::constraint2_vector_t 	constraint2_vector_t;
	typedef typename DIMENSIONS::dynamic_vector_t 		dynamic_vector_t;

	/**
	 * The default constructor.
	 */
	ControlledSystemBase()

	: BASE()
	, controllerIsSet_(false)
	{
		reset();
	}

	/**
	 * Copy constructor.
	 */
	ControlledSystemBase(const ControlledSystemBase& rhs)

	: ControlledSystemBase()
	{}

	/**
	 * Default destructor.
	 */
	virtual ~ControlledSystemBase() = default;

	/**
	 * Sets the linear control policy using the controller class.
	 * The controller class is defined as \f$ u(t,x) = u_{ff}(t) + K(t) * x \f$.
	 *
	 * @param [in] controller: The control policy.
	 */
	void setController(const controller_t& controller) {

		controller_ = controller;

		linInterpolateUff_.setTimeStamp(&controller_.time_);
		linInterpolateUff_.setData(&controller_.uff_);

		linInterpolateK_.setTimeStamp(&controller_.time_);
		linInterpolateK_.setData(&controller_.k_);

		controllerIsSet_ = true;
	}

	/**
	 * Resets the internal classes.
	 */
	virtual void reset() {

		linInterpolateK_.reset();
		linInterpolateUff_.reset();
	}

	/**
	 * Sets the linear control policy using the feedback and feedforward components.
	 * The controller class is defined as \f$ u(t,x) = u_{ff}(t) + K(t) * x \f$.
	 *
	 * @param [in] controllerTime: Time stamp.
	 * @param [in] uff: Feedforward term trajectory, \f$ u_{ff} \f$.
	 * @param [in] k: Feedback term trajectory, \f$ K \f$.
	 */
	void setController(
			const scalar_array_t& controllerTime,
			const input_vector_array_t& uff,
			const input_state_array_t& k) {

		controller_.time_ = controllerTime;
		controller_.uff_ = uff;
		controller_.k_ = k;

		setController(controller_);
	}

	/**
	 * Computes input vector.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @return Current input.
	 */
	input_vector_t computeInput(
			const scalar_t& t,
			const state_vector_t& x) {

		input_vector_t uff;
		linInterpolateUff_.interpolate(t, uff);
		int greatestLessTimeStampIndex = linInterpolateUff_.getGreatestLessTimeStampIndex();

		input_state_t k;
		linInterpolateK_.interpolate(t, k, greatestLessTimeStampIndex);

		return uff + k*x;
	}

	/**
	 * Computes derivative of the autonomous system dynamics with the given control policy.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [out] dxdt: Current state time derivative.
	 */
	void computeFlowMap(
			const scalar_t& t,
			const state_vector_t& x,
			state_vector_t& dxdt)  {

		BASE::numFunctionCalls_++;
		input_vector_t u = computeInput(t, x);
		computeFlowMap(t, x, u, dxdt);
	}

	/**
	 * Initializes the system dynamics.
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
	virtual ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const = 0;

	/**
	 * Computes derivative of the autonomous system dynamics.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [in] u: Current input.
	 * @param [out] dxdt: Current state time derivative.
	 */
	virtual void computeFlowMap(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u,
			state_vector_t& dxdt) = 0;

	/**
	 * State map at the transition time
	 *
	 * @param [in] time: transition time
	 * @param [in] state: transition state
	 * @param [out] mappedState: mapped state after transition
	 */
	virtual void computeJumpMap(
			const scalar_t& time,
			const state_vector_t& state,
			state_vector_t& mappedState) override {

		BASE::computeJumpMap(time, state, mappedState);
	}

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
			dynamic_vector_t& guardSurfacesValue) override {

		BASE::computeGuardSurfaces(time, state, guardSurfacesValue);
	}

protected:
	controller_t controller_;

	bool controllerIsSet_;

	LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > linInterpolateUff_;
	LinearInterpolation<input_state_t, Eigen::aligned_allocator<input_state_t> > linInterpolateK_;
};

} // namespace ocs2

#endif
