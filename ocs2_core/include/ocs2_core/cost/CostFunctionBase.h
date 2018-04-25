/*
 * CostFunctionBase.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef COSTFUNCTIONBASE_OCS2_H_
#define COSTFUNCTIONBASE_OCS2_H_

#include <memory>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"

namespace ocs2{

/**
 * Cost Function Base.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class CostFunctionBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::scalar_t       		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t 		scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 		state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t 	state_vector_array_t;
	typedef typename DIMENSIONS::state_matrix_t 		state_matrix_t;
	typedef typename DIMENSIONS::control_vector_t  		input_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::control_matrix_t 		control_matrix_t;
	typedef typename DIMENSIONS::control_feedback_t 	control_feedback_t;

	/**
	 * Default constructor
	 */
	CostFunctionBase()
	: timeStart_(0.0),
	  timeFinal_(1.0),
	  timeSD_(0.17),
	  timeMean_(0.5),
	  tNominalTrajectoryPtr_(nullptr),
	  xNominalTrajectoryPtr_(nullptr),
	  uNominalTrajectoryPtr_(nullptr),
	  xNominalFunc_(),
	  uNominalFunc_()
	{
		xNominalFunc_.setZero();
		uNominalFunc_.setZero();
	};

	/**
	 * Copy constructor
	 */
	CostFunctionBase(const CostFunctionBase& rhs)
	: CostFunctionBase()
	{}

	/**
	 * Default destructor
	 */
	virtual ~CostFunctionBase() = default;

	/**
	 * Sets the nominal state and input trajectories used in the cost function.
	 *
	 * @param [in] timeTrajectory: A Reference to the time trajectory.
	 * @param [in] stateTrajectory: A Reference to the state trajectory.
	 * @param [in] inputTrajectory: A Reference to the inout trajectory.
	 */
	virtual void setCostNominalTrajectories(
			const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory,
			const input_vector_array_t& inputTrajectory = input_vector_array_t()) {

		tNominalTrajectoryPtr_ = &timeTrajectory;

		if (stateTrajectory.empty()==false) {
			xNominalTrajectoryPtr_ = &stateTrajectory;
			xNominalFunc_.reset();
			xNominalFunc_.setTimeStamp(tNominalTrajectoryPtr_);
			xNominalFunc_.setData(xNominalTrajectoryPtr_);
		} else {
			xNominalTrajectoryPtr_ = nullptr;
			xNominalFunc_.setZero();
		}

		if (inputTrajectory.empty()==false) {
			uNominalTrajectoryPtr_ = &inputTrajectory;
			uNominalFunc_.reset();
			uNominalFunc_.setTimeStamp(tNominalTrajectoryPtr_);
			uNominalFunc_.setData(uNominalTrajectoryPtr_);
		} else {
			uNominalTrajectoryPtr_ = nullptr;
			uNominalFunc_.setZero();
		}
	}

	/**
	 * Gets the nominal state used in the cost function.
	 *
	 * @param [out] timeTrajectory: The time trajectory.
	 * @param [out] stateTrajectory: The state trajectory.
	 * @param [out] inputTrajectory: The input trajectory.
	 */
	virtual void getCostNominalTrajectories(
			scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory,
			input_vector_array_t& inputTrajectory) const {

		getCostNominalState(timeTrajectory, stateTrajectory);

		if (uNominalTrajectoryPtr_) {
			inputTrajectory = *uNominalTrajectoryPtr_;
		} else {
			inputTrajectory.empty();
		}
	}

	/**
	 * Gets the nominal state used in the cost function.
	 *
	 * @param [out] timeTrajectory: The time trajectory.
	 * @param [out] stateTrajectory: The state trajectory.
	 */
	virtual void getCostNominalState(
			scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const {

		if (xNominalTrajectoryPtr_) {
			timeTrajectory  = *tNominalTrajectoryPtr_;
			stateTrajectory = *xNominalTrajectoryPtr_;
		} else {
			timeTrajectory.empty();
			stateTrajectory.empty();
		}
	}

	/**
	 * Gets the nominal input used in the cost function.
	 *
	 * @param [out] timeTrajectory: The time trajectory.
	 * @param [out] inputTrajectory: The input trajectory.
	 */
	virtual void getCostNominalInput(
			scalar_array_t& timeTrajectory,
			input_vector_array_t& inputTrajectory) const {

		if (uNominalTrajectoryPtr_) {
			timeTrajectory  = *tNominalTrajectoryPtr_;
			inputTrajectory = *uNominalTrajectoryPtr_;
		} else {
			timeTrajectory.empty();
			inputTrajectory.empty();
		}
	}

    /**
     * Returns pointer to the class.
     *
     * @return A raw pointer to the class.
     */
	virtual CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const = 0;

	/**
	 * Initializes the cost function.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(
			LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL)
	{}

    /**
     * Sets the current time, state, and control input
     *
     * @param [in] t: Current time
     * @param [in] x: Current state vector
     * @param [in] u: Current input vector
     */
	virtual void setCurrentStateAndControl(
			const scalar_t& t, const
			state_vector_t& x, const
			input_vector_t& u) {

		t_ = t;
		x_ = x;
		u_ = u;
	}

    /**
     * Evaluates the cost.
     *
     * @param [out] L: The cost value.
     */
	virtual void evaluate(scalar_t& L) = 0;

    /**
     * Gets the state derivative.
     *
     * @param [out] dLdx: First order cost derivative with respect to state vector.
     */
	virtual void stateDerivative(state_vector_t& dLdx) = 0;

    /**
     * Gets state second order derivative.
     *
     * @param [out] dLdxx: Second order cost derivative with respect to state vector.
     */
	virtual void stateSecondDerivative(state_matrix_t& dLdxx) = 0;

    /**
     * Gets control input derivative.
     *
     * @param [out] dLdu: First order cost derivative with respect to input vector.
     */
	virtual void controlDerivative(input_vector_t& dLdu) = 0;

    /**
     * Gets control input second derivative.
     *
     * @param [out] dLduu: Second order cost derivative with respect to input vector.
     */
	virtual void controlSecondDerivative(control_matrix_t& dLduu) = 0;

    /**
     * Gets the state, control input derivative.
     *
     * @param [out] dLdxu: Second order cost derivative with respect to state and input vector.
     */
	virtual void stateControlDerivative(control_feedback_t& dLdxu) = 0;

    /**
     * Gets the terminal cost.
     *
     * @param [out] Phi: The final cost value
     */
	virtual void terminalCost(scalar_t& Phi) = 0;

    /**
     * Gets the terminal cost state derivative.
     *
     * @param [out] dPhidx: First order final cost derivative with respect to state vector.
     */
	virtual void terminalCostStateDerivative(state_vector_t& dPhidx) = 0;

    /**
     * Gets the terminal cost state second derivative
     *
     * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
     */
	virtual void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx) = 0;

protected:
	/**
	 * Sets the time period.
	 *
	 * @param [in] timeStart: The start time of the period.
	 * @param [in] timeFinal: The final time of the period.
	 */
	virtual void setTimePeriod(const double& timeStart, const double& timeFinal) {
		timeStart_ = timeStart;
		timeFinal_ = timeFinal;

		timeSD_ = (timeFinal-timeStart) / 6.0;
		timeMean_ = (timeFinal+timeStart) / 2.0;
	}

/*
 * Variables
 */
	scalar_t timeStart_ = 0.0;
	scalar_t timeFinal_ = 1.0;

	scalar_t timeSD_ = 0.17;  //1.0 / 6.0;
	scalar_t timeMean_ = 0.5;

	const scalar_array_t*		tNominalTrajectoryPtr_;
	const state_vector_array_t* xNominalTrajectoryPtr_;
	const input_vector_array_t* uNominalTrajectoryPtr_;

	ocs2::LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > xNominalFunc_;
	ocs2::LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > uNominalFunc_;

	scalar_t t_;
	state_vector_t x_;
	input_vector_t u_;
};

} // namespace ocs2


#endif /* COSTFUNCTIONBASE_OCS2_H_ */
