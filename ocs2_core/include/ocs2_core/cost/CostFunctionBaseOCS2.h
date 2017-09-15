/*
 * CostFunctionBaseOCS2.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef COSTFUNCTIONBASE_OCS2_H_
#define COSTFUNCTIONBASE_OCS2_H_

#include "ocs2_core/Dimensions.h"

namespace ocs2{

/**
 * Cost Function Base.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class CostFunctionBaseOCS2
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::scalar_t       		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t 		scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 		state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t 	state_vector_array_t;
	typedef typename DIMENSIONS::state_matrix_t 		state_matrix_t;
	typedef typename DIMENSIONS::control_vector_t  		control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_matrix_t 		control_matrix_t;
	typedef typename DIMENSIONS::control_feedback_t 	control_feedback_t;

	/**
	 * Default constructor
	 */
	CostFunctionBaseOCS2() {};

	/**
	 * Default destructor
	 */
	virtual ~CostFunctionBaseOCS2() {};

    /**
     * Returns pointer to CostFunctionOCS2 class.
     * @return CostFunctionBaseOCS2*: a shared_ptr pointer.
     */
	virtual std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > clone() const = 0;

    /**
     * Sets the current time, state, and control input
     *
     * @param [in] t: Current time
     * @param [in] x: Current state vector
     * @param [in] u: Current input vector
     */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) {
		t_ = t;
		x_ = x;
		u_ = u;
	}

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

    /**
     * Sets the nominal state used in the cost function.
     *
     * @param [in] timeTrajectory: The time trajectory.
     * @param [in] stateTrajectory: The state trajectory.
     */
	virtual void setCostNominalState(const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory)
	{}

    /**
     * Gets the nominal state used in the cost function.
     *
     * @param [out] timeTrajectory: The time trajectory.
     * @param [out] stateTrajectory: The state trajectory.
     */
	virtual void getCostNominalState(scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const
	{}

    /**
     * Sets the nominal input used in the cost function.
     *
     * @param [in] timeTrajectory: The time trajectory.
     * @param [in] inputTrajectory: The input trajectory.
     */
	virtual void setCostNominalInput(const scalar_array_t& timeTrajectory,
			const control_vector_array_t& inputTrajectory)
	{}

    /**
     * Gets the nominal input used in the cost function.
     *
     * @param [out] timeTrajectory: The time trajectory.
     * @param [out] inputTrajectory: The input trajectory.
     */
	virtual void getCostNominalInput(scalar_array_t& timeTrajectory,
			control_vector_array_t& inputTrajectory) const
	{}

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
     * Gets control derivative.
     *
     * @param [out] dLdu: First order cost derivative with respect to input vector.
     */
	virtual void controlDerivative(control_vector_t& dLdu) = 0;

    /**
     * Gets control second derivative.
     *
     * @param [out] dLduu: Second order cost derivative with respect to input vector.
     */
	virtual void controlSecondDerivative(control_matrix_t& dLduu) = 0;

    /**
     * Gets the state, input derivative.
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
	scalar_t t_;
	state_vector_t x_;
	control_vector_t u_;

	double timeStart_ = 0.0;
	double timeFinal_ = 1.0;

	double timeSD_ = 0.17;  //1.0 / 6.0;
	double timeMean_ = 0.5;
};

} // namespace ocs2


#endif /* COSTFUNCTIONBASE_OCS2_H_ */
