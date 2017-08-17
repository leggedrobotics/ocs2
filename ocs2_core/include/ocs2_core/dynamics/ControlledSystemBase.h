/*
 * ControlledSystemBase.h
 *
 *  Created on: Dec 29, 2015
 *      Author: farbod
 */

#ifndef CONTROLLEDSYSTEMBASE_OCS2_H_
#define CONTROLLEDSYSTEMBASE_OCS2_H_

#include <cstring>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/SystemBase.h"
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2{

/**
 * Base class for controlled system
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ControlledSystemBase : public SystemBase<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > Ptr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_feedback_t control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::constraint1_vector_t constraint1_vector_t;
	typedef typename DIMENSIONS::constraint2_vector_t constraint2_vector_t;

	ControlledSystemBase()
		: modelUpdated_(true)
	{}
	virtual ~ControlledSystemBase() {}

	/**
	 * Sets controller
	 * @param [in] controller
	 */
	void setController(const controller_t& controller) {

		controller_ = controller;

		linInterpolateUff_.setTimeStamp(&controller_.time_);
		linInterpolateUff_.setData(&controller_.uff_);

		linInterpolateK_.setTimeStamp(&controller_.time_);
		linInterpolateK_.setData(&controller_.k_);

		modelUpdated_ = true;
	}

	/**
	 * Sets controller
	 * @param [in] controllerTime
	 * @param [in] uff
	 * @param [in] k
	 */
	void setController(const scalar_array_t& controllerTime,
			const control_vector_array_t& uff,
			const control_feedback_array_t& k) {

		controller_.time_ = controllerTime;
		controller_.uff_ = uff;
		controller_.k_ = k;

		setController(controller_);
	}

	/**
	 * Computes input
	 * @param [in] t
	 * @param [in] x
	 * @param [out] u
	 */
	void computeInput(const scalar_t& t, const state_vector_t& x, control_vector_t& u)
	{
		control_vector_t uff;
		control_feedback_t k;

		linInterpolateUff_.interpolate(t, uff);
		linInterpolateK_.interpolate(t, k);

		u = uff + k*x;
	}

	/**
	 * Computes derivative
	 * @param [in] t
	 * @param [in] x
	 * @param [out] dxdt
	 */
	void computeDerivative(const scalar_t& t, const state_vector_t& x, state_vector_t& dxdt)  {

		SystemBase<STATE_DIM>::numFunctionCalls_++;

		control_vector_t u;
		computeInput(t, x, u);
		computeDerivative(t, x, u, dxdt);
	}

	/**
	 * Computes first constraint
	 * @param [in] t
	 * @param [in] x
	 * @param [in] u
	 * @param [out] numConstraint1
	 * @param [out] g1
	 */
	virtual void computeConstriant1(const scalar_t& t, const state_vector_t& x, const control_vector_t& u, size_t& numConstraint1, constraint1_vector_t& g1)  {

		numConstraint1 = 0;
	}

	/**
	 * Computes second constraint
	 * @param [in] t
	 * @param [in] x
	 * @param [out] numConstraint2
	 * @param [out] g2
	 */
	virtual void computeConstriant2(const scalar_t& t, const state_vector_t& x, size_t& numConstraint2, constraint2_vector_t& g2)  {
		numConstraint2 = 0;
	}

	/**
	 * Computes final constraint
	 * @param [in] t
	 * @param [in] x
	 * @param [out] numFinalConstraint2
	 * @param [out] g2Final
	 */
	virtual void computeFinalConstriant2(const scalar_t& t, const state_vector_t& x, size_t& numFinalConstraint2, constraint2_vector_t& g2Final)  {
		numFinalConstraint2 = 0;
	}

	/**
	 * Initializes the model
	 * @param [in] systemStockIndexes
	 * @param [in] switchingTimes
	 * @param [in] initState
	 * @param [in] activeSubsystemIndex
	 * @param [in] algorithmName
	 */
	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL)
	{
		SystemBase<STATE_DIM>::numFunctionCalls_ = 0;
		if (activeSubsystemIndex>=switchingTimes.size()-1)
			throw std::runtime_error("activeSubsystemIndex refers to a non-existing subsystem based on the input switchingTimes sequence.");
	}

	/**
	 * Returns copy of the shared pointer to ControlledSystemBase
	 * @return
	 */
	virtual std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > clone() const = 0;

	/**
	 * Computes derivative
	 * @param [in] t
	 * @param [in] x
	 * @param [in] u
	 * @param [out] dxdt
	 */
	virtual void computeDerivative(
			const scalar_t& t,
			const state_vector_t& x,
			const control_vector_t& u,
			state_vector_t& dxdt) = 0;

protected:
	controller_t controller_;

	bool modelUpdated_;

	LinearInterpolation<control_vector_t, Eigen::aligned_allocator<control_vector_t> > linInterpolateUff_;
	LinearInterpolation<control_feedback_t, Eigen::aligned_allocator<control_feedback_t> > linInterpolateK_;
};

} // namespace ocs2

#endif
