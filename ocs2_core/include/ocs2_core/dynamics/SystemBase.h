/*
 * SystemBase.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef SYSTEMBASE_OCS2_H_
#define SYSTEMBASE_OCS2_H_

#include <Eigen/Dense>

#include "ocs2_core/Dimensions.h"

namespace ocs2{

/**
 * The base class for autonomous system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class SystemBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Dimensions<STATE_DIM, 0> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t         scalar_t;
	typedef typename DIMENSIONS::state_vector_t	  state_vector_t;
	typedef typename DIMENSIONS::dynamic_vector_t dynamic_vector_t;

    /**
     * Default constructor
     */
	SystemBase()
	: numFunctionCalls_(0)
	{}

	/**
	 * Default destructor
	 */
	virtual ~SystemBase() = default;

    /**
     * Default copy constructor
     */
	SystemBase(const SystemBase& rhs)
	: SystemBase()
	{}

	/**
	 * Gets the number of function calls.
	 *
	 * @return size_t: number of function calls
	 */
	size_t getNumFunctionCalls() {
		return numFunctionCalls_;
	}

	/**
	 * Resets the number of function calls to zero.
	 *
	 */
	void resetNumFunctionCalls() {
		numFunctionCalls_ = 0;
	}

	/**
	 * Computes the autonomous system dynamics.
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [out] dxdt: Current state time derivative
	 */
	virtual void computeFlowMap(
			const scalar_t& t,
			const state_vector_t& x,
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
			state_vector_t& mappedState) {

		mappedState = state;
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
			dynamic_vector_t& guardSurfacesValue) {

		guardSurfacesValue = -dynamic_vector_t::Ones(1);
	}

protected:
	size_t numFunctionCalls_;

};

} // namespace ocs2

#endif /* SYSTEMBASE_H_ */
