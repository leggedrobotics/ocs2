/*
 * EventHandlerBase.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef EVENTHANDLERBASE_OCS2_H_
#define EVENTHANDLERBASE_OCS2_H_

#include <exception>
#include <memory>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/Dimensions.h"

namespace ocs2{

/**
 * Event handler class for ode solvers.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class EventHandlerBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<EventHandlerBase<STATE_DIM>> Ptr;

	typedef Dimensions<STATE_DIM, 0> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t 				scalar_t;
	typedef typename DIMENSIONS::scalar_array_t 		scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t			state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t	state_vector_array_t;
	typedef typename DIMENSIONS::dynamic_vector_t		dynamic_vector_t;

	/**
	 * Default constructor
	 */
	EventHandlerBase()
	: systemPtr_(nullptr)
	{}

	/**
	 * Default destructor
	 */
	virtual ~EventHandlerBase() = default;

	/**
	 * Sets a pointer to the system dynamics. This method is invoked by the integrator class in
	 * order to share integrator's system dynamics with eventHandler.
	 *
	 * @param systemPtr: shared pointer to the integrator's system dynamics.
	 */
	void setSystem(const std::shared_ptr<SystemBase<STATE_DIM>>& systemPtr) {

		systemPtr_ = systemPtr;
	}

	/**
	 * Resets the class.
	 */
	virtual void reset() = 0;

	/**
	 * Checks if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 * @return boolean: Whether an event is active.
	 */
	virtual bool checkEvent(
			const state_vector_t& state,
			const scalar_t& time) = 0;

	/**
	 * The operation should be performed if an event is activated. The method gets references to the time and state
	 * trajectories. The current time and state are the last elements of their respective container.
	 * The method should also return a "Non-Negative" ID which indicates the a unique ID for the active events.
	 * Note that will the negative return values are reserved to handle internal events for the program.
	 *
	 * @param [out] stateTrajectory: The state trajectory which contains the current state vector as its last element.
	 * @param [out] timeTrajectory: The time trajectory which contains the current time as its last element.
	 * @retune boolean: A non-negative unique ID for the active events.
	 */
	virtual int handleEvent(
			state_vector_array_t& stateTrajectory,
			scalar_array_t& timeTrajectory) = 0;

protected:
	/**
	 * System dynamics used by integrator.
	 */
	std::shared_ptr<SystemBase<STATE_DIM>> systemPtr_;

};


} // namespace ocs2

#endif /* EVENTHANDLERBASE_OCS2_H_ */
