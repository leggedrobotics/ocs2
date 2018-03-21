/*
 * SystemEventHandler.h
 *
 *  Created on: 09.08.2015
 *      Author: mgiftthaler
 */

#ifndef SYSTEMEVENTHANDLER_OCS2_H_
#define SYSTEMEVENTHANDLER_OCS2_H_

#include <limits>
#include <memory>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/integration/EventHandlerBase.h"

namespace ocs2{

/**
 * System event ID. all values are negative.
 */
enum sys_event_id
{
	killIntegration = -1,      //!< killIntegration: kill integration due to an external signal.
	maxCall			= -2       //!< maximum number of function calls.
};

/**
 * Specialized event handler for handling toolbox invoked events.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class SystemEventHandler : public EventHandlerBase<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<SystemEventHandler<STATE_DIM>> Ptr;

	typedef EventHandlerBase<STATE_DIM> BASE;
	typedef typename BASE::scalar_t				scalar_t;
	typedef typename BASE::scalar_array_t 		scalar_array_t;
	typedef typename BASE::state_vector_t		state_vector_t;
	typedef typename BASE::state_vector_array_t state_vector_array_t;

	/**
	 * Default constructor
	 */
	SystemEventHandler()
	: maxNumSteps_(std::numeric_limits<size_t>::max()),
	  eventID_(std::numeric_limits<int>::min())
	{}

	/**
	 * Default destructor
	 */
	virtual ~SystemEventHandler() = default;

	/**
	 * Resets the class.
	 */
	virtual void reset() override {}

	/**
	 * Sets the maximum number of integration points per a second for ode solvers.
	 *
	 * @param [in] maxNumSteps: maximum number of integration points
	 */
	void setMaxNumSteps(size_t maxNumSteps) {
		maxNumSteps_ = maxNumSteps;
	}

	/**
	 * Checks if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 * @return boolean: 
	 */
	virtual bool checkEvent(
			const state_vector_t& state,
			const scalar_t& time) override {

		bool terminateFlag = false;

		if (killIntegration_==true) {
			terminateFlag = true;
			eventID_ = sys_event_id::killIntegration;
		}

		if (BASE::systemPtr_->getNumFunctionCalls()>maxNumSteps_) {
			terminateFlag = true;
			eventID_ = sys_event_id::maxCall;
		}

		return terminateFlag;
	}

	/**
	 * If an event is activated, it will terminates the integration. The method gets references to the time and state
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
			scalar_array_t& timeTrajectory) override {

		return eventID_;
	}

	/**
	 * Activate KillIntegrationEvent.
	 */
	static void ActivateKillIntegration() {

		killIntegration_ = true;
	}

	/**
	 * Deactivate KillIntegrationEvent.
	 */
	static void DeactivateKillIntegration() {

		killIntegration_ = false;
	}

protected:
	static bool killIntegration_; /*=false*/

	size_t maxNumSteps_;

	int eventID_;
};


template <int STATE_DIM>
bool SystemEventHandler<STATE_DIM>::killIntegration_ = false;

} // namespace ocs2


#endif /* SYSTEMEVENTHANDLER_OCS2_H_ */


