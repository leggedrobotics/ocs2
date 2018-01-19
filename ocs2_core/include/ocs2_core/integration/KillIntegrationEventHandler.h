/*
 * KillIntegrationEventHandler.h
 *
 *  Created on: 09.08.2015
 *      Author: mgiftthaler
 */

#ifndef OCS2_KILL_INTEGRATION_EVENTHANDLER_H_
#define OCS2_KILL_INTEGRATION_EVENTHANDLER_H_

#include <memory>

#include "ocs2_core/integration/EventHandler.h"

namespace ocs2{

/**
 * Specialized event handler for terminating Integration.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class KillIntegrationEventHandler : public EventHandler<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<KillIntegrationEventHandler<STATE_DIM>> Ptr;

	typedef Eigen::Matrix<double,STATE_DIM,1> State_T;

	/**
	 * Default constructor
	 */
	KillIntegrationEventHandler() {}

	/**
	 * Default destructor
	 */
	~KillIntegrationEventHandler() {}

	/**
	 * Checks if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 * @return boolean: 
	 */
	bool checkEvent(const State_T& state, const double& time) override {
		return EventHandler<STATE_DIM>::killIntegration_;
	}

	/**
	 * Stop integration.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 */
	void handleEvent(const State_T& state, const double& time) override {
		/* throw an exception which stops the integration */
		throw std::runtime_error("Integration terminated due to external event specified by user.");
	}

private:

};

} // namespace ocs2

#endif
