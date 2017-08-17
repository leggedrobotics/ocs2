/*
 * KillIntegrationEventHandler.h
 *
 *  Created on: 09.08.2015
 *      Author: mgiftthaler
 */

#ifndef OCS2_KILL_INTEGRATION_EVENTHANDLER_H_
#define OCS2_KILL_INTEGRATION_EVENTHANDLER_H_

#include "ocs2_core/integration/EventHandler.h"

namespace ocs2{

/**
 * Kill Integration Event Handler Class
 * @tparam STATE_DIM
 */
template <size_t STATE_DIM>
class KillIntegrationEventHandler : public EventHandler<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double,STATE_DIM,1> State_T;

	KillIntegrationEventHandler():
		killIntegration_(false)
	{}

	~KillIntegrationEventHandler() {}

    /**
     * Checks event
     * @param [in] state
     * @param [in] t
     * @return
     */
	bool checkEvent(const State_T& state, const double& t) override {
		return killIntegration_;
	}

    /**
     * Handles event
     * @param [in] state
     * @param [in] t
     */
	void handleEvent(const State_T& state, const double& t) override {

		/* throw an exception which stops the integration */
		throw std::runtime_error("Integration terminated due to external event specified by user.");
	}

    /**
     * Sets event
     */
	void setEvent() {
		killIntegration_ = true;
	}

    /**
     * Resets event
     */
	void resetEvent() {
		killIntegration_ = false;
	}

private:
	bool killIntegration_;
};

} // namespace ocs2

#endif
