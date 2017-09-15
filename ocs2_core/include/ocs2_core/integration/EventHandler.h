/*
 * EventHandler.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef OCS2_EVENTHANDLER_H_
#define OCS2_EVENTHANDLER_H_

#include <Eigen/Dense>

namespace ocs2{

/**
 * Event handler class for ode solvers.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class EventHandler
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double,STATE_DIM,1> State_T;

	/**
	 * Default constructor
	 */
	EventHandler() {}

	/**
	 * Default destructor
	 */
	virtual ~EventHandler() {}

	/**
	 * Checks if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 * @return
	 */
	virtual bool checkEvent(const State_T& state, const double& time) = 0;

	/**
	 * The operation should be performed if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 */
	virtual void handleEvent(const State_T& state, const double& time) = 0;

private:
};

} // namespace ocs2

#endif /* OCS2EVENTHANDLER_H_ */
