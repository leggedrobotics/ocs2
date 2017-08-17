/*
 * EventHandler.h
 *
 *  Created on: 18.06.2015
 *      Author: neunertm
 */

#ifndef OCS2_EVENTHANDLER_H_
#define OCS2_EVENTHANDLER_H_

namespace ocs2{

#include <Eigen/Dense>

/**
 * Event Handler Class
 * @tparam STATE_DIM
 */
template <int STATE_DIM>
class EventHandler
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double,STATE_DIM,1> State_T;

	EventHandler() {}
	virtual ~EventHandler() {}

	/**
	 * Checks event
	 * @param [in] state
	 * @param [in] t
	 * @return
	 */
	virtual bool checkEvent(const State_T& state, const double& t) = 0;

	/**
	 * Handle event
	 * @param [in] state
	 * @param [in] t
	 */
	virtual void handleEvent(const State_T& state, const double& t) = 0;

private:
};

} // namespace ocs2

#endif /* OCS2EVENTHANDLER_H_ */
