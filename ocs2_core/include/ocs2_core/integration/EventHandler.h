/*
 * EventHandler.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef EVENTHANDLER_OCS2_H_
#define EVENTHANDLER_OCS2_H_

#include <exception>
#include <memory>

namespace ocs2{

/**
 * Exception type which can be catch outside of the ODE solver where the programs returns the program handle to
 * EventHandler::mapState for reinitializing the ODE solver.
 */
class EventHandlerException: public std::exception
{
public:
	EventHandlerException(const double& time)
	: time_(time)
	{}

	void getActiveEventTime(double& time) const {
		time = time_;
	}

	virtual const char* what() const throw() {
		return "OCS2-integration EventHandler's exception is triggered.";
	}

private:
	double time_;
};

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

	typedef std::shared_ptr<EventHandler<STATE_DIM>> Ptr;

	typedef std::exception Even_Exception_T;

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
	 * @return boolean: 
	 */
	virtual bool checkEvent(const State_T& state, const double& time) = 0;

	/**
	 * The operation should be performed if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 */
	virtual void handleEvent(const State_T& state, const double& time) = 0;
};


} // namespace ocs2

#endif /* EVENTHANDLER_OCS2_H_ */
