/*
 * Observer.h
 *
 *  Created on: 18.06.2015
 *      Author: neunertm
 */

#ifndef OCS2_OBSERVER_H_
#define OCS2_OBSERVER_H_

#include <limits>
#include "ocs2_core/integration/EventHandler.h"
#include "ocs2_core/dynamics/SystemBase.h"


namespace ocs2{

template <int STATE_DIM>
class IntegratorBase;

/**
 * Observer Class
 * @tparam STATE_DIM
 */
template <int STATE_DIM>
class Observer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::vector<double> TimeTrajectory_T;
	typedef Eigen::Matrix<double,STATE_DIM,1> State_T;
	typedef std::vector<State_T, Eigen::aligned_allocator<State_T> > StateTrajectory_T;

	friend class IntegratorBase<STATE_DIM>;

    /**
     * Constructor
     * @param [in] eventHandler
     */
	Observer(const std::shared_ptr<EventHandler<STATE_DIM> >& eventHandler = nullptr) :
		observeWrap([this](const State_T& x, const double& t){ this->observe(x,t); }),
		eventHandler_(eventHandler),
		system_(nullptr),
		maxNumSteps_(std::numeric_limits<size_t>::max())
	{}

    /**
     * Reset function
     */
	void reset() {
		stateTrajectory_.clear();
		timeTrajectory_.clear();
	}

    /**
     *
     * @param [in] maxNumSteps
     * @param [in] system
     */
	void setMaxNumSteps(size_t maxNumSteps, const std::shared_ptr<SystemBase<STATE_DIM> >& system) {
		maxNumSteps_ = maxNumSteps;
		system_ = system;
	}

    /**
     * Observe function
     * @param [in] x
     * @param [in] t
     */
	void observe(const State_T& x, const double& t)
	{
		stateTrajectory_.push_back(x);
		timeTrajectory_.push_back(t);

		if (eventHandler_ && eventHandler_->checkEvent(x, t))
		{
			eventHandler_->handleEvent(x, t);
		}

		if (system_)
			if (system_->getNumFunctionCalls() > maxNumSteps_)  throw std::runtime_error("integration terminated: max number of steps reached.\n");
	}

	// Lambda to pass to odeint (odeint takes copies of the observer so we can't pass the class
	std::function<void (const State_T& x, const double& t)> observeWrap;

private:
	std::shared_ptr<EventHandler<STATE_DIM> > eventHandler_;
	std::shared_ptr<SystemBase<STATE_DIM> > system_;
	size_t maxNumSteps_;

	StateTrajectory_T stateTrajectory_;
	TimeTrajectory_T timeTrajectory_;

};


} // namespace ocs2

#endif /* OCS2OBSERVER_H_ */
