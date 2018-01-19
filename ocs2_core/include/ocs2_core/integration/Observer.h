/*
 * Observer.h
 *
 *  Created on: 17.12.2015
 *      Author: farbodf
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
     * Sets the maximum number of integration points per a second for ode solvers.
     *
     * @param [in] maxNumSteps: maximum number of integration points
     * @param [in] system: A pointer to system.
     */
	void setMaxNumSteps(size_t maxNumSteps, const std::shared_ptr<SystemBase<STATE_DIM> >& system) {
		maxNumSteps_ = maxNumSteps;
		system_ = system;
	}

    /**
     * Observe function to retrieve the variable of interest.
     * @param [in] x: Current state.
     * @param [in] t: Current time.
     */
	void observe(const State_T& x, const double& t)
	{
		stateTrajectoryPtr_->push_back(x);
		timeTrajectoryPtr_->push_back(t);

		if (eventHandler_ && eventHandler_->checkEvent(x, t)) {
			eventHandler_->handleEvent(x, t);
		}

		if (system_ && system_->getNumFunctionCalls() > maxNumSteps_)
			throw std::runtime_error("integration terminated: max number of steps reached.\n");
	}

	/**
	 * Adjust observer after LogicRulesEvent
	 *
	 * @param switchStartTime: event time
	 */
	void adjustObserverAfterLogicRulesEvent(const double& switchStartTime, size_t& outputSize) {

		outputSize = stateTrajectoryPtr_->size();
		double alpha = (switchStartTime-timeTrajectoryPtr_->at(outputSize-2)) / (timeTrajectoryPtr_->at(outputSize-1)-timeTrajectoryPtr_->at(outputSize-2));

		std::cout << "alpha: " << alpha << std::endl;
		std::cout << "timeTrajectory_[-2]: " << timeTrajectoryPtr_->at(outputSize-2) << "\t state: " << stateTrajectoryPtr_->at(outputSize-2).transpose() << std::endl;
		std::cout << "timeTrajectory_[-1]: " << timeTrajectoryPtr_->at(outputSize-1) << "\t state: " << stateTrajectoryPtr_->at(outputSize-1).transpose() << std::endl;

		timeTrajectoryPtr_->at(outputSize-1)  = switchStartTime;
		stateTrajectoryPtr_->at(outputSize-1) = (1.0-alpha)*stateTrajectoryPtr_->at(outputSize-2) + alpha*stateTrajectoryPtr_->at(outputSize-1);
	}

	/**
	 * Lambda to pass to odeint (odeint takes copies of the observer so we can't pass the class
	 */
	std::function<void (const State_T& x, const double& t)> observeWrap;

	/**
	 * Set state trajectory pointer to observer.
	 *
	 * @param stateTrajectoryPtr
	 */
	void setStateTrajectory(StateTrajectory_T* stateTrajectoryPtr) {
		stateTrajectoryPtr_ = stateTrajectoryPtr;
	}

	/**
	 * Set time trajectory pointer to observer.
	 *
	 * @param timeTrajectoryPtr
	 */
	void setTimeTrajectory(TimeTrajectory_T* timeTrajectoryPtr) {
		timeTrajectoryPtr_ = timeTrajectoryPtr;
	}


private:
	std::shared_ptr<EventHandler<STATE_DIM> > eventHandler_;
	std::shared_ptr<SystemBase<STATE_DIM> > system_;
	size_t maxNumSteps_;

	StateTrajectory_T* stateTrajectoryPtr_;
	TimeTrajectory_T* timeTrajectoryPtr_;

};


} // namespace ocs2

#endif /* OCS2OBSERVER_H_ */
