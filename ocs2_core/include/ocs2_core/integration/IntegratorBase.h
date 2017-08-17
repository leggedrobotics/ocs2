/*
 * IntegratorBase.h
 *
 *  Created on: 17.12.2015
 *      Author: farbodf
 */

#ifndef OCS2_INTEGRATORBASE_H_
#define OCS2_INTEGRATORBASE_H_

#include <limits>

#include "ocs2_core/dynamics/SystemBase.h"
#include "ocs2_core/integration/Observer.h"
#include "ocs2_core/integration/EventHandler.h"


namespace ocs2{

/**
 * Integrator Base Class
 * @tparam STATE_DIM
 */
template <int STATE_DIM>
class IntegratorBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::vector<double> TimeTrajectory_T;
	typedef Eigen::Matrix<double, STATE_DIM, 1> State_T;
	typedef std::vector<State_T, Eigen::aligned_allocator<State_T> > StateTrajectory_T;

	IntegratorBase(
			const std::shared_ptr<SystemBase<STATE_DIM> >& system,
			const std::shared_ptr<EventHandler<STATE_DIM> >& eventHandler = nullptr) :
				observer_(eventHandler),
				system_(system),
				eventHandler_(eventHandler)
	{}

	virtual ~IntegratorBase() {}

	/**
	 * Reset function
	 */
	virtual void reset() {
		observer_.reset();
	}

	// Equidistant integration based on number of time steps and step length
	//	virtual bool integrate_n_steps(
	//		const State_T& initialState,
	//		const double& startTime,
	//		size_t numSteps,
	//		double dt,
	//		StateTrajectory_T& stateTrajectory,
	//		TimeTrajectory_T& timeTrajectory
	//	) = 0;

	// Equidistant integration based on initial and final time as well as step length
	// = integrate_const
	/**
	 * Equidistant integration based on initial and final time as well as step length
	 * @param [in] initialState
	 * @param [in] startTime
	 * @param [in] finalTime
	 * @param [in] dt
	 * @param [out] stateTrajectory
	 * @param [out] timeTrajectory
	 * @return boolean
	 */
	virtual bool integrate(
			const State_T& initialState,
			const double& startTime,
			const double& finalTime,
			double dt,
			StateTrajectory_T& stateTrajectory,
			TimeTrajectory_T& timeTrajectory
	) = 0;

	// Adaptive time integration based on start time and final time
	// = integrate adaptive
	/**
	 * Adaptive time integration based on start time and final time
	 * @param [in] initialState
	 * @param [in] startTime
	 * @param [in] finalTime
	 * @param [out] stateTrajectory
	 * @param [out] timeTrajectory
	 * @param [in] dtInitial
	 * @param [in] AbsTol
	 * @param [in] RelTol
	 * @param [in] maxNumSteps
	 * @return boolean
	 */
	virtual bool integrate(
			const State_T& initialState,
			const double& startTime,
			const double& finalTime,
			StateTrajectory_T& stateTrajectory,
			TimeTrajectory_T& timeTrajectory,
			double dtInitial = 0.01,
			double AbsTol = 1e-6,
			double RelTol = 1e-3,
			size_t maxNumSteps = std::numeric_limits<size_t>::max()
	) = 0;

	// Output integration based on a given time trajectory
	// = integrate times
	/**
	 * Output integration based on a given time trajectory
	 * @param [in] initialState
	 * @param [in] timeTrajectory
	 * @param [out] stateTrajectory
	 * @param [in] dtInitial
	 * @param [in] AbsTol
	 * @param [in] RelTol
	 * @return boolean
	 */
	virtual bool integrate(
			const State_T& initialState,
			const TimeTrajectory_T& timeTrajectory,
			StateTrajectory_T& stateTrajectory,
			double dtInitial = 0.01,
			double AbsTol = 1e-9,
			double RelTol = 1e-6
	) = 0;


protected:

	/**
	 * Retrieve trajectories from observer
	 * @param [out] stateTrajectory
	 * @param [out] timeTrajectory
	 */
	void retrieveTrajectoriesFromObserver(StateTrajectory_T& stateTrajectory, TimeTrajectory_T& timeTrajectory)
	{
		stateTrajectory.swap(observer_.stateTrajectory_);
		timeTrajectory.swap(observer_.timeTrajectory_);
	}

	/**
	 * Retrieves state trajectory from observer
	 * @param [out] stateTrajectory
	 */
	void retrieveStateTrajectoryFromObserver(StateTrajectory_T& stateTrajectory)
	{
		stateTrajectory.swap(observer_.stateTrajectory_);
	}

	Observer<STATE_DIM> observer_;

	std::shared_ptr<SystemBase<STATE_DIM> > system_;
	std::shared_ptr<EventHandler<STATE_DIM> > eventHandler_;
};

} // namespace ocs2

#endif /* OCS2INTEGRATORBASE_H_ */
