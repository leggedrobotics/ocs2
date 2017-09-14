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
 * The interface class for integration of autonomous systems.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class IntegratorBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::vector<double> TimeTrajectory_T;
	typedef Eigen::Matrix<double, STATE_DIM, 1> State_T;
	typedef std::vector<State_T, Eigen::aligned_allocator<State_T> > StateTrajectory_T;

	/**
	 * Constructor
	 * @param [in] system: The system dynamics.
	 * @param [in] eventHandler: The integration event function.
	 */
	IntegratorBase(
			const std::shared_ptr<SystemBase<STATE_DIM> >& system,
			const std::shared_ptr<EventHandler<STATE_DIM> >& eventHandler = nullptr) :
				observer_(eventHandler),
				system_(system),
				eventHandler_(eventHandler)
	{}

	/**
	 * Default destructor
	 */
	virtual ~IntegratorBase() {}

	/**
	 * Reset function
	 */
	virtual void reset() {
		observer_.reset();
	}

	/**
	 * Equidistant integration based on initial and final time as well as step length
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [in] dt: Time step.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @return boolean: Success flag.
	 */
	virtual bool integrate(
			const State_T& initialState,
			const double& startTime,
			const double& finalTime,
			double dt,
			StateTrajectory_T& stateTrajectory,
			TimeTrajectory_T& timeTrajectory
	) = 0;

	/**
	 * Adaptive time integration based on start time and final time
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolue tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
	 * @return boolean: Success flag.
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

	/**
	 * Output integration based on a given time trajectory
	 * @param [in] initialState: Initial state.
	 * @param [in] timeTrajectory: Input time stamp trajectory.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolue tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @return boolean: Success flag.
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
	 * Retrieve trajectories from observer.
	 *
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 */
	void retrieveTrajectoriesFromObserver(StateTrajectory_T& stateTrajectory, TimeTrajectory_T& timeTrajectory)
	{
		stateTrajectory.swap(observer_.stateTrajectory_);
		timeTrajectory.swap(observer_.timeTrajectory_);
	}

	/**
	 * Retrieves state trajectory from observer.
	 *
	 * @param [out] stateTrajectory: Output state trajectory.
	 */
	void retrieveStateTrajectoryFromObserver(StateTrajectory_T& stateTrajectory)
	{
		stateTrajectory.swap(observer_.stateTrajectory_);
	}

	/**
	 * Integrator observer for saving the variable of interest.
	 */
	Observer<STATE_DIM> observer_;

	/**
	 * System dynamics used by integrator.
	 */
	std::shared_ptr<SystemBase<STATE_DIM> > system_;

	/**
	 * Event handler used by integrator.
	 */
	std::shared_ptr<EventHandler<STATE_DIM> > eventHandler_;
};

} // namespace ocs2

#endif /* OCS2INTEGRATORBASE_H_ */
